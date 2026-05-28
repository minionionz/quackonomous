import argparse
import json
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, render_template
from flask_socketio import SocketIO, emit


SETTINGS_FILE = Path(__file__).with_name("settings.json")
BASE_DIR = Path(__file__).resolve().parent


def parse_source(source_raw: str):
    if source_raw.isdigit():
        return int(source_raw)
    return source_raw


def clamp_rgb(value: int) -> int:
    return max(0, min(255, value))


def clamp_area(value: int) -> int:
    return max(1, min(500000, value))


def clamp_percent(value: int) -> int:
    return max(1, min(100, value))


@dataclass
class DetectionParams:
    r_min: int = 60
    g_min: int = 0
    b_min: int = 0
    r_max: int = 255
    g_max: int = 31
    b_max: int = 35
    min_area: int = 500
    # Blue detection params (two largest)
    blue_r_min: int = 0
    blue_g_min: int = 0
    blue_b_min: int = 100
    blue_r_max: int = 120
    blue_g_max: int = 120
    blue_b_max: int = 255
    blue_min_area: int = 200
    blue_smoothing: int = 18

    @classmethod
    def from_dict(cls, data: dict):
        defaults = cls()
        return cls(
            r_min=clamp_rgb(int(data.get("r_min", defaults.r_min))),
            g_min=clamp_rgb(int(data.get("g_min", defaults.g_min))),
            b_min=clamp_rgb(int(data.get("b_min", defaults.b_min))),
            r_max=clamp_rgb(int(data.get("r_max", defaults.r_max))),
            g_max=clamp_rgb(int(data.get("g_max", defaults.g_max))),
            b_max=clamp_rgb(int(data.get("b_max", defaults.b_max))),
            min_area=clamp_area(int(data.get("min_area", defaults.min_area))),
            blue_r_min=clamp_rgb(int(data.get("blue_r_min", defaults.blue_r_min))),
            blue_g_min=clamp_rgb(int(data.get("blue_g_min", defaults.blue_g_min))),
            blue_b_min=clamp_rgb(int(data.get("blue_b_min", defaults.blue_b_min))),
            blue_r_max=clamp_rgb(int(data.get("blue_r_max", defaults.blue_r_max))),
            blue_g_max=clamp_rgb(int(data.get("blue_g_max", defaults.blue_g_max))),
            blue_b_max=clamp_rgb(int(data.get("blue_b_max", defaults.blue_b_max))),
            blue_min_area=clamp_area(
                int(data.get("blue_min_area", defaults.blue_min_area))
            ),
            blue_smoothing=clamp_percent(
                int(data.get("blue_smoothing", defaults.blue_smoothing))
            ),
        )

    def to_bgr_bounds(self):
        lower = np.array([self.b_min, self.g_min, self.r_min], dtype=np.uint8)
        upper = np.array([self.b_max, self.g_max, self.r_max], dtype=np.uint8)
        return lower, upper

    def to_bgr_bounds_blue(self):
        lower = np.array(
            [self.blue_b_min, self.blue_g_min, self.blue_r_min], dtype=np.uint8
        )
        upper = np.array(
            [self.blue_b_max, self.blue_g_max, self.blue_r_max], dtype=np.uint8
        )
        return lower, upper


class VideoProcessor:
    def __init__(self, source, settings_path: Path):
        self.source = source
        self.settings_path = settings_path
        self.params = DetectionParams()
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.capture = None
        self.socketio: Optional[SocketIO] = None
        self.latest_jpeg = None
        self.latest_center = None
        self.latest_area = 0
        self.latest_error = None
        self.latest_frame_size = None
        self.latest_blue_centers: list[tuple[int, int]] = []
        self.latest_blue_areas: list[int] = []
        self.latest_blue_line_x: Optional[int] = None
        self.latest_red_offset_to_blue_line: Optional[int] = None
        self._blue_smoothed_centers: list[Optional[tuple[float, float]]] = [None, None]

        self._load_settings()

    def set_socketio(self, socketio: SocketIO):
        self.socketio = socketio

    def _load_settings(self):
        if not self.settings_path.exists():
            self._save_settings()
            return

        try:
            data = json.loads(self.settings_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            self._save_settings()
            return

        params_data = data.get("params", data)
        with self.lock:
            self.params = DetectionParams.from_dict(params_data)

    def _save_settings(self):
        payload = {
            "params": asdict(self.params),
        }
        self.settings_path.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
        )

    def _configure_native_resolution(self):
        if not isinstance(self.source, int):
            return

        candidates = [
            (3840, 2160),
            (2560, 1440),
            (1920, 1080),
            (1600, 1200),
            # (1280, 720),
            # (1024, 768),
            # (800, 600),
            # (640, 480),
        ]

        for width, height in candidates:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def start(self):
        if self.running:
            return

        self.capture = cv2.VideoCapture(self.source)
        if not self.capture.isOpened():
            raise RuntimeError("Videostream konnte nicht geoeffnet werden.")

        self._configure_native_resolution()

        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        if self.capture is not None:
            self.capture.release()

    def update_params(self, data: dict):
        with self.lock:
            for key in [
                "r_min",
                "g_min",
                "b_min",
                "r_max",
                "g_max",
                "b_max",
                "min_area",
                "blue_r_min",
                "blue_g_min",
                "blue_b_min",
                "blue_r_max",
                "blue_g_max",
                "blue_b_max",
                "blue_min_area",
                "blue_smoothing",
            ]:
                if key in data:
                    value = int(data[key])
                    if key == "min_area":
                        setattr(self.params, key, clamp_area(value))
                    elif key == "blue_smoothing":
                        setattr(self.params, key, clamp_percent(value))
                    else:
                        setattr(self.params, key, clamp_rgb(value))
            self._save_settings()

    def snapshot(self):
        with self.lock:
            return {
                "params": asdict(self.params),
                "center": self.latest_center,
                "area": self.latest_area,
                "blue_centers": self.latest_blue_centers,
                "blue_areas": self.latest_blue_areas,
                "blue_line_x": self.latest_blue_line_x,
                "red_offset_to_blue_line": self.latest_red_offset_to_blue_line,
                "error": self.latest_error,
                "frame_size": self.latest_frame_size,
            }

    def get_jpeg(self):
        with self.lock:
            return self.latest_jpeg

    def _emit_position(self, payload: dict):
        if self.socketio is not None:
            self.socketio.emit("position_update", payload)

    def _smooth_center(
        self,
        prev: Optional[tuple[float, float]],
        current: tuple[int, int],
        alpha: float,
    ) -> tuple[float, float]:
        if prev is None:
            return (float(current[0]), float(current[1]))
        a = alpha
        return (
            prev[0] * (1.0 - a) + current[0] * a,
            prev[1] * (1.0 - a) + current[1] * a,
        )

    def _loop(self):
        while self.running:
            ok, frame = self.capture.read()
            if not ok:
                with self.lock:
                    self.latest_error = "Kein weiteres Bild vom Stream erhalten."
                self._emit_position(
                    {
                        "center": None,
                        "area": 0,
                        "blue_centers": [],
                        "blue_areas": [],
                        "blue_line_x": None,
                        "red_offset_to_blue_line": None,
                        "frame_size": self.latest_frame_size,
                        "error": self.latest_error,
                    }
                )
                time.sleep(0.05)
                continue

            frame_size = (int(frame.shape[1]), int(frame.shape[0]))

            with self.lock:
                params = DetectionParams(**asdict(self.params))
            smoothing_alpha = params.blue_smoothing / 100.0

            lower, upper = params.to_bgr_bounds()
            mask = cv2.inRange(frame, lower, upper)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            largest = None
            largest_area = 0.0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > largest_area and area >= params.min_area:
                    largest = contour
                    largest_area = area

            output = frame.copy()
            center = None

            if largest is not None:
                x, y, w, h = cv2.boundingRect(largest)
                moments = cv2.moments(largest)

                if moments["m00"] != 0:
                    cx = int(moments["m10"] / moments["m00"])
                    cy = int(moments["m01"] / moments["m00"])
                    center = (cx, cy)

                    cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(output, (cx, cy), 6, (255, 0, 0), -1)
                    cv2.drawContours(output, [largest], -1, (0, 255, 255), 2)

            # --- Blue detection: find two largest blue contours ---
            lower_b, upper_b = params.to_bgr_bounds_blue()
            mask_b = cv2.inRange(frame, lower_b, upper_b)
            mask_b = cv2.morphologyEx(mask_b, cv2.MORPH_OPEN, kernel)
            mask_b = cv2.morphologyEx(mask_b, cv2.MORPH_CLOSE, kernel)
            contours_b, _ = cv2.findContours(
                mask_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # sort by area desc and take two largest above blue_min_area
            contours_b = sorted(contours_b, key=cv2.contourArea, reverse=True)
            blue_candidates: list[tuple[int, int, int, np.ndarray]] = []
            for contour in contours_b:
                area_b = cv2.contourArea(contour)
                if area_b < params.blue_min_area:
                    continue
                M = cv2.moments(contour)
                if M["m00"] == 0:
                    continue
                bx = int(M["m10"] / M["m00"])
                by = int(M["m01"] / M["m00"])
                blue_candidates.append((bx, by, int(area_b), contour))
                if len(blue_candidates) == 2:
                    break

            # Keep marker identity stable as left/right and smooth centers over time.
            blue_candidates.sort(key=lambda c: c[0])
            blue_centers = []
            blue_areas = []
            for i in range(2):
                if i >= len(blue_candidates):
                    self._blue_smoothed_centers[i] = None
                    continue

                bx, by, area_b, contour = blue_candidates[i]
                smoothed = self._smooth_center(
                    self._blue_smoothed_centers[i], (bx, by), smoothing_alpha
                )
                self._blue_smoothed_centers[i] = smoothed
                sx, sy = int(smoothed[0]), int(smoothed[1])
                blue_centers.append((sx, sy))
                blue_areas.append(area_b)

                # draw first and second blue with different colors
                col = (255, 0, 0) if i == 0 else (200, 100, 0)
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(output, (x, y), (x + w, y + h), col, 2)
                cv2.circle(output, (sx, sy), 7, col, -1)
                cv2.drawContours(output, [contour], -1, (255, 255, 0), 2)

            blue_line_x = None
            red_offset_to_blue_line = None
            if len(blue_centers) == 2:
                # Vertical reference line at the midpoint of both blue blob centers.
                blue_line_x = int((blue_centers[0][0] + blue_centers[1][0]) / 2)
                cv2.line(
                    output,
                    (blue_line_x, 0),
                    (blue_line_x, frame_size[1] - 1),
                    (0, 255, 255),
                    2,
                )
                if center is not None:
                    red_offset_to_blue_line = int(center[0] - blue_line_x)

            if red_offset_to_blue_line is not None:
                cv2.putText(
                    output,
                    f"Offset X: {red_offset_to_blue_line}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            success, jpeg = cv2.imencode(".jpg", output)
            if success:
                with self.lock:
                    self.latest_jpeg = jpeg.tobytes()
                    self.latest_center = center
                    self.latest_area = int(largest_area)
                    self.latest_blue_centers = blue_centers
                    self.latest_blue_areas = blue_areas
                    self.latest_blue_line_x = blue_line_x
                    self.latest_red_offset_to_blue_line = red_offset_to_blue_line
                    self.latest_error = None
                    self.latest_frame_size = frame_size
                self._emit_position(
                    {
                        "center": center,
                        "area": int(largest_area),
                        "blue_centers": blue_centers,
                        "blue_areas": blue_areas,
                        "blue_line_x": blue_line_x,
                        "red_offset_to_blue_line": red_offset_to_blue_line,
                        "frame_size": frame_size,
                        "error": None,
                    }
                )

            time.sleep(0.01)


def build_app(processor: VideoProcessor):
    app = Flask(
        __name__,
        template_folder=str(BASE_DIR / "templates"),
        static_folder=str(BASE_DIR / "static"),
    )
    socketio = SocketIO(
        app,
        cors_allowed_origins="*",
        async_mode="threading",
        logger=False,
        engineio_logger=False,
    )
    processor.set_socketio(socketio)

    @socketio.on("connect")
    def handle_connect():
        emit("settings_state", processor.snapshot())

    @socketio.on("request_state")
    def handle_request_state():
        emit("settings_state", processor.snapshot())

    @socketio.on("update_params")
    def handle_update_params(data):
        processor.update_params(data or {})
        emit("settings_state", processor.snapshot(), broadcast=True)

    @app.route("/")
    def index():
        return render_template("index.html")

    @app.route("/video_feed")
    def video_feed():
        def generate():
            while True:
                frame = processor.get_jpeg()
                if frame is None:
                    time.sleep(0.05)
                    continue
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                time.sleep(0.03)

        return Response(
            generate(), mimetype="multipart/x-mixed-replace; boundary=frame"
        )

    @app.route("/api/state")
    def api_state():
        return jsonify(processor.snapshot())

    @app.route("/api/params", methods=["POST"])
    def api_params():
        data = request.get_json(force=True, silent=True) or {}
        processor.update_params(data)
        return jsonify({"ok": True, "params": processor.snapshot()["params"]})

    return app, socketio


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Weboberfläche fuer Live-Farberkennung mit Socket-Update und JSON-Speicherung."
    )
    parser.add_argument(
        "--source",
        default="1",
        help="Videoquelle: Webcam-Index (z.B. 0) oder Datei/RTSP-URL",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Host fuer den Webserver")
    parser.add_argument(
        "--port", default=5000, type=int, help="Port fuer den Webserver"
    )
    args = parser.parse_args()

    processor = VideoProcessor(parse_source(args.source), SETTINGS_FILE)
    app, socketio = build_app(processor)

    processor.start()

    try:
        print(f"Webinterface laeuft auf http://{args.host}:{args.port}")
        socketio.run(
            app,
            host=args.host,
            port=args.port,
            debug=False,
            allow_unsafe_werkzeug=True,
        )
    finally:
        processor.stop()


if __name__ == "__main__":
    main()
