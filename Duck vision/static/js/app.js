const socket = io();
const fields = [
	"r_min",
	"r_max",
	"g_min",
	"g_max",
	"b_min",
	"b_max",
	"min_area",
	"blue_r_min",
	"blue_r_max",
	"blue_g_min",
	"blue_g_max",
	"blue_b_min",
	"blue_b_max",
	"blue_min_area",
	"blue_smoothing",
];
let updateTimer = null;

function setOutput(id, value) {
	document.getElementById(`${id}_val`).textContent = value;
}

function currentValues() {
	return {
		r_min: Number(document.getElementById("r_min").value),
		r_max: Number(document.getElementById("r_max").value),
		g_min: Number(document.getElementById("g_min").value),
		g_max: Number(document.getElementById("g_max").value),
		b_min: Number(document.getElementById("b_min").value),
		b_max: Number(document.getElementById("b_max").value),
		min_area: Number(document.getElementById("min_area").value),
		blue_r_min: Number(document.getElementById("blue_r_min").value),
		blue_r_max: Number(document.getElementById("blue_r_max").value),
		blue_g_min: Number(document.getElementById("blue_g_min").value),
		blue_g_max: Number(document.getElementById("blue_g_max").value),
		blue_b_min: Number(document.getElementById("blue_b_min").value),
		blue_b_max: Number(document.getElementById("blue_b_max").value),
		blue_min_area: Number(document.getElementById("blue_min_area").value),
		blue_smoothing: Number(document.getElementById("blue_smoothing").value),
	};
}

function bindSlider(id) {
	const el = document.getElementById(id);
	el.addEventListener("input", () => {
		setOutput(id, el.value);
		if (updateTimer) {
			clearTimeout(updateTimer);
		}
		updateTimer = setTimeout(() => {
			socket.emit("update_params", currentValues());
		}, 30);
	});
}

function applyState(data) {
	if (data && data.params) {
		const params = data.params;
		for (const key of fields) {
			if (params[key] !== undefined) {
				document.getElementById(key).value = params[key];
				setOutput(key, params[key]);
			}
		}
	}

	if (data && data.center) {
		document.getElementById("center").textContent =
			`(${data.center[0]}, ${data.center[1]})`;
	} else {
		document.getElementById("center").textContent = "-";
	}

	document.getElementById("area").textContent =
		data && data.area !== undefined ? data.area : "-";
	document.getElementById("frame_size").textContent =
		data && data.frame_size
			? `${data.frame_size[0]} x ${data.frame_size[1]}`
			: "-";
	document.getElementById("error").textContent =
		data && data.error ? data.error : "";

	// blue info
	if (data && data.blue_centers) {
		document.getElementById("blue1").textContent = data.blue_centers[0]
			? `(${data.blue_centers[0][0]}, ${data.blue_centers[0][1]})`
			: "-";
		document.getElementById("blue2").textContent = data.blue_centers[1]
			? `(${data.blue_centers[1][0]}, ${data.blue_centers[1][1]})`
			: "-";
	}

	document.getElementById("blue_line_x").textContent =
		data && data.blue_line_x !== undefined && data.blue_line_x !== null
			? String(data.blue_line_x)
			: "-";
	document.getElementById("red_offset").textContent =
		data &&
		data.red_offset_to_blue_line !== undefined &&
		data.red_offset_to_blue_line !== null
			? String(data.red_offset_to_blue_line)
			: "-";
}

socket.on("connect", () => {
	document.getElementById("status").textContent = "Verbunden";
	socket.emit("request_state");
});

socket.on("disconnect", () => {
	document.getElementById("status").textContent = "Getrennt";
});

socket.on("settings_state", (data) => {
	applyState(data);
});

socket.on("position_update", (data) => {
	document.getElementById("center").textContent = data.center
		? `(${data.center[0]}, ${data.center[1]})`
		: "-";
	document.getElementById("area").textContent =
		data.area !== undefined ? data.area : "-";
	document.getElementById("frame_size").textContent = data.frame_size
		? `${data.frame_size[0]} x ${data.frame_size[1]}`
		: "-";
	document.getElementById("error").textContent = data.error ? data.error : "";
	if (data.blue_centers) {
		document.getElementById("blue1").textContent = data.blue_centers[0]
			? `(${data.blue_centers[0][0]}, ${data.blue_centers[0][1]})`
			: "-";
		document.getElementById("blue2").textContent = data.blue_centers[1]
			? `(${data.blue_centers[1][0]}, ${data.blue_centers[1][1]})`
			: "-";
	}
	document.getElementById("blue_line_x").textContent =
		data.blue_line_x !== undefined && data.blue_line_x !== null
			? String(data.blue_line_x)
			: "-";
	document.getElementById("red_offset").textContent =
		data.red_offset_to_blue_line !== undefined &&
		data.red_offset_to_blue_line !== null
			? String(data.red_offset_to_blue_line)
			: "-";
});

for (const id of fields) {
	bindSlider(id);
}
