# Hackaburg 2026

# Projekt Quackonomous

---

## Vision

Eine gewöhnliche Gummiente wird zum autonomen Wasserfahrzeug, gesteuert durch Embedded-Logik, navigiert selbstständig oder per Fernsteuerung.

Meilenstein 1: Ente mit Fernbedienung steuern (USB Modul, Steuerelement, ESP now)

Meilenstein 2: Hilfssysteme (Ultraschallsensor, e.g. Hindernis voraus: bremsen)

---

## Thema & Scope

| Bereich | Beschreibung |
| --- | --- |
| Fahrzeug | Gummiente mit Antrieb + Ruder |
| Steuerung | Autonom + manuelle Fernsteuerung |
| Domäne | Embedded Systems / Robotik |
| Hardware | Vollständig vorhanden  |

---

## Phasenplan

| Phase | Fokus | Status |
| --- | --- | --- |
| 1 — Setup | Hardware checken, Rollen verteilen | done |
| 2 — Core | Antrieb + Fernsteuerung |  |
| 3 — Autonomie | Sensorik + Navigation |  |
| 4 — Demo | Test, Pitch, Puffer |  |

---

## Aufgaben

### Phase 1 — Setup

- [x]  Hardware-Inventur: welche Komponenten sind da?
- [ ]  Rollen im Team verteilen
- [x]  Repo + Projektstruktur aufsetzen

### Phase 2 — Core

- [ ]  Antriebstest — Motor + Ruder auf Funktion prüfen
- [ ]  Microcontroller / SBC aufsetzen (Firmware / OS)
- [ ]  Fernsteuerung implementieren (Bluetooth / RF / WLAN)

### Phase 3 — Autonomie

- [ ]  Sensorik einbinden (Ultraschall / Kamera / GPS?)
- [ ]  Autonome Navigationslogik entwickeln
- [ ]  Wassertauglichkeit / Abdichtung prüfen

### Phase 4 — Demo

- [ ]  Demo-Lauf testen
- [ ]  GitHub README schreiben

---

## Team-Rollen

| Rolle | Verantwortung | Person |
| --- | --- | --- |
| Hardware / Mechanik | Antrieb, Verkabelung, Wasserdichtigkeit |  |
| Embedded / Firmware | Microcontroller, Treiber, Low-Level |  |
| Software / Autonomie | Navigationslogik, Sensorauswertung |  |
| Fernsteuerung / Protokoll | Kommunikation, RC-Interface |  |
| Präsentation / Doku | Demo, Pitch, README |  |

---

## Offene Fragen & Risiken

- [ ]  Stromversorgung im Wasser — Akku-Laufzeit?
- [ ]  Latenz bei Fernsteuerung akzeptabel?
- [ ]  Wie wird der Kurs definiert? (GPS-Wegpunkte / Linienverfolgung / Kompass)

---

## Links & Ressourcen

- Hardware-Doku:
- Hackathon-Regeln / Einreichung:

---

## Notizen