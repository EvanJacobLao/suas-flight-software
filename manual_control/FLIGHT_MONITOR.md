# Drone Dio manual-flight monitor

Prepared for Pixhawk 2.4.8 / ArduPlane, FlySky i6 pilot control in FBWA,
Jetson logging, and a laptop browser connected wirelessly to the Jetson.
The program is ready for software demonstration and subsequent bench integration;
real hardware and flight validation are still outstanding.

```text
FlySky i6 -> aircraft RC receiver -> Pixhawk (pilot control)
                                      |
                                USB or serial MAVLink
                                      |
                                   Jetson -> local CSV + JSONL logs
                                      |
                                wireless IP network
                                      |
                                laptop web browser
```

The receiver is identified as FS-iA6B in the handbook; verify the installed unit.
The monitor receives MAVLink only. It does not send heartbeats, stream requests,
parameter changes, RC overrides, arm/disarm commands, or mode changes. Select
FBWA with the configured transmitter switch. Displayed mode confirms what the
autopilot reports; the program does not select it for you.

## Try it now on Windows, without the aircraft or Jetson

From PowerShell in the project folder:

```powershell
python -m suas_autonomy.gcs.flight_monitor --demo
```

Open http://127.0.0.1:8080 in a browser. The dashboard prominently marks simulated
data as DEMO. Demo mode uses only the Python standard library. Stop with Ctrl+C.
Logs appear under `logs/manual_flight/demo_TIMESTAMP/` even if no browser is open.
Demo numbers are display fixtures, not flight limits or aircraft predictions.

To test over a private local network with a second device:

```powershell
python -m suas_autonomy.gcs.flight_monitor --demo --host 0.0.0.0
```

Open `http://SERVER_LOCAL_IP:8080` on that device. Use the server's actual IP,
not `0.0.0.0`. Allow inbound TCP 8080 on the private network if the firewall asks.

## Prepare the Jetson when it is available

Copy this project to the Jetson and run from its root directory. This module
uses Python 3.10+ and does not require OpenCV, CUDA, JetPack changes, or the full
project's dependency installation (the full package targets Python 3.11+).

```bash
python3 -m venv .monitor-venv
source .monitor-venv/bin/activate
python3 -m pip install 'pymavlink>=2.4.42,<3'
python3 -m suas_autonomy.gcs.flight_monitor --demo --host 0.0.0.0
```

Connect Jetson and laptop to the same private Wi-Fi network/router or a laptop
hotspot. The Jetson needs a functioning Wi-Fi adapter or another wireless IP
bridge; that hardware is not assumed to be present. No internet is required.
Find its local IP with `hostname -I`, then open `http://JETSON_IP:8080` on the
laptop. This HTTP service has no authentication or encryption: keep it on the
private local network and do not expose/port-forward it to the internet.

## Connect the Pixhawk later

USB is a candidate connection that avoids occupying a TELEM port. Confirm its
mechanical retention, power arrangement, and device enumeration on the actual
installation. Serial requires a verified free port, correct voltage/pinout, and
matching baud rate; the handbook's gimbal/companion port conflict remains open.

Identify the serial device with `ls -l /dev/serial/by-id/`. Substitute the actual
device path below; `PIXHAWK_DEVICE` is a placeholder, not a real device name.

```bash
python3 -m suas_autonomy.gcs.flight_monitor \
  --connect /dev/serial/by-id/PIXHAWK_DEVICE \
  --baud 57600 --system 1 --component 1 --host 0.0.0.0
```

The baud rate must match the selected serial link; 57600 comes from the
handbook's SiK setup and is not confirmation of the companion port's settings.
Ensure your Linux user has access to the device (commonly the `dialout` group).
Only one application should open that serial device. If a MAVLink router already
owns it, supply a dedicated receive endpoint instead, for example:

```bash
python3 -m suas_autonomy.gcs.flight_monitor \
  --connect udpin:0.0.0.0:14550 --host 0.0.0.0
```

For routed input, use a dedicated telemetry-only feed from the aircraft. The
monitor filters the configured system and component IDs (defaults 1/1); it does
not automatically switch to another aircraft. This is not cryptographic source
authentication. UDP listeners should be restricted to the intended private link.

Because this is receive-only, the Pixhawk/router must already stream telemetry.
Using Mission Planner, configure the relevant port's MAVLink stream rates after
identifying the firmware and port mapping. Do not guess the SRx index from the
physical port label. Aim for HEARTBEAT at 1 Hz; ATTITUDE, GLOBAL_POSITION_INT,
VFR_HUD at 2–5 Hz; SYS_STATUS, GPS_RAW_INT and RC_CHANNELS at 1–2 Hz;
STATUSTEXT when events occur. Available stream groups vary with firmware.
Missing streams remain unavailable/stale rather than showing invented values.
The app reconnects after receive/open exceptions and marks heartbeat loss after
3 seconds. A silent but open connection stays open awaiting data.

## Display and logs

- Browser updates roughly twice per second. Each field has its own age; after
  3 seconds without updates it is shown as stale. Heartbeat freshness is separate
  from GPS, battery, and attitude freshness.
- A laptop-network outage blanks the live cards and retains the last track.
  The Jetson continues logging if its process and Pixhawk link are still running.
- Flight mode, arm state, airspeed/groundspeed, altitude, attitude, heading,
  reported throttle, GPS fix/satellites, battery voltage/current/remaining,
  and recent autopilot text are displayed. Unknown battery values stay unknown.
- Relative altitude is relative to home, not height above local terrain.
  Airspeed is autopilot-reported and may be estimated. A 3D GPS fix and a live
  heartbeat do not establish overall flight readiness.
- The offline track uses a local metre projection with north up. It is a
  visualization, not a geofence or navigation map.
- A new directory is created for each run. CSV stores sampled data at 2 Hz,
  leaving stale fields blank. JSONL also preserves field ages, last values,
  recent status messages, and connection/logging errors. These are sampled
  monitor logs, not raw MAVLink tlogs or replacements for Pixhawk DataFlash logs.
- Host UTC timestamps are reception/sample times, not synchronized flight
  controller timestamps. Set the Jetson clock before testing.
- Logs are flushed every sample; abrupt power loss can still lose buffered
  storage writes. A write failure is shown prominently and logging stops;
  restart with working storage after resolving it. No automatic log deletion.
- No automatic low-battery, stall, geofence, or flight-readiness thresholds are
  assumed. The handbook's weight/wing-area conflict remains unresolved.

## Hardware acceptance checks before a flight

1. With the propeller removed, compare mode, arm state, attitude, battery, GPS,
   and altitude with Mission Planner. Confirm all required streams update.
2. Verify i6 channel calibration, control-surface direction, FBWA correction
   direction and mode switching independently of this monitor.
3. Stop the monitor and disconnect its laptop network: verify RC control remains
   available. Confirm the aircraft's configured RC/GCS/battery failsafe behavior
   separately; this monitor does not provide a GCS heartbeat or failsafe action.
4. Disconnect/reconnect the telemetry cable on the bench. Confirm stale warnings,
   recovery, and logs. Disconnect laptop Wi-Fi while leaving Jetson logging,
   then reconnect and verify the log spans the outage.
5. Verify the intended wireless link at the planned operating distance. A laptop
   hotspot demonstration does not establish airborne coverage.
6. Check exact ArduPlane firmware, parameter export, battery monitoring, aircraft
   geometry/CG and outdoor GPS results with the pilot before flight. FBWA still
   requires the pilot to manage throttle and airspeed; it does not guarantee
   stall prevention.

## Information still needed for hardware setup

Exact ArduPlane version and parameter export; Jetson OS/Python version;
Pixhawk-to-Jetson port/device and baud; installed wireless adapter/network and
range requirement; confirmation of the receiver, mode switch and failsafe tests.
None of these is needed to run the demo now.

## References

- MAVLink field units and unknown values: https://mavlink.io/en/messages/common.html
- Stream configuration: https://ardupilot.org/dev/docs/mavlink-requesting-data.html
- FBWA behavior: https://ardupilot.org/plane/docs/fbwa-mode.html
