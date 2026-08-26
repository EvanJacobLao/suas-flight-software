# ArduPlane Controller and Mapping

A Python program for manually controlling an ArduPlane SITL simulation
with a game controller while displaying its trajectory on a 2D map.

## Safety

This project is intended for ArduPilot SITL simulation only. It has not
been prepared or verified for use with a real aircraft.

## Requirements

- Python 3
- ArduPilot SITL
- Mission Planner
- A compatible game controller

## Installation

Install the Python dependencies:

```bash
pip install -r requirements.txt

Running
Start ArduPlane SITL and then run:
python mappingPlaneController.py
Controls
- Left stick Y: throttle
- Left stick X: rudder
- Right stick X: aileron
- Right stick Y: elevator
- Escape: cut throttle, disarm, and exit
Controller axis numbers may differ between controllers and operating systems.
Check the configured axis numbers before flying.