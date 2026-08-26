"""
mappingPlaneController.py
--------------------------
Manual fixed-wing plane control (via a game controller) + 2D trajectory
map, driven over MAVLink against ArduPlane SITL.

Same MAVLink/telemetry/mapping approach as mappingPlane.py - the only real
change is the INPUT SOURCE: a physical game controller (read via pygame)
instead of keyboard key-holds. A real analog stick gives smooth, spring-
centred input for free, so a lot of the keyboard version's "simulate a
real stick" logic goes away.

Controls (Mode-2-style, like a real RC transmitter):
    Left stick  Y-axis  - throttle (holds position, no spring-back)
    Left stick  X-axis  - rudder
    Right stick X-axis  - aileron (roll -> turns the plane)
    Right stick Y-axis  - elevator (pitch -> climb / descend)
    Keyboard ESC        - cut throttle, disarm, quit (kept as a manual
                           kill-switch independent of the controller)

SAFETY / FAILSAFE: if the controller disconnects mid-flight, throttle is
cut automatically each tick until it reconnects - this does NOT replace a
real RC failsafe/RTL setup, it's just a minimal "stop feeding power on
stale input" guard for SITL testing.

UNVERIFIED: axis numbering (which index = which physical stick) varies by
controller and OS driver. Run this once first and watch the printed axis
values while you move each stick - fix AXIS_* constants below to match
before trusting the mapping.
"""

import time
import math

import cv2
import numpy as np
import pygame
from pynput import keyboard
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
MAV_CONN_STRING = "tcp:127.0.0.1:5762"
PIXELS_PER_METRE = 20
CENTRE_X, CENTRE_Y = 500, 500

FLIGHT_MODE = "FBWA"

PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

AILERON_DEFLECTION = 300
ELEVATOR_DEFLECTION = -300  # UNVERIFIED sign - flip if pitch is backwards
RUDDER_DEFLECTION = 300

CH_AILERON = 1
CH_ELEVATOR = 2
CH_THROTTLE = 3
CH_RUDDER = 4

# Controller axis mapping - VERIFY these against your controller (see the
# diagnostic printout at startup) before flying.
AXIS_THROTTLE = 1   # left stick Y
AXIS_RUDDER = 0      # left stick X
AXIS_AILERON = 2     # right stick X
AXIS_ELEVATOR = 3    # right stick Y

AXIS_DEADZONE = 0.08  # ignore small stick drift near centre


def apply_deadzone(value, deadzone=AXIS_DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def axis_to_control_pwm(axis_value, deflection, invert=False):
    """Map a -1..1 stick axis to a PWM offset from neutral."""
    value = -axis_value if invert else axis_value
    value = apply_deadzone(value)
    return int(PWM_NEUTRAL + value * deflection)


def axis_to_throttle_pwm(axis_value, invert=True):
    """Map a -1..1 stick axis to PWM_MIN..PWM_MAX (no neutral centre)."""
    value = -axis_value if invert else axis_value  # pygame Y axes: up = -1
    value = apply_deadzone(value)
    normalised = (value + 1.0) / 2.0  # -1..1 -> 0..1
    return int(PWM_MIN + normalised * (PWM_MAX - PWM_MIN))


# ---------------------------------------------------------------------------
# Controller setup
# ---------------------------------------------------------------------------
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise RuntimeError("No game controller detected. Plug it in / pair it, then re-run.")

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Controller detected: {joystick.get_name()}")
print(f"Axes available: {joystick.get_numaxes()}")
print("Move each stick now - watch which axis index changes for each one,")
print("then fix AXIS_THROTTLE / AXIS_RUDDER / AXIS_AILERON / AXIS_ELEVATOR above if needed.")
for _ in range(60):  # ~3 seconds of live axis readout at the start
    pygame.event.pump()
    values = [round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())]
    print(values)
    time.sleep(0.05)

# ---------------------------------------------------------------------------
# Connect to SITL
# ---------------------------------------------------------------------------
print(f"Connecting to SITL at {MAV_CONN_STRING} ...")
mav = mavutil.mavlink_connection(MAV_CONN_STRING)
mav.wait_heartbeat()
print(f"Heartbeat received (system {mav.target_system}, component {mav.target_component})")


def request_message_interval(mav_connection, message_id, frequency_hz):
    mav_connection.mav.command_long_send(
        mav_connection.target_system, mav_connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, int(1e6 / frequency_hz), 0, 0, 0, 0, 0,
    )


request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 20)
request_message_interval(mav, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20)

# ---------------------------------------------------------------------------
# Telemetry state
# ---------------------------------------------------------------------------
north, east, down = 0.0, 0.0, 0.0
vx, vy, vz = 0.0, 0.0, 0.0
att_yaw = 0.0


def pullTelemetry():
    global north, east, down, vx, vy, vz, att_yaw
    while True:
        msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ATTITUDE"], blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()
        if msg_type == "LOCAL_POSITION_NED":
            north, east, down = msg.x, msg.y, msg.z
            vx, vy, vz = msg.vx, msg.vy, msg.vz
        elif msg_type == "ATTITUDE":
            att_yaw = msg.yaw


# ---------------------------------------------------------------------------
# Flight control helpers
# ---------------------------------------------------------------------------
def set_mode(mode_name):
    mode_id = mav.mode_mapping()[mode_name]
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    print(f"Mode set to {mode_name}")


def arm():
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0,
    )
    mav.motors_armed_wait()
    print("Armed.")


def disarm():
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        0, 0, 0, 0, 0, 0, 0,
    )
    print("Disarmed.")


def send_rc_override(aileron_pwm, elevator_pwm, throttle_pwm, rudder_pwm):
    channels = [65535] * 18  # 65535 = "leave this channel alone"
    channels[CH_AILERON - 1] = aileron_pwm
    channels[CH_ELEVATOR - 1] = elevator_pwm
    channels[CH_THROTTLE - 1] = throttle_pwm
    channels[CH_RUDDER - 1] = rudder_pwm
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, *channels)


# ---------------------------------------------------------------------------
# Keyboard - kept ONLY as a manual kill-switch, independent of the controller
# ---------------------------------------------------------------------------
status = True


def on_press(key):
    global status
    if key == keyboard.Key.esc:
        status = False


listener = keyboard.Listener(on_press=on_press)
listener.start()

# ---------------------------------------------------------------------------
# Map drawing
# ---------------------------------------------------------------------------
points = [(CENTRE_X, CENTRE_Y)]


def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 4, (0, 0, 255), cv2.FILLED)

    last_point = points[-1]
    ground_speed = math.sqrt(vx ** 2 + vy ** 2)

    cv2.putText(
        img,
        f'N:{north:.1f}m E:{east:.1f}m alt:{-down:.1f}m spd:{ground_speed:.1f}m/s',
        (last_point[0] + 10, last_point[1] + 30),
        cv2.FONT_HERSHEY_PLAIN, 1, (255, 0, 255), 1,
    )

    heading_len = 20
    hx = int(last_point[0] + heading_len * math.sin(att_yaw))
    hy = int(last_point[1] - heading_len * math.cos(att_yaw))
    cv2.line(img, last_point, (hx, hy), (0, 255, 0), 2)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
set_mode(FLIGHT_MODE)

for _ in range(20):
    send_rc_override(PWM_NEUTRAL, PWM_NEUTRAL, PWM_MIN, PWM_NEUTRAL)
    time.sleep(0.05)

arm()
print("Armed and ready. Push the throttle stick up to begin the takeoff roll.")

try:
    while status:
        pygame.event.pump()

        if pygame.joystick.get_count() == 0:
            # Controller disconnected mid-flight - fail safe: cut throttle,
            # keep surfaces neutral, do NOT disarm mid-air on its own.
            print("WARNING: controller disconnected - cutting throttle.")
            send_rc_override(PWM_NEUTRAL, PWM_NEUTRAL, PWM_MIN, PWM_NEUTRAL)
        else:
            aileron_pwm = axis_to_control_pwm(joystick.get_axis(AXIS_AILERON), AILERON_DEFLECTION)
            elevator_pwm = axis_to_control_pwm(joystick.get_axis(AXIS_ELEVATOR), ELEVATOR_DEFLECTION)
            rudder_pwm = axis_to_control_pwm(joystick.get_axis(AXIS_RUDDER), RUDDER_DEFLECTION)
            throttle_pwm = axis_to_throttle_pwm(joystick.get_axis(AXIS_THROTTLE))
            send_rc_override(aileron_pwm, elevator_pwm, throttle_pwm, rudder_pwm)

        pullTelemetry()

        map_x = CENTRE_X + east * PIXELS_PER_METRE
        map_y = CENTRE_Y - north * PIXELS_PER_METRE
        current_point = (int(map_x), int(map_y))
        if points[-1] != current_point:
            points.append(current_point)

        img = np.zeros((1000, 1000, 3), np.uint8)
        drawPoints(img, points)
        cv2.imshow("Map", img)
        cv2.waitKey(1)

        time.sleep(0.05)

except KeyboardInterrupt:
    pass
finally:
    print("Cutting throttle and disarming...")
    send_rc_override(PWM_NEUTRAL, PWM_NEUTRAL, PWM_MIN, PWM_NEUTRAL)
    time.sleep(0.5)
    disarm()
    listener.stop()
    cv2.destroyAllWindows()
    pygame.quit()
