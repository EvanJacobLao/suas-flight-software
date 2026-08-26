"""
Mission Planner connects on the primary MAVLink endpoint (tcp:127.0.0.1:5760).
This script connects on the secondary endpoint (tcp:127.0.0.1:5762).

Controls:
    w / s     - throttle up / down (holds position, does not spring back)
    a / d     - aileron left / right  (roll -> turns the plane)
    up / down - elevator up / down    (pitch -> climb / descend)
    q / e     - rudder left / right
    esc       - cut throttle, disarm, quit

Flight mode is FBWA (Fly-By-Wire A): you fly by hand, but ArduPilot still
prevents stalls/over-banking. Switch to "MANUAL" mode below once you're
comfortable flying without that safety net.

SAFETY: written for SITL only. No failsafe handling, no command-ack
timeouts/retries. Do not point this at real hardware without adding those.

UNVERIFIED: this has not been run against a live SITL instance. In
particular, ELEVATOR_DEFLECTION's sign (does higher PWM mean nose-up or
nose-down?) is a common point of confusion across ArduPilot configs - if
the plane pitches the opposite way you expect, flip its sign below.
"""

import time
import math

import cv2
import numpy as np
from pynput import keyboard
from pymavlink import mavutil

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
MAV_CONN_STRING = "tcp:127.0.0.1:5762"
PIXELS_PER_METRE = 20   # planes cover ground much faster than the drone did,
                         # so the map is zoomed out compared to the copter version
CENTRE_X, CENTRE_Y = 500, 500

FLIGHT_MODE = "FBWA"    # switch to "MANUAL" for raw stick passthrough, no assist

# RC PWM values. 1500 = centre/neutral for aileron, elevator, rudder.
# Throttle uses 1000 (no power) .. 2000 (full power), no centre.
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

AILERON_DEFLECTION = 300   # how far 'a'/'d' push the aileron stick from centre
ELEVATOR_DEFLECTION = -300  # NEGATIVE here: on this build, lower PWM pitched
                             # nose up. UNVERIFIED - flip the sign if backwards.
RUDDER_DEFLECTION = 300
THROTTLE_STEP = 15          # PWM change per loop tick while 'w'/'s' held

# RC channel numbers (ArduPlane default mapping)
CH_AILERON = 1
CH_ELEVATOR = 2
CH_THROTTLE = 3
CH_RUDDER = 4

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
# Telemetry state - updated every tick by pullTelemetry()
# ---------------------------------------------------------------------------
north, east, down = 0.0, 0.0, 0.0
vx, vy, vz = 0.0, 0.0, 0.0
att_yaw = 0.0  # radians; 0 = North, +ve = clockwise (East)


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
    """Send RC stick positions, exactly like a real transmitter would.
    Must be resent every loop tick - ArduPilot expects a steady stream of
    RC input and will fail safe if it stops hearing from you."""
    channels = [65535] * 18  # 65535 = "leave this channel alone"
    channels[CH_AILERON - 1] = aileron_pwm
    channels[CH_ELEVATOR - 1] = elevator_pwm
    channels[CH_THROTTLE - 1] = throttle_pwm
    channels[CH_RUDDER - 1] = rudder_pwm
    mav.mav.rc_channels_override_send(mav.target_system, mav.target_component, *channels)


# ---------------------------------------------------------------------------
# Keyboard state
# ---------------------------------------------------------------------------
pressed_keys = set()
status = True
throttle_pwm = PWM_MIN  # persists across ticks - throttle doesn't spring back


def on_press(key):
    global status
    try:
        pressed_keys.add(key.char)
    except AttributeError:
        if key == keyboard.Key.up:
            pressed_keys.add("up")
        elif key == keyboard.Key.down:
            pressed_keys.add("down")
        elif key == keyboard.Key.esc:
            status = False


def on_release(key):
    try:
        pressed_keys.discard(key.char)
    except AttributeError:
        if key == keyboard.Key.up:
            pressed_keys.discard("up")
        elif key == keyboard.Key.down:
            pressed_keys.discard("down")


def compute_rc_values():
    """Translate held keys into PWM stick positions for this tick."""
    global throttle_pwm

    # Throttle holds its position - only changes while w/s is actually held
    if "w" in pressed_keys:
        throttle_pwm = min(PWM_MAX, throttle_pwm + THROTTLE_STEP)
    if "s" in pressed_keys:
        throttle_pwm = max(PWM_MIN, throttle_pwm - THROTTLE_STEP)

    # Aileron, elevator, rudder spring back to neutral on release, like a
    # real transmitter stick
    aileron_pwm = PWM_NEUTRAL
    if "d" in pressed_keys:
        aileron_pwm = PWM_NEUTRAL + AILERON_DEFLECTION
    elif "a" in pressed_keys:
        aileron_pwm = PWM_NEUTRAL - AILERON_DEFLECTION

    elevator_pwm = PWM_NEUTRAL
    if "up" in pressed_keys:
        elevator_pwm = PWM_NEUTRAL + ELEVATOR_DEFLECTION
    elif "down" in pressed_keys:
        elevator_pwm = PWM_NEUTRAL - ELEVATOR_DEFLECTION

    rudder_pwm = PWM_NEUTRAL
    if "e" in pressed_keys:
        rudder_pwm = PWM_NEUTRAL + RUDDER_DEFLECTION
    elif "q" in pressed_keys:
        rudder_pwm = PWM_NEUTRAL - RUDDER_DEFLECTION

    return aileron_pwm, elevator_pwm, throttle_pwm, rudder_pwm


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
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

set_mode(FLIGHT_MODE)

# Establish a known-safe RC state (zero throttle, all sticks centred)
# BEFORE arming - arming with stale/undefined RC values is unsafe.
for _ in range(20):
    send_rc_override(PWM_NEUTRAL, PWM_NEUTRAL, PWM_MIN, PWM_NEUTRAL)
    time.sleep(0.05)

arm()
print("Armed and ready. Throttle up with 'w' to begin the takeoff roll.")

try:
    while status:
        aileron_pwm, elevator_pwm, throttle_pwm, rudder_pwm = compute_rc_values()
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
