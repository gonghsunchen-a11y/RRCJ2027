from maix import camera, image, app, time, uart, err, pinmap
import math

pin_function = {
        "A19": "UART1_TX",
        "A18": "UART1_RX"
    }
device = "/dev/ttyS1"

for pin, func in pin_function.items():
    err.check_raise(pinmap.set_pin_function(pin, func), f"Failed set pin{pin} function to {func}")
# Init UART
ser = uart.UART(device, 115200)


DEBUG_VIEW = True
GOAL_UPDATE_EVERY = 10

cam = camera.Camera(320, 240, image.Format.FMT_RGB888)
cam.set_awb(1)

if DEBUG_VIEW:
    from maix import display
    disp = display.Display()

# 選擇要找的球門
YELLOW_LAB = [30, 77, -16, 28, 29, 72]
BLUE_LAB = [30, 53, -24, 23, -47, -10]

GOAL_LAB = YELLOW_LAB
# GOAL_LAB = BLUE_LAB

CENTER_X = 156
CENTER_Y = 121
R_MIN = 37
R_MAX = 110

# 距離公式參數
A = 0.003379401783
B = 1671.194765
C = -23.10186756

HIST_N = 5
radius_history = []

frame_count = 0
best_goal = None

goal_found = 0
goal_angle = 0xFFFF
goal_dist = 0xFFFF

last_print = time.ticks_ms()


def point_radius(x, y):
    return math.hypot(x - CENTER_X, y - CENTER_Y)


def angle_diff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def pack_uint16(value):
    value = int(value) & 0xFFFF
    return value & 0xFF, (value >> 8) & 0xFF


def merge_goal_parts(blobs):
    parts = []

    if not blobs:
        return None

    for b in blobs:
        r = point_radius(b.cx(), b.cy())

        if not (R_MIN <= r <= R_MAX):
            continue

        if b.pixels() < 8:
            continue

        parts.append({
            "blob": b,
            "r": r,
            "angle": math.atan2(
                b.cy() - CENTER_Y,
                b.cx() - CENTER_X
            )
        })

    if not parts:
        return None

    main = max(parts, key=lambda p: p["blob"].pixels())

    group = []

    for part in parts:
        r_difference = abs(part["r"] - main["r"])
        angle_difference = angle_diff(
            part["angle"],
            main["angle"]
        )

        if (r_difference < 18 and
                angle_difference < math.radians(45)):
            group.append(part["blob"])

    if not group:
        group = [main["blob"]]

    x1 = min(b.x() for b in group)
    y1 = min(b.y() for b in group)
    x2 = max(b.x() + b.w() for b in group)
    y2 = max(b.y() + b.h() for b in group)

    total_pixels = sum(b.pixels() for b in group)

    # 被遮擋分割時，使用最左和最右推算中心
    cx = (x1 + x2) // 2

    cy = (
        sum(b.cy() * b.pixels() for b in group)
        // total_pixels
    )

    return {
        "x": x1,
        "y": y1,
        "w": x2 - x1,
        "h": y2 - y1,
        "cx": int(cx),
        "cy": int(cy),
        "pixels": total_pixels,
        "count": len(group)
    }


def calculate_goal(goal):
    r = point_radius(goal["cx"], goal["cy"])

    radius_history.append(r)

    if len(radius_history) > HIST_N:
        radius_history.pop(0)

    average_radius = (
        sum(radius_history) / len(radius_history)
    )

    distance = (
        A * average_radius * average_radius +
        B / math.sqrt(goal["pixels"]) +
        C
    )

    distance = max(distance, 0.0)

    angle_radians = math.atan2(
        goal["cy"] - CENTER_Y,
        goal["cx"] - CENTER_X
    )

    angle_degrees = math.degrees(angle_radians) % 360

    return (
        int(round(angle_degrees)) % 360,
        int(round(distance))
    )


while not app.need_exit():
    img = cam.read()

    if img is None:
        continue

    img = img.flip(image.FlipDir.X)
    img = img.flip(image.FlipDir.Y)

    frame_count += 1

    if frame_count % GOAL_UPDATE_EVERY == 0:
        blobs = img.find_blobs(
            [GOAL_LAB],
            roi=(
                CENTER_X - R_MAX,
                CENTER_Y - R_MAX,
                R_MAX * 2,
                R_MAX * 2
            ),
            merge=False,
            area_threshold=8,
            pixels_threshold=8
        )

        best_goal = merge_goal_parts(blobs)

        if best_goal:
            goal_found = 1
            goal_angle, goal_dist = calculate_goal(best_goal)

            goal_dist = min(goal_dist, 65534)

        else:
            goal_found = 0
            goal_angle = 0xFFFF
            goal_dist = 0xFFFF
            radius_history.clear()

    angle_l, angle_h = pack_uint16(goal_angle)
    dist_l, dist_h = pack_uint16(goal_dist)

    checksum = (
        0xDD +
        goal_found +
        angle_l +
        angle_h +
        dist_l +
        dist_h
    ) & 0xFF

    # CC DD found angleL angleH distL distH checksum EE
    packet = bytes([
        0xCC,
        0xDD,
        goal_found,
        angle_l,
        angle_h,
        dist_l,
        dist_h,
        checksum,
        0xEE
    ])
    ser.write(packet)

    '''
    if ser.available():
        requests = ser.read()

        print("requests")
        if requests:
            for request in requests:
                if request == 0xDD:
                ser.write(packet)
                    break
    '''
    if DEBUG_VIEW:
        img.draw_circle(
            CENTER_X, CENTER_Y, R_MIN,
            image.Color.from_rgb(0, 255, 0), 1
        )

        img.draw_circle(
            CENTER_X, CENTER_Y, R_MAX,
            image.Color.from_rgb(255, 0, 0), 1
        )

        img.draw_cross(
            CENTER_X, CENTER_Y,
            image.Color.from_rgb(255, 255, 255), 8, 1
        )

        if best_goal:
            img.draw_rect(
                best_goal["x"],
                best_goal["y"],
                best_goal["w"],
                best_goal["h"],
                image.Color.from_rgb(255, 255, 0), 2
            )

            img.draw_cross(
                best_goal["cx"],
                best_goal["cy"],
                image.Color.from_rgb(0, 255, 0), 8, 2
            )

            img.draw_line(
                CENTER_X,
                CENTER_Y,
                best_goal["cx"],
                best_goal["cy"],
                image.Color.from_rgb(255, 255, 255), 2
            )

        now = time.ticks_ms()

        if now - last_print >= 200:
            last_print = now
            print("found=", goal_found,"angle=", goal_angle,"dist=", goal_dist)

        disp.show(img)