import pygame
import math
import time
import traceback
from pymavlink import mavutil

# ==========================================
# 1. KONFIGURASI
# ==========================================

PIXHAWK_PORT = "COM4"   # sesuaikan dengan port di Device Manager
BAUD_RATE = 115200
STREAM_RATE_HZ = 10

# Batas skala gauge (ubah sesuai kebutuhan)
SPEED_MAX = 20.0        # m/s
BATTERY_MAX_V = 20.0    # 4S penuh = 16.8 V -> 20 cukup, 6S penuh = 25.2 V -> isi 25

print(f"Menghubungkan ke Pixhawk melalui {PIXHAWK_PORT}...")

connection = mavutil.mavlink_connection(
    PIXHAWK_PORT,
    baud=BAUD_RATE,
    autoreconnect=True
)

print("Port serial berhasil dibuka.")
print("Menunggu HEARTBEAT dari Pixhawk (timeout 10 detik)...")

hb = connection.wait_heartbeat(timeout=10)

if hb is None:
    print("HEARTBEAT belum diterima. Program tetap jalan, dashboard akan menunggu data.")
else:
    print(
        f"Heartbeat diterima dari system {connection.target_system}, "
        f"component {connection.target_component}"
    )

    # Minta Pixhawk mengirim semua data stream
    connection.mav.request_data_stream_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        STREAM_RATE_HZ,
        1  # 1 = aktifkan
    )
    print(f"Data stream diminta ({STREAM_RATE_HZ} Hz).")


# ==========================================
# 2. DATA TELEMETRI
# ==========================================

telemetry = {
    "connected": False,
    "armed": False,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": 0.0,
    "latitude": 0.0,
    "longitude": 0.0,
    "altitude": 0.0,
    "speed": 0.0,
    "heading": 0,
    "throttle": 0,
    "battery": 0.0,
    "mode": "UNKNOWN",
    "last_heartbeat": 0
}


# ==========================================
# 3. MEMBACA DAN TRANSFORMASI MAVLINK
# ==========================================

def read_mavlink():

    while True:

        msg = connection.recv_match(blocking=False)

        if msg is None:
            break

        msg_type = msg.get_type()

        if msg_type == "BAD_DATA":
            continue

        # HEARTBEAT
        if msg_type == "HEARTBEAT":

            # Abaikan heartbeat dari GCS (bukan dari flight controller)
            if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
                continue

            telemetry["connected"] = True
            telemetry["last_heartbeat"] = time.time()

            telemetry["armed"] = bool(
                msg.base_mode &
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

            telemetry["mode"] = connection.flightmode

        # ATTITUDE
        elif msg_type == "ATTITUDE":

            # Radian menjadi derajat
            telemetry["roll"] = math.degrees(msg.roll)
            telemetry["pitch"] = math.degrees(msg.pitch)
            telemetry["yaw"] = math.degrees(msg.yaw)

        # GPS DAN POSISI
        elif msg_type == "GLOBAL_POSITION_INT":

            # Integer menjadi koordinat desimal
            telemetry["latitude"] = msg.lat / 1e7
            telemetry["longitude"] = msg.lon / 1e7

            # Millimeter menjadi meter
            telemetry["altitude"] = msg.relative_alt / 1000.0

        # INFORMASI PENERBANGAN
        elif msg_type == "VFR_HUD":

            telemetry["speed"] = msg.groundspeed
            telemetry["heading"] = msg.heading
            telemetry["throttle"] = msg.throttle

        # BATERAI
        elif msg_type == "SYS_STATUS":

            if msg.voltage_battery != 65535:
                telemetry["battery"] = msg.voltage_battery / 1000.0


# ==========================================
# 4. INISIALISASI PYGAME
# ==========================================

pygame.init()

WIDTH = 1300
HEIGHT = 700

screen = pygame.display.set_mode((WIDTH, HEIGHT))

pygame.display.set_caption("PIXHAWK GROUND CONTROL STATION")

clock = pygame.time.Clock()

BG = (20, 25, 35)
PANEL = (28, 34, 48)
TRACK = (55, 65, 85)
WHITE = (240, 240, 240)
GREEN = (0, 220, 120)
RED = (230, 70, 70)
BLUE = (60, 150, 255)
GRAY = (150, 160, 175)
ORANGE = (255, 170, 40)
SKY = (60, 130, 200)
GROUND = (130, 90, 50)


# ==========================================
# 5. FUNGSI MENAMPILKAN TEKS (DENGAN CACHE FONT)
# ==========================================

font_cache = {}


def get_font(size, bold=False):
    key = (size, bold)
    if key not in font_cache:
        font_cache[key] = pygame.font.SysFont("consolas", size, bold=bold)
    return font_cache[key]


def draw_text(text, x, y, color=WHITE, size=20, bold=False):

    surface = get_font(size, bold).render(str(text), True, color)

    screen.blit(surface, (x, y))


def draw_text_center(text, cx, cy, color=WHITE, size=20, bold=False):

    surface = get_font(size, bold).render(str(text), True, color)

    rect = surface.get_rect(center=(cx, cy))

    screen.blit(surface, rect)


# ==========================================
# 6. FUNGSI BANTU GEOMETRI
# ==========================================

def polar(cx, cy, radius, angle_deg):
    """
    Ubah sudut (0 = atas, searah jarum jam) menjadi koordinat layar.
    """
    rad = math.radians(angle_deg)
    return (
        cx + radius * math.sin(rad),
        cy - radius * math.cos(rad)
    )


def arc_points(cx, cy, radius, start_deg, end_deg):
    steps = max(2, int(abs(end_deg - start_deg) / 3) + 1)
    return [
        polar(cx, cy, radius,
              start_deg + (end_deg - start_deg) * i / (steps - 1))
        for i in range(steps)
    ]


# ==========================================
# 7. GAUGE BUSUR (SPEED, THROTTLE, BATTERY)
# ==========================================

def draw_arc_gauge(cx, cy, r, value, vmin, vmax, unit,
                   color, decimals=2, major=4):

    frac = (value - vmin) / (vmax - vmin)
    frac = max(0.0, min(1.0, frac))

    # Latar
    pygame.draw.circle(screen, PANEL, (cx, cy), r + 22)

    # Busur latar (270 derajat, dari -135 sampai +135)
    pygame.draw.lines(
        screen, TRACK, False,
        arc_points(cx, cy, r, -135, 135), 12
    )

    # Busur nilai
    if frac > 0.005:
        pygame.draw.lines(
            screen, color, False,
            arc_points(cx, cy, r, -135, -135 + 270 * frac), 12
        )

    # Garis skala dan angka
    for i in range(major + 1):

        angle = -135 + 270 * i / major
        tick_value = vmin + (vmax - vmin) * i / major

        pygame.draw.line(
            screen, WHITE,
            polar(cx, cy, r + 9, angle),
            polar(cx, cy, r + 17, angle), 2
        )

        x, y = polar(cx, cy, r - 24, angle)
        draw_text_center(f"{tick_value:g}", x, y, GRAY, 14)

    # Angka utama di tengah
    draw_text_center(f"{value:.{decimals}f}", cx, cy - 2, color, 30, True)
    draw_text_center(unit, cx, cy + 26, GRAY, 18)


# ==========================================
# 8. ATTITUDE INDICATOR (ROLL & PITCH)
# ==========================================

mask_cache = {}


def get_circle_mask(r):
    if r not in mask_cache:
        mask = pygame.Surface((2 * r, 2 * r), pygame.SRCALPHA)
        mask.fill((0, 0, 0, 0))
        pygame.draw.circle(mask, (255, 255, 255, 255), (r, r), r)
        mask_cache[r] = mask
    return mask_cache[r]


def draw_attitude(cx, cy, r, roll, pitch):

    size = r * 4
    mid = size // 2
    pixels_per_degree = 3.0

    # Batasi pitch tampilan agar tidak keluar dari surface
    p = max(-60.0, min(60.0, pitch))

    # Pitch positif (hidung naik) -> horizon turun
    horizon_y = int(mid + p * pixels_per_degree)

    big = pygame.Surface((size, size), pygame.SRCALPHA)

    pygame.draw.rect(big, SKY, (0, 0, size, horizon_y))
    pygame.draw.rect(big, GROUND, (0, horizon_y, size, size - horizon_y))
    pygame.draw.line(big, WHITE, (0, horizon_y), (size, horizon_y), 2)

    # Garis skala pitch
    for angle in (-30, -20, -10, 10, 20, 30):

        y = int(horizon_y - angle * pixels_per_degree)
        half = 30 if angle % 20 == 0 else 18

        pygame.draw.line(
            big, WHITE,
            (mid - half, y), (mid + half, y), 2
        )

        label = get_font(14).render(str(abs(angle)), True, WHITE)
        big.blit(label, (mid + half + 6, y - 7))

    # Putar sesuai roll (pygame memutar berlawanan jarum jam untuk nilai positif)
    rotated = pygame.transform.rotate(big, roll)

    view = pygame.Surface((2 * r, 2 * r), pygame.SRCALPHA)
    view.blit(
        rotated,
        (r - rotated.get_width() // 2, r - rotated.get_height() // 2)
    )

    # Potong berbentuk lingkaran
    view.blit(
        get_circle_mask(r), (0, 0),
        special_flags=pygame.BLEND_RGBA_MULT
    )

    screen.blit(view, (cx - r, cy - r))

    pygame.draw.circle(screen, BLUE, (cx, cy), r, 3)

    # Skala roll di tepi
    for a in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):

        length = 14 if a % 30 == 0 else 8

        pygame.draw.line(
            screen, WHITE,
            polar(cx, cy, r - 2, a),
            polar(cx, cy, r - 2 - length, a), 2
        )

    # Penunjuk roll (bergerak mengikuti horizon)
    pointer_angle = -max(-70.0, min(70.0, roll))

    pygame.draw.polygon(
        screen, ORANGE,
        [
            polar(cx, cy, r - 4, pointer_angle),
            polar(cx, cy, r - 20, pointer_angle - 4),
            polar(cx, cy, r - 20, pointer_angle + 4)
        ]
    )

    # Simbol pesawat tetap di tengah
    pygame.draw.line(screen, ORANGE, (cx - 55, cy), (cx - 18, cy), 4)
    pygame.draw.line(screen, ORANGE, (cx + 18, cy), (cx + 55, cy), 4)
    pygame.draw.line(screen, ORANGE, (cx - 18, cy), (cx - 18, cy + 8), 4)
    pygame.draw.line(screen, ORANGE, (cx + 18, cy), (cx + 18, cy + 8), 4)
    pygame.draw.circle(screen, ORANGE, (cx, cy), 4)


# ==========================================
# 9. KOMPAS
# ==========================================

CARDINALS_16 = [
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
]


def heading_to_cardinal(heading):
    index = int((heading % 360) / 22.5 + 0.5) % 16
    return CARDINALS_16[index]


def draw_compass(cx, cy, radius, heading):
    """
    Kompas berputar: mata angin berputar mengikuti heading,
    penunjuk oranye di atas dan ikon pesawat di tengah tetap diam.
    """

    pygame.draw.circle(screen, PANEL, (cx, cy), radius)
    pygame.draw.circle(screen, BLUE, (cx, cy), radius, 3)
    pygame.draw.circle(screen, GRAY, (cx, cy), radius - 70, 1)

    # Garis skala setiap 5 derajat
    for deg in range(0, 360, 5):

        screen_angle = deg - heading

        if deg % 90 == 0:
            length, color, width = 24, WHITE, 3
        elif deg % 30 == 0:
            length, color, width = 18, WHITE, 2
        elif deg % 10 == 0:
            length, color, width = 11, GRAY, 1
        else:
            length, color, width = 6, GRAY, 1

        pygame.draw.line(
            screen, color,
            polar(cx, cy, radius - 3, screen_angle),
            polar(cx, cy, radius - 3 - length, screen_angle),
            width
        )

    # Huruf mata angin dan angka tiap 30 derajat
    cardinal_labels = {0: "N", 90: "E", 180: "S", 270: "W"}

    for deg in range(0, 360, 30):

        screen_angle = deg - heading

        if deg in cardinal_labels:
            x, y = polar(cx, cy, radius - 46, screen_angle)
            label = cardinal_labels[deg]
            color = RED if label == "N" else WHITE
            draw_text_center(label, x, y, color, 28, True)
        else:
            x, y = polar(cx, cy, radius - 40, screen_angle)
            draw_text_center(str(deg // 10), x, y, GRAY, 16)

    # Penunjuk heading tetap di atas
    pygame.draw.polygon(
        screen, ORANGE,
        [
            (cx, cy - radius + 6),
            (cx - 11, cy - radius - 16),
            (cx + 11, cy - radius - 16)
        ]
    )

    # Ikon pesawat di tengah
    plane = [
        (cx, cy - 34),
        (cx + 22, cy + 26),
        (cx, cy + 12),
        (cx - 22, cy + 26)
    ]

    pygame.draw.polygon(screen, WHITE, plane)
    pygame.draw.polygon(screen, BLUE, plane, 2)

    # Pembacaan digital
    draw_text_center(
        f"{int(heading) % 360:03d}\u00b0  {heading_to_cardinal(heading)}",
        cx, cy + radius + 40, ORANGE, 30, True
    )


# ==========================================
# 10. DASHBOARD
# ==========================================

def draw_dashboard():

    screen.fill(BG)

    # HEADER
    draw_text("PIXHAWK GROUND CONTROL STATION", 30, 25, BLUE, 28)

    pygame.draw.line(screen, BLUE, (30, 75), (1270, 75), 2)

    # STATUS KONEKSI
    connected = (
        telemetry["last_heartbeat"] > 0
        and time.time() - telemetry["last_heartbeat"] < 3
    )

    telemetry["connected"] = connected

    draw_text(
        f"CONNECTION : {'CONNECTED' if connected else 'WAITING'}",
        40, 100, GREEN if connected else RED
    )

    armed = telemetry["armed"]

    draw_text(
        f"FLIGHT STATUS : {'ARMED' if armed else 'DISARMED'}",
        40, 140, GREEN if armed else RED
    )

    draw_text(f"FLIGHT MODE   : {telemetry['mode']}", 40, 180, WHITE)

    draw_text(
        "MAVLINK: ACTIVE" if connected else "MAVLINK: NO DATA",
        40, 220, GREEN if connected else RED
    )

    # DATA GPS (angka)
    draw_text("GPS DATA", 480, 100, BLUE, 24)

    draw_text(f"LATITUDE  : {telemetry['latitude']:.7f}", 480, 140)
    draw_text(f"LONGITUDE : {telemetry['longitude']:.7f}", 480, 170)
    draw_text(f"ALTITUDE  : {telemetry['altitude']:.2f} m", 480, 200)

    # JUDUL INSTRUMEN
    draw_text_center("ATTITUDE", 190, 270, BLUE, 24)
    draw_text_center("SPEED", 440, 270, BLUE, 24)
    draw_text_center("THROTTLE", 650, 270, BLUE, 24)
    draw_text_center("BATTERY", 860, 270, BLUE, 24)
    draw_text_center("COMPASS", 1110, 270, BLUE, 24)

    # ATTITUDE INDICATOR + ANGKA
    draw_attitude(190, 430, 115, telemetry["roll"], telemetry["pitch"])

    draw_text(f"ROLL  : {telemetry['roll']:.2f} deg", 60, 570)
    draw_text(f"PITCH : {telemetry['pitch']:.2f} deg", 60, 600)
    draw_text(f"YAW   : {telemetry['yaw']:.2f} deg", 60, 630)

    # GAUGE SPEED, THROTTLE, BATTERY (angka di tengah gauge)
    draw_arc_gauge(
        440, 430, 80, telemetry["speed"],
        0, SPEED_MAX, "m/s", BLUE, decimals=2, major=4
    )

    draw_arc_gauge(
        650, 430, 80, telemetry["throttle"],
        0, 100, "%", ORANGE, decimals=0, major=5
    )

    battery_color = GREEN if telemetry["battery"] > 0 else GRAY

    draw_arc_gauge(
        860, 430, 80, telemetry["battery"],
        0, BATTERY_MAX_V, "V", battery_color, decimals=2, major=4
    )

    # Angka tambahan di bawah gauge
    draw_text_center(f"SPEED    {telemetry['speed']:.2f} m/s", 545, 570, WHITE, 18)
    draw_text_center(f"THROTTLE {telemetry['throttle']} %", 545, 600, WHITE, 18)
    draw_text_center(f"BATTERY  {telemetry['battery']:.2f} V", 545, 630, WHITE, 18)

    # KOMPAS
    draw_compass(1110, 440, 140, telemetry["heading"])

    pygame.display.flip()


# ==========================================
# 11. MAIN LOOP
# ==========================================

running = True

try:

    while running:

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                running = False

        # Membaca data Pixhawk
        read_mavlink()

        # Menampilkan dashboard
        draw_dashboard()

        # FPS 30
        clock.tick(30)

except KeyboardInterrupt:
    print("Program dihentikan.")

except Exception:
    print("Terjadi kesalahan:")
    traceback.print_exc()

finally:

    connection.close()
    pygame.quit()

    print("Koneksi Pixhawk ditutup.")