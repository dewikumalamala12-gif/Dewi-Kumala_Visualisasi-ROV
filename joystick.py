"""Cek nomor axis & tombol joystick.
Jalankan: python cek_joystick.py
Gerakkan stick / tekan tombol, lihat angka mana yang berubah. Ctrl+C untuk keluar.
"""
import time
import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    raise SystemExit("Joystick tidak terdeteksi. Cek kabel USB.")

js = pygame.joystick.Joystick(0)
js.init()
print("Joystick:", js.get_name())
print("Jumlah axis:", js.get_numaxes(), "| tombol:", js.get_numbuttons())

try:
    while True:
        pygame.event.pump()
        axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
        btns = [i for i in range(js.get_numbuttons()) if js.get_button(i)]
        print(f"axis={axes}  tombol_ditekan={btns}      ", end="\r")
        time.sleep(0.1)
except KeyboardInterrupt:
    pygame.quit()