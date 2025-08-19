import time
import usb
import usb_hid
import usb_host
import adafruit_usb_host_descriptors
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode
import board
import neopixel
import supervisor
import digitalio
import busio
import traceback
import struct


def find_keyboards():
    keyboards = []
    for device in usb.core.find(find_all=True):
        kbd_interface_index, kbd_endpoint_address = adafruit_usb_host_descriptors.find_boot_keyboard_endpoint(
            device)
        if isinstance(kbd_interface_index, int) and isinstance(kbd_endpoint_address, int):
            keyboards.append(
                (device, kbd_interface_index, kbd_endpoint_address))
    return keyboards


def configure_keyboard(device, kbd_interface_index):
    if device.is_kernel_driver_active(kbd_interface_index):
        if not device.detach_kernel_driver(kbd_interface_index):
            return False
    return device.set_configuration() is not None


def presses_and_releases(pasts: list, currents: list):
    releases = [x if x not in currents else 0 for x in pasts]
    presses = [x if x not in pasts else 0 for x in currents]
    return releases, presses


def emit_events_release(kbd: Keyboard, releases: list, keycodes: list):
    [kbd.release(keycodes[x-1]) for x in releases if x]


def emit_events_press(kbd: Keyboard, presses: list, keycodes: list):
    [kbd.press(keycodes[x-1]) for x in presses if x]


def send_events_release(uart, releases: list):
    [uart.write(struct.pack('b', -x)) for x in releases if x]


def send_events_press(uart, press: list):
    [uart.write(struct.pack('b', +x)) for x in press if x]


def recv_events(uart):
    data = uart.read(1)
    return struct.unpack('b', data)[0] if data is not None else None


def index_from_keycodes(keycode, map_positions):
    codes = list(keycode)[2:]
    positions = [map_positions.get(code, 0) for code in codes]
    return positions


def error(pixel):
    pixel.fill((255, 0, 0))
    time.sleep(1)
    supervisor.reload()


mapping = [
    Keycode.ESCAPE,		Keycode.Q,			Keycode.W,			Keycode.F, 			Keycode.P,			Keycode.G,
    Keycode.TAB,		Keycode.A,			Keycode.R,			Keycode.S, 			Keycode.T,			Keycode.D,
    Keycode.SHIFT,		Keycode.Y,			Keycode.X,			Keycode.C, 			Keycode.V,			Keycode.B,
    Keycode.SPACEBAR,	Keycode.SPACEBAR,	Keycode.SPACEBAR,	Keycode.SPACEBAR,	Keycode.SPACEBAR,	Keycode.SPACEBAR
]

map_positions = {
    33: 1, 34: 2, 5: 3, 6: 4, 13: 5, 14: 6,
    32: 7, 35: 8, 4: 9, 7: 10, 12: 11, 15: 12,
    31: 13, 36: 14, 39: 15, 8: 16, 11: 17, 16: 18,
    30: 19, 37: 20, 38: 21, 9: 22, 10: 23, 17: 24
}


def main():
    time.sleep(1)

    pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
    pixel.brightness = 1
    pixel.fill((0, 0, 255))

    keyboards = find_keyboards()
    if not keyboards:
        print("No boot keyboard endpoint found")

    keyboard, kbd_interface_index, kbd_endpoint_address = keyboards[0]
    if configure_keyboard(keyboard, kbd_interface_index):
        print("Keyboard configuration failed")

    print("Keyboard configuration passed")

    pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)
    pixel.brightness = 1
    pixel.fill((0, 255, 0))

    kbd = Keyboard(usb_hid.devices)
    uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0)

    keycode = bytearray(8)
    pasts = [0] * 6
    while True:
        try:
            if 8 == keyboard.read(kbd_endpoint_address, keycode, timeout=0):
                currents = index_from_keycodes(keycode, map_positions)
                releases, presses = presses_and_releases(pasts, currents)

                emit_events_release(kbd, releases, mapping)
                emit_events_press(kbd, presses, mapping)

                send_events_release(uart, releases)
                send_events_press(uart, presses)

                pasts = currents

            print(recv_events(uart))

        except Exception as e:
            traceback.print_exception(e)
            error(pixel)


main()
