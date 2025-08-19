from neopixel import NeoPixel
import time
from usb.core import find, Device
import usb_hid
import usb_host
import adafruit_usb_host_descriptors
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode
from board_definitions import adafruit_feather_rp2040_usb_host as board
from neopixel import NeoPixel
import supervisor
import digitalio
from busio import UART
import traceback
import struct
from array import array


def find_keyboard() -> tuple[Device, int, int] | tuple[None, None, None]:

    keyboard: Device | None = find() or None

    if keyboard:
        kbd_interface_index, kbd_endpoint_address = adafruit_usb_host_descriptors.find_boot_keyboard_endpoint(
            keyboard)
        if isinstance(kbd_interface_index, int) and isinstance(kbd_endpoint_address, int):
            return keyboard, kbd_interface_index, kbd_endpoint_address
        else:
            return None, None, None
    else:
        return None, None, None


def configure_keyboard(device, kbd_interface_index) -> bool:
    if device.is_kernel_driver_active(kbd_interface_index):
        if not device.detach_kernel_driver(kbd_interface_index):
            return False
    return device.set_configuration() is not None


def presses_and_releases(pasts: list, currents: list):
    releases = [x if x not in currents else 0 for x in pasts]
    presses = [x if x not in pasts else 0 for x in currents]
    return releases, presses


def emit_events_release(kbd: Keyboard, releases: list, keycodes: list) -> None:
    [kbd.release(keycodes[x-1]) for x in releases if x]


def emit_events_press(kbd: Keyboard, presses: list, keycodes: list) -> None:
    [kbd.press(keycodes[x-1]) for x in presses if x]


def send_events_release(uart, releases: list) -> None:
    [uart.write(struct.pack('b', -x)) for x in releases if x]


def send_events_press(uart: UART, press: list) -> None:
    [uart.write(struct.pack('b', +x)) for x in press if x]


def recv_events(uart: UART) -> int | None:
    data: bytes | None = uart.read(1)
    return struct.unpack('b', data)[0] if data is not None else None


def index_from_keycodes(keycode, map_positions) -> list[int]:
    codes: list[int] = list(keycode)[2:]
    positions: list[int] = [map_positions.get(code, 0) for code in codes]
    return positions


def error(pixel) -> None:
    pixel.fill((255, 0, 0))
    time.sleep(1)
    supervisor.reload()


mapping: list[int] = [
    Keycode.ESCAPE,		Keycode.Q,			Keycode.W,			Keycode.F, 			Keycode.P,			Keycode.G,
    Keycode.TAB,		Keycode.A,			Keycode.R,			Keycode.S, 			Keycode.T,			Keycode.D,
    Keycode.SHIFT,		Keycode.Y,			Keycode.X,			Keycode.C, 			Keycode.V,			Keycode.B,
    Keycode.SPACEBAR,	Keycode.SPACEBAR,	Keycode.SPACEBAR,	Keycode.SPACEBAR,	Keycode.SPACEBAR,	Keycode.SPACEBAR
]

map_positions: dict[int, int] = {
    33:  1, 34:  2,  5:  3, 6:  4, 13:  5, 14:  6,
    32:  7, 35:  8,  4:  9, 7: 10, 12: 11, 15: 12,
    31: 13, 36: 14, 39: 15, 8: 16, 11: 17, 16: 18,
    30: 19, 37: 20, 38: 21, 9: 22, 10: 23, 17: 24
}


def main() -> None:
    time.sleep(1)

    keyboard, kbd_interface_index, kbd_endpoint_address = find_keyboard()

    if (keyboard is None) or (kbd_interface_index is None) or (kbd_endpoint_address is None):
        print("No boot keyboard endpoint found")
    else:
        if configure_keyboard(keyboard, kbd_interface_index):
            print("Keyboard configuration failed")

        print("Keyboard configuration passed")

        pixel: NeoPixel = NeoPixel(board.NEOPIXEL, 1)
        pixel.brightness = 1
        pixel.fill((0, 255, 0))

        kbd: Keyboard = Keyboard(usb_hid.devices)
        uart: UART = UART(board.TX, board.RX, baudrate=115200, timeout=0)

        keycode: array[int] = array("b", [0] * 8)

        pasts: list[int] = [0] * 6
        while True:
            try:
                if 8 == keyboard.read(kbd_endpoint_address, keycode, timeout=0):
                    currents: list[int] = index_from_keycodes(
                        keycode, map_positions)
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
