# Clear the screen and hold it for 3 seconds
import curses
import time
import textwrap
from datetime import timedelta


class ScCurses:
    def __init__(self, screen) -> None:
        self._screen = screen
        self._screen.nodelay(True)
        self.write_layout()
        self._messages = []

    def write_layout(self):
        # Clear screen
        layout = """Get-there-itis Simrate Control
Sim Rate:                  Target Simrate:  
AP Mode: 
Pitch/Bank:     /          Max Pitch/Bank:     / 
Alt:                       Ground Alt: 
Waypoint:                  G. Speed: 
Waypoint Dist:             FLC:          / 
Waypoint Alt:              Target VS/Slope:         / 
VS:                        Needed VS: 
AGL:                       Min AGL: 
ETE:          /  
Messages:
        """
        layout = textwrap.dedent(layout)
        self._screen.addstr(0, 0, layout)

    def write_ete(self, seconds):
        if seconds > 0:
            seconds = min(24 * 3600 - 1, seconds)
        else:
            seconds = 0
        td = timedelta(seconds=int(seconds))
        self._screen.addstr(10, 5, f"{str(td)}")

    def write_ete_compressed(self, seconds, compression):
        seconds = seconds // compression
        if seconds > 0:
            seconds = min(24 * 3600 - 1, seconds)
        else:
            seconds = 0
        td = timedelta(seconds=int(seconds))
        self._screen.addstr(10, 16, f"{int(compression)}x = ")
        self._screen.addstr(10, 22, f"{str(td)}")

    def write_simrate(self, rate: int) -> None:
        self._screen.addstr(1, 10, f"{rate:.2f}x")

    def write_target_simrate(self, rate: int) -> None:
        self._screen.addstr(1, 43, f"{str(int(rate))}x")

    def write_simconnect_status(self, status) -> None:
        self._screen.addstr(11, 46, f"{str(status)}")

    def write_ap_mode(self, mode: str) -> None:
        self._screen.addstr(2, 9, str(mode).ljust(3))

    def write_pitch(self, angle) -> None:
        self._screen.addstr(3, 13, f"{str(int(angle))}°")

    def write_max_pitch(self, angle) -> None:
        self._screen.addstr(3, 43, f"{str(int(angle))}°")

    def write_bank(self, angle) -> None:
        self._screen.addstr(3, 18, f"{str(int(angle))}°")

    def write_max_bank(self, angle) -> None:
        self._screen.addstr(3, 49, f"{str(int(angle))}°")

    def write_alt(self, feet):
        self._screen.addstr(4, 5, f"{str(int(feet))}ft")

    def write_ground_alt(self, feet):
        self._screen.addstr(4, 39, f"{str(int(feet))}ft")

    def write_waypoint_ident(self, ident):
        self._screen.addstr(5, 10, f"{str(ident)}")

    def write_ground_speed(self, speed):
        self._screen.addstr(5, 37, f"{str(int(speed))}kts")

    def write_waypoint_distance(self, dist: float):
        self._screen.addstr(6, 15, f"{dist:.2f}nm")

    def write_tod_time(self, seconds):
        if seconds > 0:
            seconds = min(24 * 3600 - 1, seconds)
        else:
            seconds = 0
        td = timedelta(seconds=int(seconds))
        self._screen.addstr(6, 32, f"{str(td)}")

    def write_tod_distance(self, nm):
        self._screen.addstr(6, 43, f"{nm:.1f}nm")

    def write_waypoint_alt(self, feet: float):
        self._screen.addstr(7, 14, f"{str(int(feet))}ft")

    def write_target_vspeed(self, fpm: float):
        self._screen.addstr(7, 44, f"{str(int(fpm))}fpm")

    def write_target_slope(self, angle):
        self._screen.addstr(7, 54, f"{int(angle)}°")

    def write_vspeed(self, fpm: float):
        self._screen.addstr(8, 4, f"{int(fpm)}fpm")

    def write_needed_vspeed(self, fpm: float):
        self._screen.addstr(8, 38, f"{int(fpm)}fpm")

    def write_agl(self, feet: float):
        self._screen.addstr(9, 5, f"{int(feet)}ft")

    def write_min_agl(self, feet: float):
        self._screen.addstr(9, 36, f"{int(feet)}ft")

    def clear_messages(self):
        eraser = " " * curses.COLS
        start = 12
        for i in range(start, curses.LINES - 1):
            self._screen.addstr(i, 0, f"{eraser}")
            i += 1

    def _write_messages_to_screen(self):
        i = 12
        max_messages = curses.LINES - 1 - i
        for m in self._messages[0:max_messages]:
            self._screen.addstr(i, 0, f"{m}")
            i += 1
        if len(self._messages) > max_messages:
            self._screen.addstr(i, 0, "Additional messages truncated.")

    def write_messages(self, messages: list):
        self._messages += messages

    def write_message(self, msg):
        self._messages.append(str(msg))

    def update(self):
        self._write_messages_to_screen()
        k = self._screen.getch()
        self._messages = []
        self._screen.clear()
        self.write_layout()
        if k == ord("q") or k == 3:
            return False
        else:
            return True
