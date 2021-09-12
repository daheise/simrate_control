from sc_config import SimrateControlConfig
from sc_curses import ScCurses, CursesCommands
from flight_parameters import (
    FlightDataMetrics,
    SimrateDiscriminator,
    SimConnectDataError,
)
from SimConnect import *
import sys, os
import configparser
from time import sleep
from math import degrees, floor, log2

import pyttsx3

# from SimConnect import *
from geopy import distance

# logging.basicConfig(level=logging.INFO)
# LOGGER = logging.getLogger(__name__)
# LOGGER.info("START")

# A simple structure to hold distance from the previous and next waypoints
# WaypointClearance = namedtuple("WaypointClearances", "prev next")


class SimRateManager:
    """Manages the game sim rate, and audible annunciation."""

    def __init__(self, sm, config):
        self.sm = sm
        self._config = config
        self.have_paused_at_tod = False

        self.aq = AircraftRequests(self.sm)
        self.ae = AircraftEvents(self.sm)

        self.increase_sim_rate = self.ae.find("SIM_RATE_INCR")
        self.decrease_sim_rate = self.ae.find("SIM_RATE_DECR")
        # The most innocuous "SELECT" event I can find at the moment to prevent
        # wild simrrate selections while adjusting something (e.g. altitude bug)
        # during a transition.
        # TODO: Figure out something better.
        self.heading_select_bug = self.ae.find("HEADING_BUG_SELECT")
        self.set_barometer = self.ae.find("BAROMETRIC")
        self.ae_pause = self.ae.find("PAUSE_ON")
        self.ae_pause_off = self.ae.find("PAUSE_OFF")
        self.tts_engine = pyttsx3.init()

    def _get_value(self, aq_name, retries=sys.maxsize):
        # PySimConnect seems to crash the sim if requests happen too fast.
        sleep(0.05)
        val = self.aq.find(str(aq_name)).value
        i = 0
        while val is None and i < retries:
            sleep(min(1, 0.01 * (i + 1)))
            val = self.aq.find(str(aq_name)).value
            i += 1
        return val

    def get_sim_rate(self):
        """Get the current sim rate."""
        return self._get_value("SIMULATION_RATE")

    def say_sim_rate(self):
        """Speak the current sim rate using text-to-speech"""
        if not self._config.annunciation:
            return

        try:
            simrate = self.get_sim_rate()
            if simrate >= 1.0:
                self.tts_engine.say(f"Sim rate {str(int(simrate))}")
            else:
                self.tts_engine.say(f"Sim rate {str(simrate)}")
            self.tts_engine.runAndWait()
        except TypeError:
            pass

    def pause(self):
        """Pause the sim"""
        self.ae_pause()
        if self._config.annunciation:
            self.tts_engine.say(f"Game paused at tod")
            self.tts_engine.runAndWait()

    def unpause(self):
        """Pause the sim"""
        self.ae_pause_off()
        if self._config.annunciation:
            self.tts_engine.say(f"Game unpaused")
            self.tts_engine.runAndWait()

    def stop_acceleration(self):
        """Decrease the sim rate to the minimum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        while simrate > self._config.min_rate:
            sleep(2)
            self.decelerate()
            simrate /= 2

    def decelerate(self):
        """Decrease the sim rate, up to some maximum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        if simrate > self._config.min_rate:
            self.decrease_sim_rate()
            if self._config.set_barometer:
                self.set_barometer()
            self.heading_select_bug()
        elif simrate < self._config.min_rate:
            self.increase_sim_rate()
            if self._config.set_barometer:
                self.set_barometer()
            self.heading_select_bug()

    def accelerate(self):
        """Increase the sim rate, up to some maximum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        if simrate < self._config.max_rate:
            self.increase_sim_rate()
            if self._config.set_barometer:
                self.set_barometer()
            self.heading_select_bug()
        elif simrate > self._config.max_rate:
            self.decrease_sim_rate()
            if self._config.set_barometer:
                self.set_barometer()
            self.heading_select_bug()

    def update(self, max_stable_rate):
        messages = []
        prev_simrate = self.get_sim_rate()
        if max_stable_rate is None:
            raise SimConnectDataError()
        elif max_stable_rate == 0 and not self.have_paused_at_tod:
            self.have_paused_at_tod = True
            self.pause()
        elif max_stable_rate > prev_simrate:
            messages.append("accelerate")
            self.accelerate()
        elif max_stable_rate < prev_simrate:
            messages.append("decelerate")
            self.decelerate()
        sleep(0.5)
        new_simrate = self.get_sim_rate()
        if prev_simrate != new_simrate:
            self.say_sim_rate()
        return messages


def write_screen(
    sc_curses: ScCurses,
    config: SimrateControlConfig,
    flight_data_parameters: FlightDataMetrics,
    simrate_discriminator: SimrateDiscriminator,
    simrate_manager: SimRateManager,
    messages,
):
    sc_curses.write_simrate(simrate_manager.get_sim_rate())
    sc_curses.write_target_simrate(simrate_discriminator.get_max_sim_rate())
    sc_curses.write_max_simrate(config.max_rate)
    sc_curses.write_bank(degrees(flight_data_parameters.aq_bank))
    sc_curses.write_pitch(degrees(flight_data_parameters.aq_pitch))
    sc_curses.write_ground_speed(flight_data_parameters.ground_speed() * 3600)
    sc_curses.write_waypoint_ident(flight_data_parameters.aq_next_wp_ident)
    sc_curses.write_ground_alt(flight_data_parameters.get_ground_elevation())
    sc_curses.write_waypoint_distance(
        flight_data_parameters.get_waypoint_distances().next
    )
    sc_curses.write_waypoint_direction(
        flight_data_parameters.get_waypoint_directions()[1]
    )
    sc_curses.write_waypoint_alt(flight_data_parameters.next_waypoint_altitude())
    sc_curses.write_target_vspeed(flight_data_parameters.target_fpm())
    sc_curses.write_target_slope(flight_data_parameters.choose_slope_angle())
    sc_curses.write_tod_distance(flight_data_parameters.distance_to_flc())
    sc_curses.write_tod_time(
        flight_data_parameters.time_to_flc(), simrate_manager.get_sim_rate()
    )
    sc_curses.write_vspeed(flight_data_parameters.aq_vsi)
    sc_curses.write_needed_vspeed(flight_data_parameters.required_fpm())
    sc_curses.write_max_bank(config.max_bank)
    sc_curses.write_max_pitch(config.max_pitch)
    sc_curses.write_ap_mode(simrate_discriminator.is_ap_active())
    sc_curses.write_min_agl(config.min_agl_cruise)
    sc_curses.write_agl(flight_data_parameters.aq_agl)
    sc_curses.write_alt(flight_data_parameters.aq_alt_indicated)
    sc_curses.write_ete(flight_data_parameters.aq_ete)
    sc_curses.write_ete_compressed(
        flight_data_parameters.aq_ete, simrate_manager.get_sim_rate()
    )
    sc_curses.write_messages(messages)


def connect(retries=999):
    connected = False
    sm = None
    i = 0
    while not connected and i <= retries:
        i += 1
        try:
            sm = SimConnect()
            connected = True
        except KeyboardInterrupt:
            quit()
        except Exception as e:
            # ui.write_message(type(e).__name__, e)
            sleep(1)
    return sm


def main(stdscr):
    from sc_curses import ScCurses

    stdscr.nodelay(True)
    ui = ScCurses(stdscr)
    config = SimrateControlConfig("config.ini")
    ui.write_message("Not connected...")
    sm = None
    srm = None
    flight_data_metrics = None
    flight_stability = None
    while True:
        user_input = ui.update()
        if user_input == CursesCommands.QUIT:
            break
        messages = []
        if sm is None:
            ui.write_message("Not connected...")
            flight_data_metrics = None
            srm = None
            flight_stability = None
            sm = connect(0)
        else:
            if flight_data_metrics is None:
                flight_data_metrics = FlightDataMetrics(sm, config)
                flight_stability = SimrateDiscriminator(flight_data_metrics, config)
                # This will be used to toggle pause on and off
                simrate_functions = [flight_stability.get_max_sim_rate, lambda: 1]
            if srm is None:
                srm = SimRateManager(sm, config)
            ui.write_message("Connected to simulator.")

        if sm is not None and srm is not None:
            try:
                if user_input == CursesCommands.TOGGLE_ACCEL:
                    simrate_functions.reverse()
                if user_input == CursesCommands.TOGGLE_WAYPOINTS:
                    config.waypoint_vnav = not config.waypoint_vnav
                if user_input == CursesCommands.UNPAUSE:
                    srm.unpause()
                if user_input == CursesCommands.PAUSE:
                    srm.pause()
                if user_input == CursesCommands.MAX_SIMRATE_1:
                    config.max_rate = 1
                if user_input == CursesCommands.MAX_SIMRATE_2:
                    config.max_rate = 2
                if user_input == CursesCommands.MAX_SIMRATE_4:
                    config.max_rate = 4
                if user_input == CursesCommands.MAX_SIMRATE_8:
                    config.max_rate = 8
                if user_input == CursesCommands.MAX_SIMRATE_16:
                    config.max_rate = 16

                if not config.waypoint_vnav:
                    ui.write_message("Waypoint vertical detection disabled")
                if simrate_functions[0] != flight_stability.get_max_sim_rate:
                    ui.write_message("Acceleration paused by user")
                flight_data_metrics.update()
                max_stable_rate = simrate_functions[0]()
                messages += flight_stability.get_messages()
                messages += srm.update(max_stable_rate)
                write_screen(
                    ui, config, flight_data_metrics, flight_stability, srm, messages
                )
                sleep(0.01)
            except (SimConnectDataError, AttributeError, TypeError) as e:
                messages.append("DATA ERROR")
                messages.append(str(e))
                if config.decelerate_on_simconnect_error:
                    srm.decelerate()
            except KeyboardInterrupt:
                srm.stop_acceleration()
                sleep(1)
                srm.say_sim_rate()
                ui.update()
                break
            except OSError as e:
                ui.write_message(str(e))
                sm = None
    if srm is not None and sm is not None:
        srm.unpause()
        srm.stop_acceleration()
        sleep(1)
        srm.say_sim_rate()
    return 0


def curse(stdscr):
    from sc_curses import ScCurses

    ui = ScCurses(stdscr)
    ui.update(None)
    stdscr.getkey()


if __name__ == "__main__":
    from curses import wrapper

    try:
        os.system("mode con: cols=65 lines=20")
        wrapper(main)
    except OSError:
        os.system("cls")
        print("Flight Simulator exited. Shutting down.")
        sys.exit()
