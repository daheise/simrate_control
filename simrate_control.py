from sc_curses import ScCurses
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

    def __init__(self, sm, config=None):
        self.sm = sm
        self.config = config
        self.have_paused_at_tod = False
        if config is None or config.sections() == []:
            self.min_rate = 1
            self.max_rate = 4
            self.annunciation = True
        else:
            self.config = config
            self.min_rate = int(self.config["simrate"]["min_rate"])
            # If the user set something like "5" the simrate would go back and forth
            # because the game only has rates of the form 2**x. This formula will always
            # set the simrate to the valid simrate <= to the configured
            # simrate. 2**(floor(log2(x)))
            self.max_rate = int(
                2 ** floor(log2(int(self.config["simrate"]["max_rate"])))
            )
            self.annunciation = config.getboolean("simrate", "annunciation")

        self.aq = AircraftRequests(self.sm)
        self.ae = AircraftEvents(self.sm)

        self.sc_sim_rate = self.aq.find("SIMULATION_RATE")
        self.increase_sim_rate = self.ae.find("SIM_RATE_INCR")
        self.decrease_sim_rate = self.ae.find("SIM_RATE_DECR")
        self.ae_pause = self.ae.find("PAUSE_ON")
        self.tts_engine = pyttsx3.init()

    def get_sim_rate(self):
        """Get the current sim rate."""
        return self.sc_sim_rate.value

    def say_sim_rate(self):
        """Speak the current sim rate using text-to-speech"""
        if not self.annunciation:
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
        if self.annunciation:
            self.tts_engine.say(f"Paused at todd")
            self.tts_engine.runAndWait()

    def stop_acceleration(self):
        """Decrease the sim rate to the minimum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        while simrate > self.min_rate:
            sleep(2)
            self.decelerate()
            simrate /= 2

    def decelerate(self):
        """Decrease the sim rate, up to some maximum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        if simrate > self.min_rate:
            self.decrease_sim_rate()
        elif simrate < self.min_rate:
            self.increase_sim_rate()

    def accelerate(self):
        """Increase the sim rate, up to some maximum"""
        simrate = self.get_sim_rate()
        if simrate is None:
            return
        if simrate < self.max_rate:
            self.increase_sim_rate()
        elif simrate > self.max_rate:
            self.decrease_sim_rate()

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
        sleep(0.1)
        new_simrate = self.get_sim_rate()
        if prev_simrate != new_simrate:
            self.say_sim_rate()
        return messages


def update_screen(
    sc_curses: ScCurses,
    flight_data_parameters: FlightDataMetrics,
    simrate_discriminator: SimrateDiscriminator,
    simrate_manager: SimRateManager,
    messages,
):
    sc_curses.write_simrate(simrate_manager.get_sim_rate())
    sc_curses.write_target_simrate(simrate_discriminator.get_max_sim_rate())
    sc_curses.write_bank(degrees(flight_data_parameters.aq_bank))
    sc_curses.write_pitch(degrees(flight_data_parameters.aq_pitch))
    sc_curses.write_ground_speed(flight_data_parameters.ground_speed() * 3600)
    sc_curses.write_waypoint_distance(
        flight_data_parameters.get_waypoint_distances().next
    )
    sc_curses.write_waypoint_alt(flight_data_parameters.next_waypoint_altitude())
    if flight_data_parameters.target_altitude_change() > 0:
        sc_curses.write_target_vspeed(
            flight_data_parameters.target_fpm(simrate_discriminator.angle_of_climb)
        )
        sc_curses.write_target_slope(simrate_discriminator.angle_of_climb)
        sc_curses.write_tod_distance(
            flight_data_parameters.distance_to_flc(simrate_discriminator.angle_of_climb)
        )
        sc_curses.write_tod_time(
            flight_data_parameters.time_to_flc(simrate_discriminator.angle_of_climb)
        )
    else:
        sc_curses.write_target_vspeed(
            flight_data_parameters.target_fpm(simrate_discriminator.degrees_of_descent)
        )
        sc_curses.write_target_slope(simrate_discriminator.degrees_of_descent)
        sc_curses.write_tod_distance(
            flight_data_parameters.distance_to_flc(
                simrate_discriminator.degrees_of_descent
            )
        )
        sc_curses.write_tod_time(
            flight_data_parameters.time_to_flc(simrate_discriminator.degrees_of_descent)
        )
    sc_curses.write_vspeed(flight_data_parameters.aq_vsi)
    sc_curses.write_needed_vspeed(flight_data_parameters.required_fpm())
    sc_curses.write_max_bank(simrate_discriminator.max_bank)
    sc_curses.write_max_pitch(simrate_discriminator.max_pitch)
    sc_curses.write_ap_mode(simrate_discriminator.is_ap_active())
    sc_curses.write_min_agl(simrate_discriminator.min_agl_cruise)
    sc_curses.write_agl(flight_data_parameters.aq_agl)
    sc_curses.write_alt(flight_data_parameters.aq_alt_indicated)
    sc_curses.write_ete(flight_data_parameters.aq_ete)
    sc_curses.write_ete_compressed(
        flight_data_parameters.aq_ete, simrate_manager.get_sim_rate()
    )
    sc_curses.write_messages(messages)
    sc_curses.update()


def connect():
    connected = False
    sm = None
    while not connected:
        try:
            sm = SimConnect()
            connected = True
        except KeyboardInterrupt:
            quit()
        except Exception as e:
            # ui.write_message(type(e).__name__, e)
            sleep(5)
    return sm


def main(stdscr):
    from sc_curses import ScCurses

    ui = ScCurses(stdscr)
    config = configparser.ConfigParser()
    config.read("config.ini")

    connected = False
    while not connected:
        ui.update()
        try:
            sm = SimConnect()
            connected = True
            ui.write_message("Connected to simulator.")
        except KeyboardInterrupt:
            quit()
        except Exception as e:
            # ui.write_message(type(e).__name__, e)
            ui.write_message("Connection failed, retrying...")
            ui.update()
            sleep(5)

    flight_data_metrics = FlightDataMetrics(sm)
    srm = SimRateManager(sm, config)
    flight_stability = SimrateDiscriminator(flight_data_metrics, config)
    messages = []
    try:
        have_paused_at_tod = False
        while True:

            try:
                flight_data_metrics.update()
                max_stable_rate = flight_stability.get_max_sim_rate()
                messages = flight_stability.get_messages()
                messages += srm.update(max_stable_rate)
            except TypeError as e:
                messages.append(e)
            except SimConnectDataError:
                messages.append("DATA ERROR: DECEL")
                if config.getboolean("simrate", "decelerate_on_simconnect_error"):
                    srm.decelerate()
            finally:
                update_screen(ui, flight_data_metrics, flight_stability, srm, messages)
                sleep(1)
    except KeyboardInterrupt:
        srm.stop_acceleration()
        sleep(1)
        srm.say_sim_rate()
        del sm
        ui.update()
        sleep(5)
        sys.exit(0)
    except OSError as e:
        raise e
    return 0


def curse(stdscr):
    from sc_curses import ScCurses

    ui = ScCurses(stdscr)
    ui.update(None)
    stdscr.getkey()


if __name__ == "__main__":
    from curses import wrapper

    try:
        wrapper(main)
    except OSError:
        os.system("cls")
        print("Flight Simulator exited. Shutting down.")
        sys.exit()
