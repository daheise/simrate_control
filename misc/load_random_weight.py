from time import sleep
from SimConnect import *
from math import ceil
import random
import sys

class WeightManager:

    def __init__(self) -> None:
        self.sm = SimConnect()
        self.aq = AircraftRequests(self.sm)
        self.ae = AircraftEvents(self.sm)
        self.extra_weight = 0
        self._request_sleep = 0.01
        self._min_request_sleep = 0.01
        self._max_request_sleep = 0.5

    def _get_value(self, aq_name, retries=sys.maxsize):
        # PySimConnect seems to crash the sim if requests happen too fast.
        sleep(self._request_sleep)
        val = self.aq.find(str(aq_name)).value
        i = 0
        while val is None and i < retries:
            i += 1
            self._request_sleep = min(
                self._request_sleep + 0.05 * i, self._max_request_sleep
            )
            sleep(self._request_sleep)
            val = self.aq.find(str(aq_name)).value
        if i > 0:
            self.messages.append(f"Warning: Retried {aq_name} {i} times.")
        self._request_sleep = max(self._min_request_sleep, self._request_sleep - 0.01)
        return val

    def load_weight(self, weight: int):
        weight += self.extra_weight
        num_payload_bays = int(self._get_value("PAYLOAD_STATION_COUNT"))
        payload_per_bay = int(weight/num_payload_bays)
        payloads = []
        for i in range(1, num_payload_bays+1):
            payloads.append(self._get_value(f"PAYLOAD_STATION_WEIGHT:{i}"))
        for i in range(1, num_payload_bays+1):
            self.aq.set(f"PAYLOAD_STATION_WEIGHT:{i}", payload_per_bay)
        payloads = []
        for i in range(1, int(num_payload_bays+1)):
            payloads.append(self._get_value(f"PAYLOAD_STATION_WEIGHT:{i}"))
        return sum(payloads)

    @property
    def fuel_weight(self):
        w = self._get_value("FUEL_TOTAL_QUANTITY_WEIGHT")
        return w

    @property
    def maximum_payload(self):
        fuel_weight = self.fuel_weight
        max_gross = self._get_value("MAX_GROSS_WEIGHT")
        empty_weight = self._get_value("EMPTY_WEIGHT")
        w = max_gross - empty_weight - fuel_weight
        return w
    
    @property
    def random_weight(self):
        # Minimum weight is at least one pilot
        minimum_weight = random.triangular(120, 250, 150)
        low = minimum_weight
        high = self.maximum_payload
        total_payload = random.triangular(low, high)
        return total_payload


def main():
    loader = WeightManager()
    print(loader.load_weight(loader.random_weight))

main()