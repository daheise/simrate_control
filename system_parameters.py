from tkinter import Y
from sc_config import SimrateControlConfig

# from lib.simconnect_mobiflight import SimConnectMobiFlight
from collections import deque
from sys import maxsize
from math import ceil, radians, degrees, sqrt, tan, sin, cos, asin, atan2, log2
from time import sleep
import subprocess as sp
import os

class SimConnectDataError(Exception):
    pass


class SystemDataMetrics:
    def __init__(self, config: SimrateControlConfig):
        self.memory_utilization = 0.0
        self.update()

    def get_gpu_memory_utilization(self):
        command = "nvidia-smi --query-gpu=memory.free --format=csv"
        memory_free_info = sp.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
        memory_free_values = [int(x.split()[0]) for i, x in enumerate(memory_free_info)]
        command = "nvidia-smi --query-gpu=memory.total --format=csv"
        memory_total_info = sp.check_output(command.split()).decode('ascii').split('\n')[:-1][1:]
        memory_total_values = [int(x.split()[0]) for i, x in enumerate(memory_total_info)]
        return 1-(memory_free_values[0]/memory_total_values[0])

    def get_network_usage(self):
        #Powershell Mbps
        # I think ~20Mbps is an indicator of downloading photogrammetry
        #(get-counter "\Network Interface(*ethernet*)\Bytes Total/sec").CounterSamples.CookedValue*8/1000000
        return 0

    def get_messages(self):
        return []

    def update(self):
        self.memory_utilization = self.get_gpu_memory_utilization()