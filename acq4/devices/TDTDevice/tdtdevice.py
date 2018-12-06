# -*- coding: utf-8 -*-
from tdt import *    
from acq4.devices.Device import *
import time, traceback, sys, threading
#from taskGUI import *
#from numpy import byte
import numpy as np
#from scipy.signal import resample, bessel, lfilter
import scipy.signal, scipy.ndimage
import acq4.util.advancedTypes as advancedTypes
from acq4.util.debug import *
from acq4.util.Mutex import Mutex
from acq4.pyqtgraph import ptime
from win32com.client import Dispatch
import pythoncom
from .TDTtaskGUI import TDTTaskGui

# For reference: TDT RP2.1 sample rates:
# 0 = 6K, 1 = 12K, 2 = 25k, 3 = 50k, 4 = 100k, 5 = 200k, > 5 is not defined.
# samp_cof_flag =4; % 4 is for 100 kHz
# samp_flist = [6103.5256125, 122107.03125, 24414.0625, 48828.125, ...
#    97656.25, 195312.5];

class TDTDevice(Device):

    threads = {}

    def __init__(self, dm, config, name):
        Device.__init__(self, dm, config, name)
        self.config = config
        self.lock = Mutex(Mutex.Recursive)
        ## make local copy of device handle
        
    def createTask(self, cmd, parentTask):
        return TDTTask(self, cmd, parentTask)

    def taskInterface(self, task):
        """Return a widget with a UI to put in the task rack.
        Currently only has options for attenuation."""
        return TDTTaskGui(self, task)        

    def initThread(self):
        with self.lock:
            isGuiThread = QtCore.QThread.currentThread() == QtCore.QCoreApplication.instance().thread()
            if isGuiThread:
                # main thread is already initialized
                print('TDT Task GUI THREAD alreay initialized')
                return 
            tid = threading.current_thread()
            pythoncom.CoInitialize()
           # time.sleep(0.01)
            if tid not in self.threads:
                self.threads[tid] = None


class TDTTask(DeviceTask):
    def __init__(self, dev, cmd, parentTask):
        DeviceTask.__init__(self, dev, cmd, parentTask)
        dev.initThread()  # make sure COM is initialized for the current thread
        # print(dir(self.dev))
        self.lastPulseTime = None
        self.cmd = cmd
        self.circuits = {}
        self.attens = {}

    def configure(self):
        with self.dev.lock:
            for key, val in self.cmd.items():
                if key.startswith('PA5.'):
                    index = int(key[4:])
                    att = Dispatch('PA5.x')
                    att.ConnectPA5('USB', index)
                    att.SetAtten(val['attenuation'])
                    self.attens[key] = att

                elif key.startswith('RP2.'):
                    index = int(key[4:])
                    circuit = DSPCircuit(val['circuit'], 'RP2', fs=4)  # run at 100 kHz (200 maybe not working?)
                    assert circuit.is_connected
                    for tagName, tagVal in val['tags'].items():
                        circuit.set_tag(tagName, tagVal)
                    self.circuits[key] = circuit

            # self.circuit = DSPCircuit('C:\Users\Experimenters\Desktop\ABR_Code\FreqStaircase3.rcx', 'RP2')

    def start(self):
        with self.dev.lock:
            for circuit in self.circuits.values():
                circuit.start()
                circuit.trigger(1, mode='pulse')
            self.starttime = ptime.time()

    def isDone(self):
        if len(self.circuits) == 1:
            # TODO: need some standard way of asking whether a circuit has finished..
            with self.dev.lock:
                return bool(self.circuit.get_tag('Pulses'))
        else:
            return True

    def stop(self, abort=False):
        with self.dev.lock:
            for circuit in self.circuits.values():
                circuit.stop()
        with self.dev.lock:
            for att in self.attens.values():
                att.SetAtten(120)

    def getPrepTimeEstimate(self):
           # allow time for USB and device response
            return 0.05