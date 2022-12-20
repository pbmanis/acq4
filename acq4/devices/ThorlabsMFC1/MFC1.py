# -*- coding: utf-8 -*-
from __future__ import print_function
import numpy as np
from acq4.util import Qt
from ..Stage import Stage, StageInterface, MoveFuture
from acq4.drivers.ThorlabsMFC1 import MFC1 as MFC1_Driver
from acq4.util.Mutex import Mutex
from acq4.util.Thread import Thread
from pyqtgraph import debug
import time

class ChangeNotifier(Qt.QObject):
    sigPosChanged = Qt.Signal(object, object, object)


class ThorlabsMFC1(Stage):
    """Thorlabs motorized focus controller (MFC1)
    """

    def __init__(self, man, config, name):
        self.port = config.pop('port')
        self.scale = config.pop('scale', 1)
        params = config.pop('motorParams', {})
        # Optionally read limits from config
        lims = config.pop('limits', (None, None, None))
        self.setLimits(z=lims["z"]) 
        self.dev = MFC1_Driver(self.port, **params)
    #    self.parent = config.pop('parentDevice')  # needed to get xy position
        man.sigAbortAll.connect(self.dev.stop)

        # Optionally use ROE-200 z axis to control focus
        roe = config.pop('roe', None)
        self._roeDev = None
        self._roeEnabled = "waiting"  # ROE control is disabled until after the first update
        if roe is not None:
            dev = man.getDevice(roe)
            self._roeDev = dev
            # need to connect to internal change signal because 
            # the public signal should already have z-axis information removed.
            dev._notifier.sigPosChanged.connect(self._roeChanged)

        self._lastPos = None

        Stage.__init__(self, man, config, name)

        self.getPosition(refresh=True)

        self._monitor = MonitorThread(self)
        self._monitor.start()

    def axes(self):
        # device only has axes 'z', but must have all 3 re capabilities
        return 'z'
    
    def capabilities(self):
        # device only reads/writes z-axis
        return {
            'getPos': (False, False, True),
            'setPos': (False, False, True),
            'limits': (False, False, True),
        }

    def _setHardwareLimits(self, axis:int, limit:tuple):
        if axis != 2:
            raise ValueError("Thorlabs MFC1: Can only set z limits")
        self._limits = (None, None, limit)

    def mfcPosChanged(self, pos, oldpos):
        self.posChanged(pos)

    def _getPosition(self):
        poss = self.dev.position() * self.scale
        self._parent = self.parentDevice()
        if self._parent is not None:
            ppos = self._parent.getPosition()
        else:
            ppos = [0]*3
        pos = [ppos[0], ppos[1], poss]
        if self._lastPos is None: ###
            self.posChanged(pos)
        elif len(self._lastPos) == 1 and poss != self._lastPos: ###NEEDS TO BE FIXED: sometimes pos is tuple, sometimes 3 tuples!
            self.posChanged(pos)
        elif len(self._lastPos) == 3 and poss != self._lastPos[2]:
            self.posChanged(pos)
        return pos

    def _move(self, pos, speed, linear=None):
        pos = self._toAbsolutePosition(pos)
        limits = self.getLimits()  ### NEEDS TO BE FIXED: sometimes pos is tuple, sometimes 3 tuples!
        if limits[0][0] is not None:
            pos = max(pos[2], limits[0])
        if limits[0][1] is not None:
            pos = min(pos, limits[1])
        return MFC1MoveFuture(self, pos, speed)

    def targetPosition(self):
        return [self.dev.target_position() * self.scale]

    def quit(self):
        self._monitor.stop()
        Stage.quit(self)

    def _roeChanged(self, drive, pos, oldpos):
        if drive != self._roeDev.drive:
            return
        if self._roeEnabled is not True:
            if self._roeEnabled == 'waiting':
                self._roeEnabled = True
            return
        dpos = np.linalg.norm(np.array(pos, dtype=float) - np.array(oldpos, dtype=float))
        if np.abs(dpos) < 1e-7: ### # min 0.1 micron
            return
        # print("target: ", self.targetPosition(), dpos)
        # print("dev target: ", self.dev.target_position())
        # print("lastpos: ", self._lastPos)
        target = oldpos + dpos # self.dev.target_position() + dpos
        # print("new target: ", target)
        self.dev.set_holding(False)  ###
        self._move([None, None, target[2]], 'fast')
        self._lastPos = pos

    def deviceInterface(self, win):
        return MFC1StageInterface(self, win)

    def setRoeEnabled(self, enable):
        self._roeEnabled = enable

    def setZero(self):
        """Reset the device position to 0 (without moving the motor).
        """
        self.dev.set_encoder(0)
        self._getPosition()

    def stop(self):
        self.dev.stop()

    def setHolding(self, hold):
        self.dev.set_holding(hold)


class MonitorThread(Thread):
    def __init__(self, dev):
        self.dev = dev
        self.lock = Mutex(recursive=True)
        self.stopped = False
        self.interval = 0.1 ### was 0.3
        Thread.__init__(self)

    def start(self):
        self.stopped = False
        Thread.start(self)

    def stop(self):
        with self.lock:
            self.stopped = True

    def setInterval(self, i):
        with self.lock:
            self.interval = i

    def run(self):
        minInterval = 100e-3
        interval = minInterval
        lastPos = None
        while True:
            try:
                with self.lock:
                    if self.stopped:
                        break
                    maxInterval = self.interval
                pos = self.dev._getPosition()
                if pos != lastPos:
                    # stage is moving; request more frequent updates
                    interval = minInterval
                else:
                    interval = min(maxInterval, interval*2)
                lastPos = pos

                time.sleep(interval)
            except:
                debug.printExc('Error in MFC1 monitor thread:')
                time.sleep(maxInterval)


class MFC1StageInterface(StageInterface):
    def __init__(self, dev, win):
        StageInterface.__init__(self, dev, win)
        if dev._roeDev is not None:
            self.btnLayout.setContentsMargins(0, 0, 0, 0)
            self.connectRoeBtn = Qt.QPushButton('Enable ROE')
            self.connectRoeBtn.setCheckable(True)
            self.connectRoeBtn.setChecked(True)
            row = self.layout.rowCount()
            self.layout.addWidget(self.connectRoeBtn, row, 0, 1, 1) # self.nextRow, 0, 1, 2)
            self.connectRoeBtn.toggled.connect(self.connectRoeToggled)

            self.setZeroBtn = Qt.QPushButton('Set Zero')
            self.layout.addWidget(self.setZeroBtn, row, 1, 1, 1)
            self.setZeroBtn.clicked.connect(self.setZeroClicked)

    def setZeroClicked(self):
        self.dev.setZero()

    def connectRoeToggled(self, b):
        self.dev.setRoeEnabled(b)


class MFC1MoveFuture(MoveFuture):
    """Provides access to a move-in-progress on an MPC200 drive.
    """
    def __init__(self, dev, pos, speed):
        MoveFuture.__init__(self, dev, pos, speed)
        self.startPos = dev.getPosition()
        self.stopPos = pos
        self._moveStatus = {'status': None}
        self.id = dev.dev.move(pos[2] / dev.scale)

    def wasInterrupted(self):
        """Return True if the move was interrupted before completing.
        """
        return self._getStatus()['status'] in ('interrupted', 'failed')

    def percentDone(self):
        """Return an estimate of the percent of move completed based on the 
        device's speed table.
        """
        if self.isDone():
            return 100

        pos = self.dev.getPosition() - self.startPos
        target = self.stopPos - self.startPos
        if target == 0:
            return 99
        return 100 * pos / target

    def isDone(self):
        """Return True if the move is complete.
        """
        return self._getStatus()['status'] in ('interrupted', 'failed', 'done')

    def errorMessage(self):
        stat = self._getStatus()
        if stat['status'] == 'interrupted':
            return "move was interrupted"
        elif stat['status'] == 'failed':
            return "did not reach the expected position (%s != %s)" % (stat['final_pos'], stat['target'])
        else:
            return None

    def _getStatus(self):
        # check status of move unless we already know it is complete.
        if self._moveStatus['status'] in (None, 'moving'):
            self._moveStatus = self.dev.dev.move_status(self.id)
        return self._moveStatus
        


