from __future__ import print_function
from collections import OrderedDict
import numpy as np

"""
Thorlabs MFC1 : microscope focus controller based on Trinamic TMCM-140-42-SE
and PDx-140-42-SE.

"""


"""
Hardware notes:

* Although the controller supports up to 64 microsteps per full step, the
  actual microstep size is not constant and becomes very nonlinear at around 
  16 microsteps. Using mixed_decay_threshold=-1 helps somewhat.

* Stall detection only works for a limited range of current + speed + threshold

* Encoder has 4096 values per rotation; gear ratio is 1:5.

* Setting encoder prescaler to 8192 yields +1 per encoder step 

"""
try:
    # this is nicer because it provides deadlock debugging information
    from acq4.util.Mutex import RecursiveMutex as RLock
except ImportError:
    from threading import RLock

from pyqtgraph import ptime

from .tmcm import TMCM140, TMCMError



def threadsafe(method):
    # decorator for automatic mutex lock/unlock
    def lockMutex(self, *args, **kwds):
        with self.lock:
            return method(self, *args, **kwds)
    return lockMutex


class MFC1(object):
    def __init__(self, port, baudrate=9600, **kwds):
        self.lock = RLock(debug=True)

        # Constant that affects ability to seek to encoder position.
        # Low values (~10) cause a very slow approach to the target encoder position.
        # High values (~2000) cause the motor to overshoot the target position
        self.tracking_const = kwds.pop('tracking_const', 400)
        
        """
        Parameter dictionaries for two different firmware versions
        """
        params_1140 = OrderedDict([
            ('maximum_current', 100),  # range is 0-255; in 32 steps. Do not make too large
            ('maximum_acceleration',100),
            ('maximum_speed', 800),
            ('ramp_divisor',2),
            ('pulse_divisor',3),
            ('standby_current',0),
            ('encoder_prescaler', 1600),   # See note 6.2 in firmware manual for prescaler value
            ('microstep_resolution',4), # Use 4 for 16 microsteps, and engage interpolation
            ('step_interpolation_enable', 1),  # enable interpolation 1 for new controller
            ('stallGuard2_threshold', 0), # for NEW firmware 1140 only
            ('freewheeling',1),
        ])
        params_110_42 = OrderedDict([
            ('maximum_current', 100),  # range is 0-255; in 32 steps. Do not make too large
            ('maximum_acceleration',200),  # lukes default 1000; I like 200
            ('maximum_speed', 800),  # luke's max 2000, I like 800
            ('ramp_divisor',2),  # lc default 7, I like 2
            ('pulse_divisor',3),
            ('standby_current',0),
            ('mixed_decay_threshold',-1), # for old firmware only
            ('encoder_prescaler', 8192),   #  8192 causes encoder_position to have exactly the same resolution as the encoder itself
            ('microstep_resolution',5), # 32 microsteps. Use 4 and 16 and interpolation enable for
            ('fullstep_threshold', 0), # for old firmware only
            ('stall_detection_threshold', 0), # for old firmware only
            ('freewheeling',1),
        ])

        optional_params = ['mixed_decay_threshold', 'stall_detection_threshold', 'fullstep_threshold']

        self.mcm = TMCM140(port, baudrate)
        self.mcm.get_firmware()  ### 
        self.mcm.stop_program()
        self.mcm.stop()

        # configure parameter dictionaries
        if self.mcm.firmware_version.startswith(b'1140V'):
            params = params_1140
        elif self.mcm.firmware_version.startswith(b'140V4'):
            params = params_110_42
        else:
            print('Firmware {:s} is not recognized; must be 1140V or 140V4'.format(self.mcm.firmware_version))
            raise ValueError('Firmware version error')
        for k, v in kwds.items():
            if k not in params:
                raise NameError("Unknown MFC1 parameter '%s'" % k)
            params[k] = v
        # now set parameters
        self.mcm.set_params(**params)
        for k,v in params.items():
            try:
                self.mcm.set_param(k, v)
            except TMCMError as err:
                if err.status == 3 and k in optional_params:
                    # Some parameters are not supported on all motors; ignore these silently
                    pass
                else:
                    print(k, v)
                    raise
        self._target_position = self.mcm['encoder_position']
        self.mcm.set_global('gp0', self._target_position)
        self._upload_program()

        self._move_status = {}
        self._last_move_id = -1
        
    def _upload_program(self):
        """Upload a program used to seek to a specific encoder value.

        This controls the motor velocity while seeking for a specific encoder 
        value.

        Note: the move command provided by the tmcm firmware only tracks the motor
        microsteps and does not make use of the encoder. Because the microsteps are not
        uniform, it is not possible to reliably move to a specific encoder position
        using the built-in move command.
        """
        m = self.mcm
        max_speed = m['maximum_speed']
        with m.write_program() as p:
            # start with a brief wait because sometimes the first command may be 
            # ignored.
            p.command('wait', 0, 0, 1)

            # distance = target_position - current_position
            p.get_param('encoder_position')   # accu = encoder_position
            p.calcx('load')                   # X = accu
            p.get_global('gp0')               # accu = gp0  (target position)
            p.calcx('sub')                    # accu -= X
            p.set_global('gp1', 'accum')      # gp1 = accu
            
            # if dx is 0, stop motor and program
            p.comp(0)                         
            p.jump('ne', p.count+3)           # if accu != 0: jump 3 instructions ahead
            p.set_param('target_speed', 0)    # else: stop here
            p.set_param('actual_speed', 0)
            p.command('stop', 0, 0, 0)
            
            # calculate distance-to-target where we should begin (de)accelerating
            # set Tx = speed**2 / tracking_const
            p.get_param('actual_speed')         # accu = actual_speed
            p.calcx('load')                     # X = accu
            p.calcx('mul')                      # accu *= X
            p.calc('div', self.tracking_const)  # accu /= tracking_const
            
            # invert threshold if v < 0
            p.calcx('swap')                     
            p.get_param('actual_speed')         
            p.comp(0)                           
            p.calcx('swap')                     
            p.jump('ge', p.count+1)             # if actual_speed < 0:  
            p.calc('mul', -1)                   #   accu *= -1
            p.set_global('gp2', 'accum')        # gp2 = accu  (distance until decel)
            
            # calculate desired speed
            p.calcx('swap')                     # X = accu
            p.get_global('gp1')                 # accu = gp1  (distance to target)
            p.calcx('sub')                      # accu -= X
            p.calcx('swap')                     # X = accu  (distance_to_target - distance_until_decel)
            p.get_param('actual_speed')         # accu = actual_speed
            p.calcx('add')                      # accu += X
            p.calc('mul', 2)                    # accu *= 2/3
            p.calc('div', 3)
            
            # new_speed = clip(new_speed, -2047, 2047)
            p.comp(max_speed)
            p.jump('gt', p.count+3)
            p.comp(-max_speed)
            p.jump('lt', p.count+3)
            p.jump(p.count+3)
            p.calc('load', max_speed)
            p.jump(p.count+1)
            p.calc('load', -max_speed)
            
            # 0 speed should never be requested if there is an offset
            p.comp(0)
            p.jump('ne', p.count+1)
            p.get_global('gp1')
            
            
            # output and repeat
            p.set_param('target_speed', 'accum')
            #p.set_param('actual_speed', 'accum')
            p.jump(1)

    def position(self):
        """Return the current encoder position.
        """
        pos = self.mcm['encoder_position']
        nsamp = 8
        pos_i = np.zeros(nsamp)
        for i in range(nsamp):  # average
            pos_i[i]= self.mcm['encoder_position']
        pos = np.median(pos_i)  # clean up encoder measurement.
        if not self.program_running():
            # when program is not running, target position should follow actual position
            self._target_position = pos
            self.set_holding(False)
        return pos
    
    @threadsafe
    def target_position(self):
        """Return the final target position if the motor is currently moving
        to a specific position.

        If the motor is stopped or freely rotating, return the current position.
        """
        return self._target_position

    @threadsafe
    def move(self, position):
        """Move to the requested position.

        If the motor is already moving, then update the target position.
        
        Return an object that may be used to check 
        whether the move is complete (see move_status).
        """
        id = self._last_move_id

        if self.program_running():
            self._interrupt_move()
            self.mcm.set_global('gp0', position)
            start = ptime.time()
        else:
            self.mcm.set_global('gp0', position)
            self.mcm.start_program()
            start = ptime.time()

        self._target_position = position

        id += 1
        self._last_move_id = id
        self._move_status[id] = {'start': start, 'status': 'moving', 'target': position}
        return id

    @threadsafe
    def move_status(self, id, clear=True):
        """Return the status of a previously requested move.

        The return value is a dict with the following keys:

        * status: 'moving', 'interrupted', 'failed', or 'done'.
        * start: the start time of the move.
        * target: the target position of the move.
        """
        stat = self._move_status[id]
        if stat['status'] == 'moving' and not self.program_running():
            pos = self.position()
            if abs(pos - stat['target']) <= 3:  # can we get the tolerance lower?  ### try 4?
                stat['status'] = 'done'
            else:
                stat['status'] = 'failed'
                stat['final_pos'] = pos

        if clear and stat['status'] != 'moving':
            del self._move_status[id]

        return stat
        
    @threadsafe
    def rotate(self, speed):
        """Begin rotating at *speed*. Positive values turn right.
        """
        self._interrupt_move()
        self.mcm.stop_program()
        self.mcm.rotate(speed)

    def _interrupt_move(self):
        """If a move is currently in progress, set its state to 'interrupted'
        """
        id = self._last_move_id
        try:
            stat = self.move_status(id, clear=False)
            if stat['status'] == 'moving':
                self._move_status[id]['status'] = 'interrupted'
        except KeyError:
            pass
    
    def stop(self):
        """Immediately stop the motor and any programs running on the motor
        comtrol module.
        """
        self._interrupt_move()
        self.mcm.stop_program()
        self.mcm.stop()

    def program_running(self):
        return self.mcm.get_global('tmcl_application_status') == 1

    def set_encoder(self, x):
        self.mcm['encoder_position'] = x

    def set_holding(self, hold):
        """Set whether the motor should hold its position (True) or should
        power down and allow freewheeling (False).
        """
        if hold:
            self.mcm['standby_current'] = self.mcm['maximum_current']
        else:
            self.mcm['standby_current'] = 0
