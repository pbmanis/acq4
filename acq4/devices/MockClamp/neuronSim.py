from __future__ import print_function
"""
Use NEURON to simulate a simple cell for testing with MockClamp
"""
import numpy as np
from pathlib import Path
import sys, os
import platform
# Try standard locations to find neuron library

print(platform.system)
# try to load extra mechanisms
# if platform.system == 'Linux':
#     for nrnpath in ['/usr/local/nrn/lib/python']:
#     if os.path.isdir(nrnpath):
#         sys.path.append(nrnpath)
#     from .neuron import h
#     from . import neuron
#     for name in ('i386', 'x86_64'):
#         mechlib = os.path.join(os.path.dirname(__file__), name + '/.libs/libnrnmech.so')
#         print("NEURON load (Linux):", mechlib)
#         if os.path.isfile(mechlib):
#             h.nrn_load_dll(mechlib)

# elif platform.system == 'Darwin':
#     from neuron import h
#     import neuron

#     for name in ('arm64', 'x86_64'):
#         meclab = os.path.join(os.path.dirname(__file__), name + '/.libs/libnrnmech.dylib')
#         print("NEURON load (arm64):", mechlib)
#         if os.path.isfile(mechlib):
#             h.nrn_load_dll(mechlib)
# else:
#     raise Exception("Unsupported platform: %s" % platform.system)

from neuron import h
import neuron
from neuron.units import ms, mV

h.celsius = 22

soma = h.Section()
soma.insert('hh')
soma.insert('pas')
#soma.insert('hcno')
soma.L = 20
soma.diam = 20
soma(0.5).pas.g = 2e-5
#soma(0.5).hcno.gbar = 15e-4

ic = h.IClamp(soma(0.5))
ic.dur = 1e9
ic.delay = 0
icRec = h.Vector()
icRec.record(soma(0.5)._ref_v)

vc = h.SEClamp(soma(0.5))
vc.dur1 = 1e9
vc.rs = 5  # Rs, in megohms
#vc.amp1 = 0
#vc.dur2 = 1e9
vcrs = vc.rs
vcRec = h.Vector()
vcRec.record(vc._ref_i)

# add an alpha synapse, reversal -7 mV
# note that synaptic current will appear in the SEClamp or IClamp
syn = h.AlphaSynapse(soma(0.5))



t = 0
    
def run(cmd):
    global t, soma, ic, vc, syn, icRec, vcRec, vcrs
    icRec.clear()
    vcRec.clear()
    
    dt = cmd['dt'] * 1e3  ## convert s -> ms
    h.dt = dt
    data = cmd['data']
    mode = cmd['mode']
    #print "data:", data.min(), data.max()

    #times = h.Vector(np.linspace(h.t, h.t+len(data)*dt, len(data)))
    #print "times:", times.min(), times.max()
    if mode == 'ic':
        #ic.delay = h.t
        ic.delay = 0
        vc.rs = 1e9
        im = h.Vector(data)
        im.play(ic._ref_amp, dt)

    elif mode == 'vc':
        #vc.amp1 = data[0]
        vc.rs = vcrs
        ic.delay = 1e9
        #vc.dur1 = h.t
        vm = h.Vector(data)
        vm.play(vc._ref_amp1, dt)

        syn.onset = 400. #ms
        syn.tau = 1.5 # ms
        syn.gmax = 0.04 # umho
        syn.e = -7.0 # mV
        #syn.i --- nA
        
    else:
        sys.stderr.write("Unknown mode '%s'" % cmd['mode'])
        raise Exception("Unknown mode '%s'" % cmd['mode'])

    #t2 = t + dt * (len(data)+2)
    #print "run until:", t2


    h.finitialize(-65. * mV)
    tstop = (dt*len(data)+2)
    neuron.run(tstop * ms) #dt * (len(data)+2))
    #neuron.run(t2)
    #t = t2

    #print len(out), out
    #out = np.array(out)[:len(data)]

    if mode == 'ic':
        out = np.array(icRec)[:len(data)] * 1e-3 #  + np.random.normal(size=len(data), scale=0.3e-3)
    elif mode == 'vc':
        out = np.array(vcRec)[:len(data)] * 1e-9 #  + np.random.normal(size=len(data), scale=3.e-12)
    return out


# provide a visible test to make sure code is working and failures are not ours.
# call this from the command line to observe the clamp plot results
#
if __name__ == '__main__':
    import pyqtgraph as pg
    from acq4.util import Qt
    app = Qt.QApplication([])
    win = pg.GraphicsLayoutWidget()
    mode = sys.argv[1].lower()
    win.resize(1000,600)
    win.setWindowTitle('Testing hhSim.py')
    p = win.addPlot(title=mode)
    npts = 10000
    x1 = 2000
    x2 = 7000
    if mode == 'vc':
        x = np.arange(-100, 41, 10)*1e-3 # clamp v in V
        cmd = np.ones((len(x), npts))*-65.0*1e-3
    elif mode == 'ic':
        x = np.arange(-1000, 1000, 200)*1e-3 # in A
        cmd = np.zeros((len(x), npts))
    data = np.zeros((len(x), npts))
    dt = 1e-4
    tb = np.arange(0, npts*dt, dt)
    for i, v in enumerate(x):
        print('V: ', v)
        cmd[i, x1:x2] = v
        opts = {
            'mode': mode.lower(),
            'dt': dt,
            'data': cmd[i,:]
        }
        data[i,:] = run(opts)
        p.plot(tb, data[i])
    win.show()
    Qt.QApplication.instance().exec()
    