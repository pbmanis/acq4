import os, sys, time
path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.append(path)

from acq4.drivers.ThorlabsMFC1 import MFC1
import acq4.pyqtgraph as pg
import time

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("comport", help="comport (device may be com3, /dev/ttyACM0, etc.)")
parser.add_argument('-p', dest='plot_motion', action='store_true',
                    help='Plot Motion with move (test for target position stability)')
parser.add_argument('-e', dest='encoding', action='store_true',
                    help='Test encoder stability by reading encoder repeatedly')
args = parser.parse_args()
mfc = MFC1(args.comport)

if args.encoding:
    print('test encoding')
    start = time.time()
    while time.time() < start+10: # 10 sec test
        print('pos: ', mfc.position())
        lt = time.time()
        while time.time() - lt < 0.25:
            continue


if args.plot_motion:
    print "pos:", mfc.position()

    def plot_motion():
        app = pg.mkQApp()
        win = pg.GraphicsWindow()
        p1 = win.addPlot(title='position')
        p2 = win.addPlot(title='speed', row=1, col=0)
        p3 = win.addPlot(title='target speed', row=2, col=0)
        p4 = win.addPlot(title='distance', row=3, col=0)
        p2.setXLink(p1)
        p3.setXLink(p1)
        p4.setXLink(p1)

        t = []
        x = []
        v = []
        vt = []
        dx = []
        dxt = []
        start = time.time()
        started = False
        while True:
            now = time.time()
            pos = mfc.position()
            if not started and now - start > 0.2:
                move = pos + 20000
                print "move:", move
                mfc.move(move)
                started = True
            if now - start > 2:
                break
            t.append(now-start)
            x.append(pos)
            v.append(mfc.mcm['actual_speed'])
            vt.append(mfc.mcm['target_speed'])
            dx.append(mfc.mcm.get_global('gp1'))
            dxt.append(mfc.mcm.get_global('gp2'))
        p1.plot(t, x, clear=True)
        p2.plot(t, v, clear=True)
        p3.plot(t, vt, clear=True)
        p4.plot(t, dx, clear=True)
        p4.plot(t, dxt, pen='g')

        print "Final:", mfc.position()
        return app, win

    app, win = plot_motion()
    if (sys.flags.interactive != 1) or not hasattr(pg.Qt.QtCore, 'PYQT_VERSION'):
            pg.Qt.QtGui.QApplication.instance().exec_()
