from __future__ import print_function

import collections
import time
import weakref
from pathlib import Path

import numpy as np
import pyqtgraph as pg

import acq4.util.InterfaceCombo  # just to register 'interface' parameter type
from acq4.modules.Module import Module
from acq4.util import Qt
from acq4.util.DataManager import getDirHandle

class Tasker(Module):
    """Used to automatically run multiple protocols in sequence.
    """

    moduleDisplayName = "Tasker"
    moduleCategory = "Utilities"

    def __init__(self, manager, name, config):
        Module.__init__(self, manager, name, config)
        self.manager = manager

        self.running = False
        self.startTime = None
        self.isStarted = False
        self.test_mode = False
        self.win = Qt.QSplitter()

        if "Task Runner" not in self.manager.modules:
            raise ValueError("Tasker requires a loaded Task Runner module")
        self.TR = self.manager.modules["Task Runner"]

        self.ctrlWidget = pg.LayoutWidget()
        self.win.addWidget(self.ctrlWidget)

        self.loadBtn = Qt.QPushButton("Load Task List")
        self.addBtn = Qt.QPushButton("Add Task to List")
        self.delBtn = Qt.QPushButton("Delete Task from List")
        self.saveBtn = Qt.QPushButton("Save Task List")

        self.startBtn = Qt.QPushButton("Start")
        self.startBtn.setCheckable(True)
        self.testBtn = Qt.QPushButton("Test")
        self.testBtn.setCheckable(True)
        self.fileLabel = Qt.QLabel()

        self.ctrlWidget.addWidget(self.loadBtn, 0, 0)
        self.ctrlWidget.addWidget(self.addBtn, 0, 1)
        self.ctrlWidget.addWidget(self.delBtn, 0, 2)
        self.ctrlWidget.addWidget(self.saveBtn, 0, 3)

        self.ctrlWidget.addWidget(self.startBtn, 1, 0)
        self.ctrlWidget.addWidget(self.testBtn, 2, 0)
        self.ctrlWidget.addWidget(self.fileLabel, 3, 0)
        self.listWidget = Qt.QListWidget()
        self.listWidget.setDragDropMode(Qt.QAbstractItemView.DragDropMode.InternalMove)
        self.ctrlWidget.addWidget(self.listWidget, 1, 1, 4, 3)

        self.loadBtn.clicked.connect(self.loadClicked)
        self.startBtn.toggled.connect(self.startToggled)
        self.testBtn.toggled.connect(self.test_runOnce)

        self.params = pg.parametertree.Parameter.create(
            name="params",
            type="group",
            children=[
                dict(
                    name="interval",
                    type="float",
                    value=10,
                    suffix="s",
                    siPrefix=True,
                    limits=[0.001, None],
                    step=1.0,
                ),
            ],
        )
        self.ptree = pg.parametertree.ParameterTree()
        self.ptree.setParameters(self.params)
        self.ctrlWidget.addWidget(self.ptree, 4, 0)

        self.channelLayout = Qt.QSplitter()
        self.win.addWidget(self.channelLayout)

        first_task = self.manager.dirHandle(
            "/Users/pbmanis/Desktop/acq4/config/example/protocols/CCIV"
        )
        second_task = self.manager.dirHandle(
            "/Users/pbmanis/Desktop/acq4/config/example/protocols/CCIV"
        )
        self.task_list = [first_task, second_task]

        self.win.show()

        self.timer = Qt.QTimer()
        self.timer.timeout.connect(self.runOnce)

    def quit(self):
        self.startBtn.setChecked(False)
        Module.quit(self)

    def test_runOnce(self):
        if self.testBtn.isChecked():
            self.test_mode = True
            self.runOnce()
            self.testBtn.setChecked(False)
            self.test_mode = False

    def runOnce(self):
        if self.running:
            return
        self.running = True
        for task in self.task_list:
            shortTaskName = Path(task.name()).stem
            try:
                self.TR.loadTask(task)
                self.fileLabel.setText(f"Running: {shortTaskName:s}")
                if not self.test_mode:
                    self.TR.runSequence()
                else:
                    print("Test mode: not running sequence")
            except:
                self.running = False
            self.fileLabel.setText(f"Completed: {shortTaskName:s}")

    def startToggled(self):
        if not self.isStarted:
            try:
                # if self.recordDir is None or not self.recordWritable:
                #     self.newRecord()
                if self.startTime is None:
                    self.startTime = time.time()
                self.timer.start(int(self.params["interval"] * 1000))
                self.isStarted = True
                self.runOnce()
            except:
                self.startBtn.setChecked(False)
                raise

            self.startBtn.setText("Stop")
        else:
            self.timer.stop()
            self.isStarted = False
            self.startBtn.setText("Start")

    def loadClicked(self):
        return
        try:
            startDir = self.manager.getCurrentDir()
        except Exception:
            startDir = self.manager.getBaseDir()
        dirname = Qt.QFileDialog.getExistingDirectory(
            self.win, "Open Record", startDir.name()
        )
        if dirname == "":
            return
        self.recordDir = getDirHandle(dirname)
        self.recordWritable = False
        self.updateFileLabel()
        self.clearChannels()

        for dev in self.recordDir.ls():
            w = self.addChannel(dev, mode=None, recordDir=self.recordDir)
