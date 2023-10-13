from __future__ import print_function

import collections
import os
import time
import weakref
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from acq4.util.HelpfulException import HelpfulException
import acq4.util.DirTreeWidget as DirTreeWidget
import acq4.util.InterfaceCombo  # just to register 'interface' parameter type
from acq4.modules.Module import Module
from acq4.util import Qt
from acq4.util.DataManager import getDirHandle

class Loader(DirTreeWidget.DirTreeSelector):
    def __init__(self, host, baseDir):
        DirTreeWidget.DirTreeSelector.__init__(self, baseDir, create=True)
        self.host = host

    def add(self, fileHandle):
        self.host.addTask(fileHandle)
        return True
    
    def remove(self, fileHandle):
        self.host.removeTask(fileHandle)
        return True
    
    def refresh(self):
        self.host.refreshTaskList()
        return True

    # def load(self, handle):
    #     self.host.loadTask(handle)
    #     return True

    # def save(self, handle):
    #     self.host.saveTask(handle)
        return True


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
            raise ValueError("Tasker requires a loaded *Task Runner* module")
        self.TR = self.manager.modules["Task Runner"]
        self.loaderWidget = pg.LayoutWidget()
        self.loaderWidget.setSizePolicy(Qt.QtWidgets.QSizePolicy.Policy.Expanding, Qt.QtWidgets.QSizePolicy.Policy.Expanding)
        self.win.addWidget(self.loaderWidget)
        self.loaderWidget.setMaximumWidth(250)
        self.ctrlWidget = pg.LayoutWidget()
        self.ctrlWidget.setSizePolicy(Qt.QtWidgets.QSizePolicy.Policy.Expanding, Qt.QtWidgets.QSizePolicy.Policy.Expanding)
        self.win.addWidget(self.ctrlWidget)
        self.ctrlWidget.setMaximumWidth(200)
        self.protocolWidget = pg.LayoutWidget()
        self.protocolWidget.setSizePolicy(Qt.QtWidgets.QSizePolicy.Policy.Expanding, Qt.QtWidgets.QSizePolicy.Policy.Expanding)
        self.win.addWidget(self.protocolWidget)

        self.loadBtn = Qt.QPushButton("Load Task List")
        # self.addBtn = Qt.QPushButton("Add Task to List")
        # self.delBtn = Qt.QPushButton("Delete Task from List")
        self.saveBtn = Qt.QPushButton("Save Task List")

        self.startBtn = Qt.QPushButton("Start")
        self.startBtn.setCheckable(True)
        self.testBtn = Qt.QPushButton("Test")
        self.testBtn.setCheckable(True)
        self.fileLabel = Qt.QLabel()

        self.protocolWidget.addWidget(self.loadBtn, 0, 1, colspan=2)
        # self.ctrlWidget.addWidget(self.addBtn, 0, 2)
        # self.ctrlWidget.addWidget(self.delBtn, 0,3)
        self.protocolWidget.addWidget(self.saveBtn, 0, 3, colspan=2)

        self.ctrlWidget.addWidget(self.startBtn, 1, 0)
        self.ctrlWidget.addWidget(self.testBtn, 2, 0)
        self.ctrlWidget.addWidget(self.fileLabel, 3, 0)
        self.listWidget = Qt.QListWidget()
        self.listWidget.setDragDropMode(Qt.QAbstractItemView.DragDropMode.InternalMove)
        self.protocolWidget.addWidget(self.listWidget, 1, 1, colspan=4, rowspan=6)

        self.loadBtn.clicked.connect(self.loadClicked)
        self.startBtn.toggled.connect(self.startToggled)
        self.testBtn.toggled.connect(self.test_runOnce)
        # self.addBtn.clicked.connect(self.addTask)
        # self.delBtn.clicked.connect(self.removeTask)


        try:
            try:
                taskDir = config['taskDir']
            except KeyError:
                taskDir = os.path.join(self.manager.configDir, "protocols")
            self.taskList = Loader(self, taskDir)
        except KeyError:
            raise HelpfulException("Config is missing 'taskDir'; cannot load task list.")
        self.loaderWidget.addWidget(self.taskList)

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
        self.channelLayout.setSizes([1, 8])
        self.win.addWidget(self.channelLayout)
        protocoldir = '/Users/Experimenters/acq4/config/protocols'
        self.task_list = []        
        self.win.show()

        self.timer = Qt.QTimer()
        self.timer.timeout.connect(self.runOnce)

    def quit(self):
        self.startBtn.setChecked(False)
        Module.quit(self)

    def updateTaskList(self):
        self.taskList = []
        self.taskList = [self.listWidget.item(i).text() for i in range(self.listWidget.count())]


    def addTask(self, fileHandle):
        """addTask adds task from the protocol list

        Parameters
        ----------
        fileHandle : _type_
            _description_
        """

        self.listWidget.addItem(fileHandle.name())
        self.updateTaskList()

    def removeTask(self, fileHandle):
        """removeTask removes protocol (task) from the task list

        Parameters
        ----------
        fileHandle : _type_
            _description_
        """
        items = self.listWidget.selectedItems()
        if len(items) == 0:
            return
        for item in items:
            self.listWidget.takeItem(self.listWidget.row(item))
        self.updateTaskList()

    def refreshTaskList(self):
        pass

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