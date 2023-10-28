# -*- coding: utf-8 -*-
from __future__ import print_function

import six

import acq4.util.DataManager as DataManager
from acq4.util import Qt
from acq4.util.debug import printExc

Ui_Form = Qt.importTemplate('.DirTreeSelectorTemplate')


class DirTreeSelector(Qt.QWidget):
    """DirTreeSelector For Tasker:
    allow selection or deselection of protocols to add to the task list
    No operations on the protocols is allowed (e.g. manage all of the
    protocols from within the TaskRunner module)

    Parameters
    ----------
    Qt : _type_
        _description_

    Returns
    -------
    _type_
        _description_

    Raises
    ------
    Exception
        _description_
    Exception
        _description_
    Exception
        _description_
    Exception
        _description_
    """
    
    sigCurrentFileChanged = Qt.Signal(object, object, object)
    
    def __init__(self, baseDir, sortMode='alpha', create=False, *args):
        Qt.QWidget.__init__(self, *args)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        if isinstance(baseDir, six.string_types):
            baseDir = DataManager.getDirHandle(baseDir, create=create)
        self.baseDir = baseDir
        self.currentFile = None
        
        self.ui.fileTree.setSortMode(sortMode)
        self.ui.fileTree.setBaseDirHandle(baseDir)

        self.ui.addBtn.clicked.connect(self.addClicked)
        self.ui.removeBtn.clicked.connect(self.removeClicked)
        self.ui.refreshBtn.clicked.connect(self.refreshClicked)
        self.ui.fileTree.itemDoubleClicked.connect(self.addClicked)

    def selectedFile(self):
        return self.ui.fileTree.selectedFile()

    def addClicked(self):
        self.add(self.selectedFile())
    
    def add(self, fileHandle):
        raise Exception("Function must be reimplemented in subclass.")
    
    def removeClicked(self):
        self.remove(self.selectedFile())
        
    def remove(self, fileHandle):
        raise Exception("Function must be reimplemented in subclass.")
    
    def refreshClicked(self):
        pass
    # self.ui.fileTree.refresh()
    
        
    def setCurrentFile(self, handle):
        if self.currentFile is not None:
            #Qt.QObject.disconnect(self.currentFile, Qt.SIGNAL('changed'), self.currentFileChanged)
            try:
                self.currentFile.sigChanged.disconnect(self.currentFileChanged)
            except TypeError:
                pass
            
        if handle is None:
            self.ui.currentLabel.setText("")
        else:
            self.ui.currentLabel.setText(handle.name(relativeTo=self.baseDir))
            handle.sigChanged.connect(self.currentFileChanged)
            
        self.currentFile = handle
            
        
    def currentFileChanged(self, handle, change, args):
        if change == 'deleted':
            self.ui.currentLabel.setText("[deleted]")
        else:
            self.ui.currentLabel.setText(self.currentFile.name(relativeTo=self.baseDir))
        self.sigCurrentFileChanged.emit(handle, change, args)