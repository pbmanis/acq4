"""
Fix the objective scale factor
Give "stated objective", "actual objective"

('objective', '4x 0.1na ACHROPLAN')

"""

import os
import sys

from dataclasses import dataclass
from typing import Union
import numpy as np
from acq4.modules.Module import Module
from acq4.util import Qt
import acq4.util.InterfaceCombo  # just to register 'interface' parameter type
from acq4.util.DataManager import getDirHandle

from pathlib import Path
from pyqtgraph import configfile
import pprint
import datetime
import pyqtgraph as pg
import pyqtgraph.dockarea as PGD

from pyqtgraph.parametertree import Parameter, ParameterTree

pp = pprint.PrettyPrinter(indent=4)

CineScale = 1.0
refscale = [(1.0 / CineScale) * 6.54e-6, -(1.0 / CineScale) * 6.54e-6]

objectiveDict = {
    "4x 0.1na ACHROPLAN": 4.0,
    "10x 0.3na W N-ACHROPLAN": 10.0,
    "20x 0.5na W N-ACHROPLAN": 20.0,
    "40x 0.8na ACHROPLAN": 40.0,
    "63x 0.9na ACHROPLAN": 63.0,
}


@dataclass
class Changer:
    change_type: str = "new"
    filename: str = ""
    from_objective: str = "4x 0.1na ACHROPLAN"
    to_objective: str = "10x 0.3na W N-ACHROPLAN"
    videos: Union[str, None] = None
    images: Union[str, None] = None


class SequenceParser:
    def __init__(self):
        pass

    def seqparse(self, sequence: str, mode: str = "nd"):
        """parse the list of the format:
        12;23/10 etc... like nxtrec in datac
        now also parses matlab functions and array formats, using eval

        first arg is starting number for output array
        second arg is final number
        / indicates the skip arg type
        basic: /n means skip n : e.g., 1;10/2 = 1,3,5,7,9
        special: /##:r means randomize order (/##rn means use seed n for randomization)
        special: /##:l means spacing of elements is logarithmic
        special: /##:s means spacing is logarithmic, and order is randomized. (/##sn means use seed n for randomization)
        special: /:a## means alternate with a number
        multiple sequences are returned in a list... just like single sequences...

        3 ways for list to be structured:
        1. standard datac record parses. List is enclosed inbetween single quotes
        2. matlab : (array) operator expressions. [0:10:100], for example
        3. matlab functions (not enclosed in quotes). Each function generates a new list
        note that matlab functions and matrices are treated identically

        Updated 9/07/2000, 11/13/2000, 4/7/2004 (arbitrary matlab function argument with '=')
        converted to python 3/2/2009
        Paul B. Manis, Ph.D.
        pmanis@med.unc.edu
        """

        seq = []
        target = []
        sequence.replace(" ", "")  # remove all spaces - nice to read, not needed to calculate
        (seq2, sep, remain) = sequence.partition("&")  # find  and return nested sequences
        # print("seq: ", seq2, 'sep: ', sep, 'remain: ', remain)
        while len(seq2) != 0:
            try:
                (oneseq, onetarget) = self.recparse(seq2)
                seq.append(oneseq)
                target.append(onetarget)
            except:
                raise ValueError("recparse failed on : ", seq2)
            seq2, sep, remain = remain.partition("&")  # find  and return nested sequences
        if mode == "sequential":
            seq = [[x for y in seq for x in y], None]
        return seq, target

    def recparse(self, cmdstr: str):
        """function to parse basic word unit of the list - a;b/c or the like
        syntax is:
        [target:]a;b[/c][*n]
        where:
        target is a parameter target identification (if present)
        the target can be anything - a step, a duration, a level....
        it just needs to be in a form that will be interepreted by the PyStim
        sequencer.
        a, b and c are numbers
        n, if present *n implies a "mode"
        such as linear, log, randomized, etc.
        """

        recs = []
        target = []
        seed = 0
        skip = 1.0
        (target, sep, rest) = cmdstr.partition(":")  # get the target
        if rest == "":
            rest = target  # no : found, so no target designated.
            target = ""
        rest = rest.replace(" ", ",")
        if "," in rest:
            recs = eval(f"[{rest:s}]")  # evaluate as a list
            return recs, target
        (sfn, sep, rest1) = rest.partition(";")
        (sln, sep, rest2) = rest1.partition("/")
        (sskip, sep, mo) = rest2.partition("*")  # look for mode

        fn = float(sfn)
        ln = float(sln)
        if sskip != "":
            skip = float(sskip)
        else:
            skip = 1.0
        ln = ln + 0.01 * skip
        if mo == "":  # linear spacing; skip is size of step
            recs = np.arange(fn, ln, skip)

        if mo.find("l") >= 0:  # log spacing; skip is length of result
            recs = np.logspace(np.log10(fn), np.log10(ln), skip)

        if mo.find("t") >= 0:  # just repeat the first value
            recs = [fn]

        if mo.find("n") >= 0:  # use the number of steps, not the step size
            if skip == 1.0:
                sk = ln - fn
            else:
                sk = (ln - fn) / (skip - 1.0)
            recs = np.arange(fn, ln, sk)

        if mo.find("r") >= 0:  # randomize the result
            if recs == []:
                recs = np.arange(fn, ln, skip)
            recs = sample(recs, len(recs))

        if mo.find("a") >= 0:  # alternation - also test for a value after that
            (arg, sep, value) = mo.partition("a")  # is there anything after the letter?
            if value == "":
                value = 0.0
            else:
                value = float(value)
            c = [value] * len(recs) * 2  # double the length of the sequence
            c[0 : len(c) : 2] = recs  # fill the alternate positions with the sequence
            recs = c  # copy back
        return (recs, target)


class FixObjective(Module):
    """
    Config
    ----------

    enableMockPatch : bool
        Whether or not to allow mock patching.

    """

    moduleDisplayName = "FixObjective"
    moduleCategory = "Utilities"

    def __init__(self, manager, name, config):
        Module.__init__(self, manager, name, config)
        self.manager = manager
        self.win = FixObjectiveWindow(self)
        self.win.show()

    def quit(self):
        return Module.quit(self)


class FixObjectiveWindow(Qt.QWidget):

    def __init__(self, module):

        Qt.QWidget.__init__(self)
        self.module = module
        self.manager = module.manager
        self.name = module.name

        self.write = False
        try:
            self.datadir = self.manager.getCurrentDir()
        except:  # if not running in acq4
            self.datadir = Path(".")
        self.objdata = Changer()
        self.SQP = SequenceParser()
        self.set_window()

    def getProtocolDir(self, reload_last=False):
        try:
            current_dir = self.manager.getCurrentDir()
        except Exception:
            current_dir = self.manager.getBaseDir()
        print(current_dir)
        if current_dir is None:
            current_dir = "."
        else:
            current_dir = current_dir.name()
        current_directory = pg.Qt.QtWidgets.QFileDialog.getExistingDirectory(
            self, "Open Directory with Images", current_dir
        )
        if current_directory == "":
            return

        self.objdata.filename = current_directory
        self.read_index(objective=self.objdata, write=False)
        self.show_index()

    def rewrite_index(self, index: dict, index_file: Union[str, Path]):
        configfile.writeConfigFile(index, index_file)

    def read_indexes(self, changeList: list = [], write: bool = False):
        """
        To change a list of images/videos
        """
        print("write: ", write)
        for objective in changeList:
            self.read_index(objective, write=write)

    def read_index(self, objective: object = None, write: bool = False):
        print("\nfix_objscale: We will be using the following reference scale: ", refscale)
        print("   This scale may be specific to your camera!!!!!")
        print("read_index write flag is: ", write)
        print("Objective: ")
        print(f"    Change Type: {objective.change_type!s}")
        print(f"    File: {objective.filename!s}")
        print(f"    From: {objective.from_objective!s}X to {objective.to_objective!s}X")
        print(f"    For images: {str(objective.images)!s}")
        print(f"    For videos: {str(objective.videos)!s}")
        print("=" * 40)
        self.index_file = Path(objective.filename, ".index")
        self.index = configfile.readConfigFile(self.index_file)

    def show_index(self):
        print("self.index: ")
        # print(self.index)
        self.textbox.setCurrentFont(Qt.QtGui.QFont("Courier New"))
        text = []  # ["<font color='red' size='3' font-family='monospace'>Hello PyQt5!\nHello"]
        # ["<font-family:'Courier New' color='blue' size='10'>"]
        for k in self.index.keys():
            print("index is: ", k)
            if k.startswith("image"):
                text.append(
                    f" {k:16s}: {self.index[k]['objective']:24s}, {str(self.index[k]['binning']):8s}"
                    + f" {str(self.index[k]['transform']['scale']):s}<br>"
                )
            if k.startswith("video"):
                text.append(
                    f" {k:16s}: {self.index[k]['objective']:24s}, {str(self.index[k]['binning']):8s}"
                    + f" {str(self.index[k]['transform']['scale']):s}<br>"
                )

        self.textbox.setHtml("\n".join([t for t in text]))

    def set_new_objective(self, data):
        self.objdata.to_objective = data

    def update_from_objective(self):
        fns = self.get_imagefilenames(self.objdata)  # get for the current
        print("fns: ", fns)
        # print('self.index: ', self.index)
        for fn in fns:
            objname = self.index[fn]["objective"]
            if objname not in list(objectiveDict.keys()):
                Qt.QtWidgets.QErrorMessageDialog(
                    f"Objective {objname:s} not found in list of known objectives."
                )
                continue
            print("objname: ", objname)
            self.ptreedata.param("Original Objective").setValue(objname)

    def get_original_objective(self):
        pass

    def view_proposed_changes(self, write=False):
        print("Proposed changes - new objective data: ", self.objdata)
        self.change_scale(self.objdata, write=write)

    def get_imagefilenames(self, objective: object) -> str:
        imagefiles = []
        print("objective img: ", objective.images)
        if objective.images[0] is not None:
            images = self.SQP(objective.images)[0]
            print("images: ", images)
            for img in images:
                print("img: ", img)
                imagefiles.append(f"image_{int(img[0]):03d}.tif")
        elif objective.images[1] is not None:
            videos = self.SQP(objective.images)[1]
            for vid in videos:
                imagefiles.append(f"img_{int(vid):03d}.tif")
        else:
            pass
        return imagefiles

    def change_scale(self, objective: object = None, write: bool = False):
        imagefiles = self.get_imagefilenames(objective)
        if len(imagefiles) == 0:
            return

        print("\n----------------------------")
        print(imagefiles)
        print(self.index.keys())
        for imagefile in imagefiles:
            if imagefile not in self.index.keys():
                QtWidgets.QErrorMessageDialog(
                    "File {imagefile:s} not found in {str(list(self.index.keys())):s}"
                )
                continue
            print("Index imagefile: ", imagefile)
            print(
                "Old objective: ", self.index[imagefile]["objective"]
            )  # pp.pprint(index[imagefile] )
            old_objective = self.index[imagefile]["objective"]
            pp.pprint("   Old transform: ")
            pp.pprint(self.index[imagefile]["transform"])
            pp.pprint("   Old device transform,: ")
            pp.pprint(self.index[imagefile]["deviceTransform"])
            binning = self.index[imagefile]["binning"]
            new_objective = objective.to_objective  # string name
            magnification_new_objective = float(objectiveDict[new_objective])  # new magnification
            self.index[imagefile]["transform"]["scale"] = (
                binning[0] * refscale[0] / magnification_new_objective,
                binning[1] * refscale[1] / magnification_new_objective,
                1.0,
            )
            self.index[imagefile]["deviceTransform"]["scale"] = (
                binning[0] * refscale[0] / magnification_new_objective,
                binning[1] * refscale[1] / magnification_new_objective,
                1.0,
            )
            d = datetime.datetime.now()
            dstr = d.strftime("%Y-%m-%d %H:%M:%S")
            self.index[imagefile]["objective"] = objective.to_objective
            self.index[imagefile][
                "note"
            ] = f"Objective scale corrected from {str(objectiveDict[old_objective]):s}"
            self.index[imagefile][
                "note"
            ] += f" to {str(objectiveDict[new_objective]):s} on {dstr:s} by PBM"
            print(
                "New objective: ", self.index[imagefile]["objective"]
            )  # pp.pprint(index[imagfile] )
            print("   New transform: ")
            pp.pprint(self.index[imagefile]["transform"])
            print("   New device transform: ")
            pp.pprint(self.index[imagefile]["deviceTransform"])
            print("   Added Note: ", self.index[imagefile]["note"])
            print("----------------------------")

            print("read_index write: ", write)
            index_filename = Path(objective.filename, ".index")
            if write:
                self.rewrite_index(self.index, index_filename)
                print(".index file has been updated")

            else:
                print("Dry Run: .index file was NOT modified")
                print(f" file to modify is: {str(index_filename):s}")
        # then update index file display
        self.show_index()

    def build_ptree(self):
        self.params = [
            # {"name": "Pick Cell", "type": "list", "values": cellvalues, "value": cellvalues[0]},
            {"name": "Set Directory/Protocol", "type": "action"},
            {"name": "Reload Last Protocol", "type": "action"},
            {"name": "Images", "type": "str", "value": ""},
            {"name": "Videos", "type": "str", "value": ""},
            {
                "name": "Original Objective",
                "type": "list",
                "values": list(objectiveDict.keys()),
                "value": self.objdata.from_objective,
                "renamable": False,
            },
            {
                "name": "New Objective",
                "type": "list",
                "values": list(objectiveDict.keys()),
                "value": self.objdata.to_objective,
                "renamable": False,
            },
            {"name": "View .index", "type": "action"},
            {"name": "View proposed changes", "type": "action"},
            {"name": "Apply changes", "type": "action"},
            {"name": "Quit", "type": "action"},
        ]
        self.ptree = ParameterTree()
        self.ptreedata = Parameter.create(name="Commands", type="group", children=self.params)

        self.ptree.setParameters(self.ptreedata)
        self.ptree.setMaximumWidth(self.ptreewid)
        self.ptree.setMinimumWidth(self.ptreewid - 100)

    def command_dispatcher(self, param, changes):
        """
        Dispatcher for the commands from parametertree
        path[0] will be the command name
        path[1] will be the parameter (if there is one)
        path[2] will have the subcommand, if there is one
        data will be the field data (if there is any)
        """
        for param, change, data in changes:
            path = self.ptreedata.childPath(param)

            if path[0] == "Quit":
                self.quit()
            elif path[0] == "View .index":
                self.show_index()
            elif path[0] == "Set Directory/Protocol":
                self.getProtocolDir()
            elif path[0] == "Videos":
                self.objdata.videos = data
            elif path[0] == "Images":
                self.objdata.images = data
                print(self.objdata.images)
            elif path[0] == "Original Objective":
                self.objdata.from_objective = data
            elif path[0] == "New Objective":
                self.objdata.to_objective = data
                self.update_from_objective()
            elif path[0] == "View proposed changes":
                self.view_proposed_changes(write=False)
            elif path[0] == "Apply changes":
                self.view_proposed_changes(write=True)

    def set_window(self):
        self.layout = Qt.QtWidgets.QGridLayout()
        self.setLayout(self.layout)
        self.layout.setSpacing(0)
        self.layout.setContentsMargins(0,0,0,0)

        self.splitter = Qt.QtWidgets.QSplitter()
        self.layout.addWidget(self.splitter, 0, 0)
        self.setWindowTitle("FixObjective")

        self.DockArea = PGD.DockArea()
        self.layout.addWidget(self.DockArea)
        win_wid = 768
        win_ht = 400
        self.resize(win_wid, win_ht)
        self.fullscreen_widget = None

        self.ptreewid = 250
        self.build_ptree()
        # Initial Dock Arrangment

        self.Dock_Params = PGD.Dock("Params", size=(self.ptreewid, win_ht))
        self.Dock_Params.addWidget(self.ptree)
        self.Dock_Report = PGD.Dock("Reporting", size=(win_wid - self.ptreewid, win_ht))

        self.textbox = Qt.QtWidgets.QTextEdit()
        self.textbox.setReadOnly(True)
        self.textbox.setText("(.index file)")
        self.Dock_Report.addWidget(self.textbox)

        self.DockArea.addDock(self.Dock_Params, "left")
        self.DockArea.addDock(self.Dock_Report, "right", self.Dock_Params)
        self.ptreedata.sigTreeStateChanged.connect(self.command_dispatcher)

    def quit(self):
        Module.quit(self)
