# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import glob
import json
import weakref
from collections import OrderedDict
import numpy as np
from pathlib import Path
import scipy
import scipy.stats
from typing import Union

import acq4.util.debug as debug
import pyqtgraph as pg
import pyqtgraph.multiprocess as MP
import platform
import MetaArray
import multiprocessing as MPROC
from acq4.analysis.AnalysisModule import AnalysisModule
from acq4.util import Qt
import acq4.util.DataManager as DataManager
import acq4.analysis.atlas as atlas
import acq4.analysis.modules.MosaicEditor.markers as Markers
from acq4.util.Canvas.Canvas import Canvas
from acq4.util.Canvas import items
from six.moves import range


Ui_Form = Qt.importTemplate(".MosaicEditorTemplate")


class MosaicEditor(AnalysisModule):
    """
    The Mosiac Editor allows the user to bring in multiple images onto
    a canvas, and manipulate the images, including adjusting contrast,
    position, and alpha.
    Images created in Acq4 that have position information will be
    represented according to their x,y positions (but not the z).

    Groups of images can be scaled together.
    An image stack can be "flattened" with different denoising methods
    - useful for a quick reconstruction of filled cells.
    Images can be compared against an atlas for reference, if the atlas
    data is loaded.
    This tool is useful for combining images taken at different positions
    with a camera or 2P imaging system.
    The resulting images may be saved as SVG or PNG files.
    Mosaic Editor makes extensive use of pyqtgraph Canvas methods.
    """

    # Version number for save format.
    #   increment minor version number for backward-compatible changes
    #   increment major version number for backward-incompatible changes
    _saveVersion = (2, 0)

    def __init__(self, host):
        AnalysisModule.__init__(self, host)

        self.items = weakref.WeakKeyDictionary()
        self.files = weakref.WeakValueDictionary()

        self._addTypes = OrderedDict()

        self.ctrl = Qt.QWidget()
        self.ui = Ui_Form()
        self.ui.setupUi(self.ctrl)
        self.atlas = None
        self.parallel = False
        self.videosSelected = False
        self.videosShown = True
        self.canvas = Canvas(name="MosaicEditor")

        self._elements_ = OrderedDict(
            [
                (
                    "File Loader",
                    {"type": "fileInput", "size": (200, 300), "host": self},
                ),
                (
                    "Mosaic",
                    {
                        "type": "ctrl",
                        "object": self.ctrl,
                        "pos": ("right",),
                        "size": (600, 100),
                    },
                ),
                (
                    "Canvas",
                    {
                        "type": "ctrl",
                        "object": self.canvas.ui.view,
                        "pos": ("bottom", "Mosaic"),
                        "size": (800, 800),
                    },
                ),
                (
                    "ItemList",
                    {
                        "type": "ctrl",
                        "object": self.canvas.ui.canvasCtrlWidget,
                        "pos": ("right", "Canvas"),
                        "size": (200, 400),
                    },
                ),
                (
                    "ItemCtrl",
                    {
                        "type": "ctrl",
                        "object": self.canvas.ui.canvasItemCtrl,
                        "pos": ("bottom", "ItemList"),
                        "size": (200, 400),
                    },
                ),
            ]
        )

        self.initializeElements()

        self.clear(ask=False)

        self.ui.fileLoader = self.getElement("File Loader", create=True)
        self.ui.fileLoader.ui.fileTree.hide()

        try:
            self.ui.fileLoader.setBaseClicked()  # get the currently selected directory in the DataManager
        except:
            pass

        for a in atlas.listAtlases():
            self.ui.atlasCombo.addItem(a)

        # Add buttons to the canvas control panel
        self.btnBox = Qt.QWidget()
        self.btnLayout = Qt.QGridLayout()
        self.btnLayout.setContentsMargins(0, 0, 0, 0)
        self.btnBox.setLayout(self.btnLayout)
        l = self.canvas.ui.gridLayout
        l.addWidget(self.btnBox, l.rowCount(), 0, 1, l.columnCount())

        self.addCombo = Qt.QComboBox()
        self.addCombo.currentIndexChanged.connect(self._addItemChanged)
        self.btnLayout.addWidget(self.addCombo, 0, 0, 1, 2)
        self.addCombo.addItem("Add item..")

        self.saveBtn = Qt.QPushButton("Save ...")
        self.saveBtn.clicked.connect(self.saveClicked)
        self.btnLayout.addWidget(self.saveBtn, 1, 0)

        self.clearBtn = Qt.QPushButton("Clear All")
        self.clearBtn.clicked.connect(self._handleClearBtnClick)
        self.btnLayout.addWidget(self.clearBtn, 1, 1)

        self.canvas.sigItemTransformChangeFinished.connect(self.itemMoved)
        self.ui.atlasCombo.currentIndexChanged.connect(self.atlasComboChanged)

        # Group Operations:
        self.ui.normalizeBtn.clicked.connect(self.normalizeImages)
        self.ui.blendBtn.clicked.connect(self.blendImages)
        self.ui.MaxImageProjectionBtn.clicked.connect(self.MIP_Images)
        self.ui.MaxImageProjectionGaussianBtn.clicked.connect(self.MIP_Images)
        self.ui.MaxImageProjectionMedianBtn.clicked.connect(self.MIP_Images)
        self.ui.checkSelectedBtn.clicked.connect(self.checkSelected)
        self.ui.uncheckSelectedBtn.clicked.connect(self.uncheckSelected)
        self.ui.globalParallel_checkBox.clicked.connect(self.setParallel)
 
        # Tile Operation:
        self.ui.tileShadingBtn.clicked.connect(self.tileShadeImages)
        self.ui.mosaicApplyScaleBtn.clicked.connect(self.updateScaling)
        self.ui.mosaicResetScaleBtn.clicked.connect(self.resetScaling)
        self.ui.mosaicFlipLRBtn.clicked.connect(self.flipLR)
        self.ui.mosaicFlipUDBtn.clicked.connect(self.flipUD)
        
        # Annotation Tools:
        self.ui.mosaicCreateMarkers.clicked.connect(self.createMarkers)
        self.ui.mosaicSelectVideos.clicked.connect(self.selectAllVideos)
        self.ui.mosaicShowHide.clicked.connect(self.showAllVideos)
        self.ui.getSpotImage.clicked.connect(self.get_laser_spots)

        self.imageMax = 0.0

        self.registerItemType(items.getItemType("GridCanvasItem"))
        self.registerItemType(items.getItemType("RulerCanvasItem"))
        self.registerItemType(items.getItemType("MarkersCanvasItem"))
        self.registerItemType(items.getItemType("CellCanvasItem"))
        self.registerItemType(items.getItemType("AtlasCanvasItem"))

    def _handleClearBtnClick(self):
        self.clear(ask=True)

    def registerItemType(self, itemclass, menuString=None):
        """Add an item type to the list of addable items."""
        if menuString is None:
            menuString = itemclass.typeName()
        if itemclass.__name__ not in items.itemTypes():
            items.registerItemType(itemclass)
        self._addTypes[menuString] = itemclass.__name__
        self.addCombo.addItem(menuString)

    def _addItemChanged(self, index):
        # User requested to create and add a new item
        if index <= 0:
            return
        itemtype = self._addTypes[self.addCombo.currentText()]
        self.addCombo.setCurrentIndex(0)
        self.addItem(type=itemtype)

    def atlasComboChanged(self, ind):
        if ind == 0:
            self.closeAtlas()
            return
        name = self.ui.atlasCombo.currentText()
        self.loadAtlas(name)

    def closeAtlas(self):
        if self.atlas is not None:
            self.atlas.close()
            self.atlas = None
        while True:
            ch = self.ui.atlasLayout.takeAt(0)
            if ch is None:
                break
            ch = ch.widget()
            ch.hide()
            ch.setParent(None)

    def loadAtlas(self, name):
        name = str(name)
        self.closeAtlas()

        cls = atlas.getAtlasClass(name)
        obj = cls()
        ctrl = obj.ctrlWidget(host=self)
        self.ui.atlasLayout.addWidget(ctrl, 0, 0)
        self.atlas = ctrl

    def loadFileRequested(self, files):
        if files is None:
            return

        for f in files:
            if f.shortName().endswith(".mosaic"):
                self.loadStateFile(f.name())
                continue
            if f.shortName().startswith("Map_"):
                spotimage = self.get_laser_spots(mapdir = f)


            if f in self.files:  ## Do not allow loading the same file more than once
                item = self.files[f]
                item.show()  # just show the file; but do not load it
                continue

            if f.isFile():  # add specified files
                item = self.addFile(f)
            elif f.isDir():  # Directories are more complicated
                if self.dataModel is None:
                    print("No data model set")
                    continue
                if (
                    self.dataModel.dirType(f) == "Cell"
                ):  #  If it is a cell, just add the cell "Marker" to the plot
                    item = self.canvas.addFile(f)
                else:  # in all other directory types, look for MetaArray files
                    filesindir = glob.glob(f.name() + "/*.ma")
                    for (
                        fd
                    ) in (
                        filesindir
                    ):  # add files in the directory (ma files: e.g., images, videos)
                        try:
                            fdh = DataManager.getFileHandle(
                                fd
                            )  # open file to get handle.
                        except IOError:
                            continue  # just skip file
                        item = self.addFile(fdh)
                    if len(filesindir) == 0:  # add protocol sequences
                        item = self.addFile(f)
        self.canvas.autoRange()

    def addFile(self, f, name=None, inheritTransform=True):
        """Load a file and add it to the canvas.

        The new item will inherit the user transform from the previous item
        (chronologically) if it does not already have a user transform specified.
        """
        if f.isFile():
            fp = Path(f.name())
            if fp.suffix in [".ma", ".tif"]:
                name = str(Path(fp.parent.name, fp.name)) # give a name that includes the parent directory
            return self.addOneFile(f, name=name, inheritTransform=inheritTransform)
        elif f.isDir():
            allfiles = f.ls()                                      # get all the tif files in the directory
            for fi in allfiles:
                fh = DataManager.getDirHandle(Path(f.name(), fi))
                if fh.ext() == ".tif":
                    name = str(Path(fh.parent().shortName(), fh.shortName()))
                    print("name: ", name)
                    self.addOneFile(fh, name=name, inheritTransform=inheritTransform)
        else:
            raise ValueError("Cannot load file (not file or dir?) ", f)
    
    def addOneFile(self, f, name=None, inheritTransform=True):
        item = self.canvas.addFile(f, name=name)

        self.canvas.selectItem(item)

        if isinstance(item, list):
            item = item[0]

        self.items[item] = f
        self.files[f] = item
        try:
            item.timestamp = f.info()["__timestamp__"]
        except:
            item.timestamp = None
        ## load or guess user transform for this item
        if (
            inheritTransform
            and not item.hasUserTransform()
            and item.timestamp is not None
        ):
            ## Record the timestamp for this file, see what is the most recent transformation to copy
            best = None
            for i2 in self.items:
                if i2 is item:
                    continue
                if i2.timestamp is None:
                    continue
                if i2.timestamp < item.timestamp:
                    if best is None or i2.timestamp > best.timestamp:
                        best = i2

            if best is not None:
                trans = best.saveTransform()
                item.restoreTransform(trans)

        return item

    def addItem(self, item=None, type=None, **kwds):
        """Add an item to the MosaicEditor canvas.

        May provide either *item* which is a CanvasItem or QGraphicsItem instance, or
        *type* which is a string specifying the type of item to create and add.
        """
        if isinstance(item, Qt.QGraphicsItem):
            print("Loading qgraphics item: ", item)
            return self.canvas.addGraphicsItem(item, **kwds)
        else:
            # print("type: ", type)
            if type == "CellCanvasItem":
                fh = self.ui.fileLoader.selectedFiles()
                if len(fh) == 1:
                    fh = fh[0]
                    if fh.shortName().startswith("cell"):
                        name = fh.shortName()
                        kwds['name'] = name
                elif len(fh) > 0:
                    pname = fh[0].parent().shortName()
                    if pname.startswith("cell"):
                        name = pname
                        kwds['name'] = name
                else:
                    name = "Cell"
                    kwds['name'] = name
           # elif type == ""
            item = self.canvas.addItem(item, type, **kwds)
            self.canvas.selectItem(item)
            return item

    def checkSelected(self):
        w = self.canvas.ui.canvasCtrlWidget.children()
        tw = None
        for c in w:  # look for the tree widget
            if isinstance(c, pg.widgets.TreeWidget.TreeWidget):
                tw = c
        if tw is None:  # hmm. not there.
            return
        allItems = tw.listAllItems()
        for item in allItems:
            if item.isSelected():  # selected by name
                item.setCheckState(0, Qt.QtCore.Qt.CheckState.Checked)

    
    def uncheckSelected(self):
        w = self.canvas.ui.canvasCtrlWidget.children()
        tw = None
        for c in w:  # look for the tree widget
            if isinstance(c, pg.widgets.TreeWidget.TreeWidget):
                tw = c
        if tw is None:  # hmm. not there.
            return
        allItems = tw.listAllItems()
        for item in allItems:
            if item.isSelected():  # selected by name
                item.setCheckState(0, Qt.QtCore.Qt.CheckState.Unchecked)
   

    def get_laser_spots(self, mapdir:Union[Path, str]):
        """get_laser_spots from the selected map directory camera images,
        and compare to the spot locations in the scanner file
        Generates a maximal image projection of the camera images
        taken during the mapping experiment, and retuns that image
        """
        imagecount = 0

        mappoints = list(Path(mapdir.name()).glob("*"))
        mappoints = [mp for mp in mappoints if mp.is_dir()]
        print("map points: ", mappoints)
        useframe = 1
        for imagecount, mp in enumerate(mappoints):
            cameraframe = Path(mp, 'Camera', 'frames.ma')
            print("reading: ", cameraframe)
            frame = MetaArray.MetaArray(file=str(cameraframe),  # read the camera frame
                                        readAll=True,  # read all data into memory
                                        verbose=False)
            frame_data = frame.view(np.ndarray)
            if imagecount == 0:
                frame_data_max = frame_data[useframe,:,:]
                frame_bkgd = np.zeros_like(frame_data[useframe,:,:])
            else:
                if useframe == 0:
                    frame_data_max += frame_data[useframe,:,:]
                    frame_bkgd = np.zeros_like(frame_data[useframe,:,:])
                else:
                    frame_data_max = np.maximum(frame_data_max, frame_data[useframe,:,:])
                    frame_bkgd += frame_data[0,:,:]

        frame_bkgd = frame_bkgd/int(imagecount)
        if useframe == 0:
            frames = frame_data_max/int(imagecount)
        else:
            frames = frame_data_max - frame_bkgd
        # print(np.max(frames), np.min(frames))
        # frames = frames  > np.min(frames)*1.5
        info = frame.infoCopy()
        spotimage = MetaArray.MetaArray(frames, info=info[1:])  # remove the time axis.
        # fout = Path(str(mapdir)+'_spotimage.ma')
        # print('info: ', info)
        # spotimage.write(str(fout))
        # exit()
        return spotimage

    def createMarkers(self):
        """createMarkers Instantiate a standard set of markers:
        including the Cell, surface, AN, and slice markers.
        """
    
        # get the type of marker items to create
        markerType = self.ui.MosaicMarkersCombo.currentText()
        if markerType not in Markers.definedMarkers.keys():
            raise ValueError("Marker type not defined: ", markerType)
        
        markerItem = self.addItem(type="MarkersCanvasItem", name=markerType)
        markerItem.params.setName(markerType)

        # don't put all the markers in the same place - logical offsets (although,
        # this might result in markers that assumed a particular orientation
        markerdict = Markers.definedMarkers[markerType]
        for i, marker in enumerate(markerdict.keys()):
            markerItem.addMarker(marker)  # adds marker centered on view
            thismarker = markerItem.params.child(marker)
            pos = thismarker.target.param().target.pos()
            print("MarkerDict: ", markerdict[marker])
            thismarker.target.param().target.setPos(
                pos.x() + markerdict[marker][0], pos.y() + markerdict[marker][1]
            )
        

    def selectAllVideos(self):
        """select or deselect all of the videos in the canvas.
        We do this through the tree widget in the ItemList.
        """

        w = self.canvas.ui.canvasCtrlWidget.children()
        tw = None
        for c in w:  # look for the tree widget
            if isinstance(c, pg.widgets.TreeWidget.TreeWidget):
                tw = c
        if tw is None:  # hmm. not there.
            return
        allItems = tw.listAllItems()
        for item in allItems:
            if item.name.startswith("video_"):  # selected by name
                if self.videosSelected is False:
                    item.setSelected(True)
                else:
                    item.setSelected(False)
        self.videosSelected = not self.videosSelected

    def showAllVideos(self):
        """showAllVideos Show or hide all of the videos in the canvas.
        Here we work directly with the items on the canvas, rather than through
        the tree widget.
        """
        for item in self.canvas.items:
            if not hasattr(item, "data"):
                continue
            if item.data.ndim == 3:
                if self.videosShown is False:
                    item.setVisible(True)
                else:
                    item.setVisible(False)
        self.videosShown = not self.videosShown

    def setParallel(self):
        if self.ui.globalParallel_checkBox.isChecked():
            print("set parallel")
            self.parallel = True
        else:
            print("unset parallel")
            self.parallel = False

    def MIP_Images(self):
        if self.parallel:
            if platform.system() == "Darwin":
                raise NotImplementedError(
                    "Parallel processing is not implemented for this function on Mac OS"
                )
            print("running parallel")
            nWorkers = MPROC.cpu_count()
            TASKS = [
                item
                for j, item in enumerate(self.canvas.selectedItems())
                if hasattr(item, "data") and item.data.ndim == 3
            ]
            tresults = [None] * len(TASKS)
            msg = f"Processing {len(TASKS):d} videos"
            with MP.Parallelize(
                enumerate(TASKS), results=tresults, workers=nWorkers, progressDialog=msg
            ) as tasker:
                for j, item in tasker:
                    item.filter.filterBtnClicked(True)

        else:
            # Non parallelized version:
            print("Not running parallel")
            with pg.ProgressDialog(
                "Processing..", 0, len(self.canvas.selectedItems())
            ) as dlg:
                for i, currentItem in enumerate(self.canvas.selectedItems()):
                    if not hasattr(currentItem, "data"):
                        continue
                    if currentItem.data.ndim == 3 and currentItem.name.startswith("video_"):
                        print("   operating on : ", currentItem.name)
                        currentItem.filter.filterBtnClicked(True)
                    else:
                        print("   skipping: ", currentItem.name)
                    dlg.setValue(i)  ## could also use dlg += 1
                    if dlg.wasCanceled():
                        raise Exception("Processing canceled by user")

    def blendImages(self):
        raise NotImplementedError("blendImages not implemented yet")

    def _rescale_newimage(self, d, blimage, m, hm):
        if d.shape != blimage.shape:
            print(
                "rescale newimage: data shape and blimage shape do not match: ",
                d.shape,
                blimage.shape,
            )
            return None
        # flatten the field using the blimage average illumination pattern
        newImage = d / blimage  # (d - imin)/(blimg - imin) # rescale image.
        hn = np.histogram(newImage, bins=hm[1])  # use bins from global image
        n = np.argmax(hn[0])
        newImage = (hm[1][m] / hn[1][n]) * newImage  # rescale to the global max.
        return newImage

    def tileShadeImages(self):
        """
        Apply corrections to the images and rescale the data.
        The goal is to correct for uneven illumination and to rescale the images
        This does the following:
        1. compute mean image over entire selected group
            If the group includes videos, then the max projection of each video is taken
            to compute the value for that image
        2. smooth the mean image heavily.
        3. rescale the images and correct for field flatness from the average image
        4. apply the scale.
        Use the min/max mosaic button to readjust the display scale after this
        automatic operation if the scaling is not to your liking.
        """
        print("rescaling images")
        nsel = len(self.canvas.selectedItems())
        if nsel == 0:
            print("no selected items")
            return
        nhistbins = 100
        # generate a histogram of the global levels in the image (all images and all frames in videos selected)
        all_images: list = []
        for i in range(nsel):
            currentItem = self.canvas.selectedItems()[i]
            if currentItem.data.ndim == 2:
                all_images.append(currentItem.data)
            elif currentItem.data.ndim == 3:
                all_images.append(currentItem.data.max(axis=0))
                # for x in currentItem.data:
                #     all_images.append(x)
        hm = np.histogram(np.dstack(all_images), nhistbins)
        n = 0
        self.imageMax = 0.0
        for i in range(nsel):
            currentItem = self.canvas.selectedItems()[i]
            item_size = currentItem.data.shape
            item_dim = currentItem.data.ndim
            if item_dim == 3:
                currentImage = np.max(currentItem.data, axis=0)
            else:
                currentImage = currentItem.data
            if i == 0:
                nxm = currentImage.shape
                meanImage = np.zeros((nxm[0], nxm[1]))

            try:
                meanImage = meanImage + np.array(currentImage)
                imagemax = np.amax(meanImage)
                if imagemax > self.imageMax:
                    self.imageMax = imagemax
                n = n + 1
            except:
                print("image i = %d failed" % i)
                print("file name: ", self.canvas.selectedItems()[i].name)
                print("expected shape of nxm: ", nxm)
                print(
                    " but got data shape: ", self.canvas.selectedItems()[i].data.shape
                )
        meanImage = meanImage / n  # np.mean(meanImage[0:n], axis=0)
        filtwidth = np.floor(nxm[0] / 10 + 1)
        blimg = scipy.ndimage.filters.gaussian_filter(
            meanImage, filtwidth, order=0, mode="reflect"
        )
        m = np.argmax(hm[0])  # returns the index of the max count

        # now rescale each image/stack individually
        # rescaling is done against the global histogram, in an attempt to keep the gain constant.
        self.imageMax = 0
        for i in range(nsel):
            d = np.array(self.canvas.selectedItems()[i].data)
            #            hmd = np.histogram(d, 512) # return (count, bins)
            xh = (
                d.shape
            )  # capture shape just in case it is not right (have data that is NOT !!)
            if d.ndim == 3:
                for j in range(xh[0]):
                    newImage = self._rescale_newimage(d[j], blimg, m, hm)
                    if newImage is None:
                        continue
                    self.canvas.selectedItems()[i].data[j] = newImage
            else:
                newImage = self._rescale_newimage(d, blimg, m, hm)
                if newImage is None:
                    continue
                self.canvas.selectedItems()[i].data = newImage
            imagemax = np.max(newImage)
            if imagemax > self.imageMax:
                self.imageMax = imagemax
            self.canvas.selectedItems()[i].graphicsItem().updateImage(newImage)
            # thisimage = self.canvas.selectedItems()[i].graphicsItem()

        # self.imageMax = 0.0
        # for i in range(nsel):
        #     d = np.array(self.canvas.selectedItems()[i].data)
        #     imagemax = np.amax(d)
        #     if imagemax > self.imageMax:
        #         self.imageMax = imagemax
        for i in range(nsel):
            thisimage = self.canvas.selectedItems()[i].graphicsItem()
            thisimage.setLevels([0, self.imageMax])
        print("rescale done")

    def normalizeImages(self):
        """Normalize the images to the min/max of the selected
        group of images in the canvas.
        """
        print("normalizeImages")
        min_image = 1e6
        max_image = -1.0
        nsel = len(self.canvas.selectedItems())
        if nsel == 0:
            return
        for item in self.canvas.selectedItems():
            # print(dir(item))
            thisimage = item.graphicsItem()
            # d = item.data.min()
            # d = thisimage.getHistogram()
            if np.min(item.data) < min_image:
                min_image = np.min(item.data)
            if np.max(item.data) > max_image:
                max_image = np.max(item.data)
            print("min/max = ", min_image, max_image)
        for item in self.canvas.selectedItems():
            thisimage = item.graphicsItem()
            thisimage.setLevels([min_image, max_image])

        self.canvas.autoRange()

    def resetScaling(self):
        """
        Set all the selected images to have the original scaling (just min/max)
        """
        print("resetScaling")
        nsel = len(self.canvas.selectedItems())
        if nsel == 0:
            return
        for i in range(nsel):
            thisimage = self.canvas.selectedItems()[i].graphicsItem()
            minval = np.min(self.canvas.selectedItems()[i].data)
            maxval = np.max(self.canvas.selectedItems()[i].data)
            thisimage.setLevels(
                [minval, maxval]
            )

    def updateScaling(self):
        """
        Set all the selected images to have the scaling in the editor bar (absolute values)
        """
        print("updateScaling")
        nsel = len(self.canvas.selectedItems())
        if nsel == 0:
            return
        for i in range(nsel):
            thisimage = self.canvas.selectedItems()[i].graphicsItem()
            thisimage.setLevels(
                [self.ui.mosaicDisplayMin.value(), self.ui.mosaicDisplayMax.value()]
            )

    def flipUD(self):
        """
        flip each image array up/down, in place. Do not change position.
        Note: arrays are rotated, so use lr to do ud, etc.
        """
        nsel = len(self.canvas.selectedItems())
        if nsel == 0:
            return
        for i in range(nsel):
            self.canvas.selectedItems()[i].data = np.fliplr(
                self.canvas.selectedItems()[i].data
            )
            self.canvas.selectedItems()[i].graphicsItem().updateImage(
                self.canvas.selectedItems()[i].data
            )
        # print dir(self.canvas.selectedItems()[i])

    def flipLR(self):
        """
        Flip each image array left/right, in place. Do not change position.
        """
        nsel = len(self.canvas.selectedItems())
        if nsel == 0:
            return
        for i in range(nsel):
            self.canvas.selectedItems()[i].data = np.flipud(
                self.canvas.selectedItems()[i].data
            )
            self.canvas.selectedItems()[i].graphicsItem().updateImage(
                self.canvas.selectedItems()[i].data
            )

    def itemMoved(self, canvas, item):
        """Save an item's transformation if the user has moved it.
        This is saved in the 'userTransform' attribute; the original position data is not affected.
        """
        fh = self.items.get(item, None)
        if not hasattr(fh, "setInfo"):
            fh = None

        try:
            item.storeUserTransform(fh)
        except Exception as ex:
            if (
                len(ex.args) > 1 and ex.args[1] == 1
            ):  ## this means the item has no file handle to store position
                return
            raise

    def getLoadedFiles(self):
        """Return a list of all file handles that have been loaded"""
        return list(self.items.values())

    def clear(self, ask=True):
        """Remove all loaded data and reset to the default state.

        If ask is True (and there are items loaded), then the user is prompted
        before clearing. If the user declines, then this method returns False.
        """
        if ask and len(self.items) > 0:
            response = Qt.QtWidgets.QMessageBox.question(
                self.clearBtn,
                "Warning",
                "Really clear all items?",
                Qt.QtWidgets.QMessageBox.StandardButton.Ok
                | Qt.QtWidgets.QMessageBox.StandardButton.Cancel,
            )
            if response != Qt.QtWidgets.QMessageBox.StandardButton.Ok:
                return False

        self.canvas.clear()
        self.items.clear()
        self.files.clear()
        self.videosSelected = False  # reset
        self.videosShown = True
        self.lastSaveFile = None
        return True

    def saveState(self, relativeTo=None):
        """Return a serializable representation of the current state of the MosaicEditor.

        This includes the list of all items, their current visibility and
        parameters, and the view configuration.
        """
        items = list(self.canvas.items)
        items.sort(key=lambda i: i.zValue())

        return OrderedDict(
            [
                ("contents", "MosaicEditor_save"),
                ("version", self._saveVersion),
                ("rootPath", relativeTo.name() if relativeTo is not None else ""),
                ("items", [item.saveState(relativeTo=relativeTo) for item in items]),
                ("view", self.canvas.view.getState()),
            ]
        )

    def saveStateFile(self, filename):
        dh = DataManager.getDirHandle(os.path.dirname(filename))
        state = self.saveState(relativeTo=dh)
        json.dump(state, open(filename, "w"), indent=4, cls=Encoder)

    def restoreState(self, state, rootPath=None):
        if state.get("contents", None) != "MosaicEditor_save":
            raise TypeError("This does not appear to be MosaicEditor save data.")
        if state["version"][0] > self._saveVersion[0]:
            raise TypeError(
                "Save data has version %d.%d, but this MosaicEditor only supports up to version %d.x."
                % (state["version"][0], state["version"][1], self._saveVersion[0])
            )

        if not self.clear():
            return

        root = state["rootPath"]
        if root == "":
            # data was stored with no root path; filenames should be absolute
            root = None
        else:
            # data was stored with no root path; filenames should be relative to the loaded file
            root = DataManager.getHandle(rootPath)

        loadfail = []
        for itemState in state["items"]:
            fname = itemState.get("filename")
            if fname is None:
                # create item from scratch and restore state
                itemtype = itemState.get("type")
                if itemtype not in items.itemTypes():
                    # warn the user later on that we could not load this item
                    loadfail.append(
                        (itemState.get("name"), 'Unknown item type "%s"' % itemtype)
                    )
                    continue
                item = self.addItem(type=itemtype, name=itemState["name"])
            else:
                # create item by loading file and restore state
                if root is None:
                    fh = DataManager.getHandle(fh)
                else:
                    fh = root[fname]
                item = self.addFile(fh, name=itemState["name"], inheritTransform=False)
            item.restoreState(itemState)

        self.canvas.view.setState(state["view"])
        if len(loadfail) > 0:
            msg = "\n".join(["%s: %s" % m for m in loadfail])
            raise Exception("Failed to load some items:\n%s" % msg)

    def loadStateFile(self, filename):
        state = json.load(open(filename, "r"))
        self.restoreState(state, rootPath=os.path.dirname(filename))

    def saveClicked(self):
        base = self.ui.fileLoader.baseDir()
        if self.lastSaveFile is None:
            path = base.name()
        else:
            path = self.lastSaveFile

        filename = Qt.QFileDialog.getSaveFileName(
            None, "Save mosaic file", path, "Mosaic files (*.mosaic)"
        )[0]
        if filename == "":
            return
        if not filename.endswith(".mosaic"):
            filename += ".mosaic"
        self.lastSaveFile = filename

        self.saveStateFile(filename)

    def quit(self):
        self.files = None
        self.items = None
        self.canvas.clear()


class Encoder(json.JSONEncoder):
    """Used to clean up state for JSON export.
    turn numpy types into python types
    """

    def default(self, o):
        if isinstance(o, np.integer):
            return int(o)
        if isinstance(o, (np.float32, np.float64)):
            return float(o)

        return json.JSONEncoder.default(o)
