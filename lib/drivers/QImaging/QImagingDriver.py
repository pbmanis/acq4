import time
from ctypes import *
import sys, os
d = os.path.dirname(__file__)
sys.path.append(os.path.join(d, '../../util'))
from clibrary import *
from numpy import empty, uint16, ascontiguousarray, concatenate, newaxis
from pyqtgraph import graphicsWindows as gw
from PyQt4 import QtGui
from Mutex import Mutex, MutexLocker
from advancedTypes import OrderedDict
import atexit
p = CParser('QCamApi.h', cache='QCamApi.h.cache', macros={'_WIN32': '', '__int64': ('long long')})
if sys.platform == 'darwin':
    dll = cdll.LoadLibrary('/Library/Frameworks/QCam.framework/QCam')
else:
    dll = windll.QCamDriver
lib = CLibrary(dll, p, prefix = 'QCam_')        #makes it so that functions in the header file can be accessed using lib.nameoffunction, ie: QCam_LoadDriver is lib.LoadDriver
                                                #also interprets all the typedefs for you....very handy
                                                #anything from the header needs to be accessed through lib.yourFunctionOrParamete
    
###functions that are called from CameraDevice:
# setUpCamera(self) - """Prepare the camera at least so that get/setParams will function correctly"""
# listParams(self, params=None)
# setParams(self, params, autoRestart=True, autoCorrect=True)
# getParams(self, params=None)

externalParams = ['triggerMode',
                  'exposure',
                  'binningX',
                  'binningY',
                  'regionX',
                  'regionY',
                  'regionW',
                  'regionH',
                  'qprmNormalizedGain',
                  'qprmS32NormalizedGaindB',
                  'qprmS32AbsoluteOffset',
                  'qprmReadoutSpeed',
                  'qprmCoolerActive',
                  'qprmS32RegulatedCoolingTemp'
                  'qprmBlackoutMode',
                  'qprmImageFormat',
                  'qprmSyncb',
                  ]


class QCamFunctionError(Exception):
    def __init__(self, value, message):
        self.value = value
        self.message = message
    def __str__(self):
        return repr(self.message)
        
class _QCamDriverClass:
    def __init__(self):
        self.cams = {}
        self.paramTable = OrderedDict()
        
        self.loadDriver()
        
        
        global externalParams
        for p in externalParams:
            self.paramTable[p] = p        
    
    def call(self, function, *args):
        a = function(*args)
        if a() != 0:
            for x in lib('enums', 'QCam_Err'):
                if lib('enums', 'QCam_Err')[x] == a():
                    raise QCamFunctionError(a(), "There was an error running a QCam function. Error code = %s" %(x))
        else:
            return a

    def loadDriver(self):
        self.call(lib.LoadDriver)
 
    def releaseDriver(self):
        self.call(lib.ReleaseDriver)
        
    def listCameras(self):
        number = c_ulong(10)
        L = lib.CamListItem * 10
        l = L()
        self.call(lib.ListCameras, l, number)
        cams = []
        for x in list(l)[:number.value]:
            cams.append(x.cameraId)
        return cams
    
    def getCamera(self, cam):
        if not self.cams.has_key(cam):
            self.cams[cam] = _QCameraClass(cam, self)
        return self.cams[cam]
        
    def __del__(self):
        self.quit()

    def quit(self):
        for c in self.cams:
            self.cams[c].quit()
        self.releaseDriver()
        
class _QCameraClass:
    def __init__(self, name, driver):
        self.name = name
        self.driver = driver
        self.isOpen = False
        self.handle = self.open()
        self.paramAttrs = OrderedDict()
        self.cameraInfo = {}
        self.getCameraInfo()
        self.frames = []
        self.arrays = []
        self.i = 0
        self.stopSignal = True
        self.mutex = Mutex(Mutex.Recursive)
        self.lastFrame = (0,0)
        self.fnp1 = lib.AsyncCallback(self.callBack1)
        
        ## Some parameters can be accessed as groups
        self.groupParams = {
            'binning': ['binningX', 'binningY'],
            'region': ['regionX', 'regionY', 'regionW', 'regionH'],
            'sensorSize': [self.cameraInfo['qinfCcdHeight'], self.cameraInfo['qinfCcdWidth']] 
        }

        self.paramEnums = {
            'qprmImageFormat': 'QCam_ImageFormat',
            #'qprmPostProcessImageFormat': 'QCam_ImageFormat',
            'qprmSyncb': 'QCam_qcSyncb',
            'qprmReadoutSpeed': 'QCam_qcReadoutSpeed',
            #'qprmColorWheel': 'QCam_qcWheelColor',
            'triggerMode': 'QCam_qcTriggerType'
        }
        
        self.userToCameraDict = {
            'triggerMode':'qprmTriggerType',
            'exposure':'qprm64Exposure',
            'binningX':'qprmBinning',
            'binningY':'qprmBinning',
            'regionX':'qprmRoiX',
            'regionY':'qprmRoiY',
            'regionW':'qprmRoiWidth',
            'regionH':'qprmRoiHeight'
        }
        
        self.cameraToUserDict = {
            'qprmTriggerType':'triggerMode',
            'qprm64Exposure':'exposure',
            'qprmBinning':'binningX',
            'qprmRoiX':'regionX',
            'qprmRoiY':'regionY',
            'qprmRoiWidth':'regionW',
            'qprmRoiHeight':'regionH'
        }
            
       
        
    def call(self, function, *args):
        a = function(*args)
        if a() != 0:
            for x in lib('enums', 'QCam_Err'):
                if lib('enums', 'QCam_Err')[x] == a():
                    raise QCamFunctionError(a(), "There was an error running a QCam function. Error code = %s" %(x))
        else:
            return a

    def open(self): #opens the camera and returns the handle
        """Opens the chosen camera and returns the handle. Takes cameraID parameter."""
        if not self.isOpen: 
            a = self.call(lib.OpenCamera, self.name, lib.Handle())
            self.isOpen = True
            #self.call(lib.SetStreaming, a[1], 1)
            return a[1]   
  
    def __del__(self):
        self.quit()
    
    def quit(self):
        self.call(lib.Abort, self.handle)
        self.call(lib.SetStreaming, self.handle, 0)
        self.call(lib.CloseCamera, self.handle)
        self.isOpen = False
        
    def translateToCamera(self, arg):
        return self.userToCameraDict.get(arg, arg)
            
    def translateToUser(self, arg):
        return self.cameraToUserDict.get(arg, arg)
        
    def readSettings(self):
        s = self.call(lib.ReadSettingsFromCam, self.handle)[1]
        s.size = sizeof(s)
        return s

    def mkFrame(self):
        s = call(lib.GetInfo, handle, lib.qinfImageSize)[2]
        f = lib.Frame()
        frame = ascontiguousarray(empty(s/2, dtype=uint16))
        f.pBuffer = frame.ctypes.data
        f.bufferSize = s
        return (f, frame)
    
    def grabFrame(self):
        s = lib.GetInfo(handle, lib.qinfImageSize)[2]
        (f, frame) = mkFrame()
        call(lib.GrabFrame, handle, byref(f))
        w = call(lib.GetInfo, handle, lib.qinfCcdWidth)[2]
        frame.shape = (s/w, w)
        return frame

    def listParams(self, allParams=False):
        """Fills in the 'paramAttrs' dictionary with the state parameters available on the camera.
        The key is the name of the parameter, while the value is a tuple: (acceptablevalues, isWritable, isReadable, [dependencies]."""
        s = self.readSettings()
        if allParams:
            p = (lib('enums', 'QCam_Param'), lib('enums', 'QCam_ParamS32'), lib('enums', 'QCam_Param64'))
        else:
            p = (externalParams, externalParams, externalParams)
        #for x in lib('enums', 'QCam_Param'):
        for x in p[0]:
            x = self.translateToCamera(x)
            try:
                if self.call(lib.GetParam, byref(s), getattr(lib, x))() == 0:
                    try: ###first try to get a SparseTable
                        table = (c_ulong *32)()
                        r = self.call(lib.GetParamSparseTable, byref(s), getattr(lib,x), table, c_long(32))
                        self.paramAttrs[self.translateToUser(x)] = (list(r[2])[:r[3]])
                    except QCamFunctionError, err: ###if sparse table doesn't work try getting a RangeTable
                        if err.value == 1:  
                            min = self.call(lib.GetParamMin, byref(s), getattr(lib,x))[2]
                            max = self.call(lib.GetParamMax, byref(s), getattr(lib,x))[2]
                            self.paramAttrs[self.translateToUser(x)] = (min, max)
                        else: raise      
            except QCamFunctionError, err:
                if err.value == 1: pass    
                else: raise
        #for x in lib('enums', 'QCam_ParamS32'):
        for x in p[1]:
            x = self.translateToCamera(x)
            try:
                if self.call(lib.GetParamS32, byref(s), getattr(lib, x))() == 0:
                    try:
                        table = (c_long *32)()
                        r = self.call(lib.GetParamSparseTableS32, byref(s), getattr(lib,x), table, c_long(32))
                        self.paramAttrs[self.translateToUser(x)] = (list(r[2])[:r[3]])
                    except QCamFunctionError, err:
                        if err.value == 1:
                            min = self.call(lib.GetParamS32Min, byref(s), getattr(lib,x))[2]
                            max = self.call(lib.GetParamS32Max, byref(s), getattr(lib,x))[2]
                            self.paramAttrs[self.translateToUser(x)] = (min, max)
                        else: raise
            except QCamFunctionError, err:
                if err.value == 1: pass
                else: raise
        #for x in lib('enums', 'QCam_Param64'):
        for x in p[2]:
            x = self.translateToCamera(x)
            try:
                if self.call(lib.GetParam64, byref(s), getattr(lib, x))() == 0:
                    try:
                        table = (c_ulonglong *32)()
                        r = self.call(lib.GetParamSparseTable64, byref(s), getattr(lib,x), table, c_long(32))
                        self.paramAttrs[self.translateToUser(x)] = (list(r[2])[:r[3]])
                    except QCamFunctionError, err:
                        if err.value == 1:
                            min = self.call(lib.GetParam64Min, byref(s), getattr(lib,x))[2]
                            max = self.call(lib.GetParam64Max, byref(s), getattr(lib,x))[2]
                            self.paramAttrs[self.translateToUser(x)] = (min, max)
                        else: raise
            except QCamFunctionError, err:
                if err.value == 1:  pass
                else: raise
        #self.paramAttrs.pop('qprmExposure')
        #self.paramAttrs.pop('qprmOffset')
        ### Replace qcam enum numbers with qcam strings
        for x in self.paramAttrs: 
            if type(self.paramAttrs[x]) == type([]):
                if x in self.paramEnums: ## x is the name of the parameter
                    #print "Param: ", x, self.paramAttrs[x]
                    for i in range(len(self.paramAttrs[x])): ## i is the index
                        a = self.paramAttrs[x][i] ## a is the value
                        for b in lib('enums', self.paramEnums[x]): # b is the number code for the parameter option
                            if lib('enums', self.paramEnums[x])[b] == a:
                                self.paramAttrs[x][i] = b
                                #print "old: ", a, "new: ", self.paramAttrs[x][i]
        ##### For camera on rig1, listParams returns: {                       
        ##    'qprmBlackoutMode': [0L, 1L],
        ##    'qprmBinning': [1L, 2L, 4L, 8L],
        ##          'qprmExposureRed': (10L, 1073741823L),
        ##    'qprm64Exposure': (10000L, 1073741823000L),
        ##          'qprmPostProcessBayerAlgorithm': [1L, 2L, 3L, 4L],
        ##    'qprmImageFormat': ['qfmtRaw8', 'qfmtRaw16', 'qfmtMono8', 'qfmtMono16', 'qfmtRgbPlane8', 'qfmtRgbPlane16', 'qfmtBgr24', 'qfmtXrgb32', 'qfmtRgb48', 'qfmtBgrx32', 'qfmtRgb24'],
        ##    'qprmCoolerActive': [0L, 1L], # 0=disable cooler
        ##    'qprmRoiHeight': (1L, 1200L),
        ##          'qprmColorWheel': ['qcWheelRed', 'qcWheelGreen', 'qcWheelBlue'], #color of the RGB filter
        ##          'qprm64ExposureRed': (10000L,1073741823000L),
        ##    'qprmRoiX': (0L, 1599L),
        ##    'qprmRoiY': (0L, 1199L),
        ##    'qprmS32NormalizedGaindB': (-6930000, 26660000),
        ##          'qprmPostProcessGainBlue': (10L, 100000L),
        ##    'qprmTriggerType': ['qcTriggerNone', 'qcTriggerEdgeHi', 'qcTriggerEdgeLow', 'qcTriggerPulseHi', 'qcTriggerPulseLow', 'qcTriggerSoftware', 'qcTriggerStrobeHi', 'qcTriggerStrobeLow'],
        ##          'qprmPostProcessImageFormat': ['qfmtRaw8', 'qfmtRaw16','qfmtMono8', 'qfmtMono16', 'qfmtRgbPlane8', 'qfmtRgbPlane16', 'qfmtBgr24', 'qfmtXrgb32', 'qfmtRgb48', 'qfmtBgrx32', 'qfmtRgb24'],
        ##          'qprm64ExposureBlue': (10000L, 1073741823000L),
        ##          'qprmExposureBlue': (10L, 1073741823L),
        ##    'qprmSyncb': ['qcSyncbOem1', 'qcSyncbExpose'],
        ##    'qprmRoiWidth': (1L, 1600L),
        ##          'qprmPostProcessGainRed': (10L, 100000L),
        ##          'qprmGain': (0L, 4095L), #DEPRECATED
        ##    'qprmS32RegulatedCoolingTemp': (-45, 0),
        ##    'qprmS32AbsoluteOffset': (-2048, 2047),
        ##    'qprmNormalizedGain': (451000L, 21500000L),#default is 1
        ##          'qprmPostProcessGainGreen': (10L, 100000L),
        ##    'qprmReadoutSpeed': ['qcReadout20M', 'qcReadout10M', 'qcReadout5M']}
        

        
                                
    def getCameraInfo(self):
        for x in lib('enums', 'QCam_Info'):
            try:
                a = self.call(lib.GetInfo, self.handle, getattr(lib, x))
                self.cameraInfo[x] = a[2]
            except QCamFunctionError, err:
                if err.value == 1:
                    #print "No info for: ", x
                    pass
                else: raise

    def getParam(self, param):
        if param in self.groupParams:
            return self.getParams(self.groupParams[param], asList=True)
        s = self.readSettings()
        #s.size = sizeof(s)
        param = self.translateToCamera(param)
        if param in lib('enums', 'QCam_Param'):
            value = self.call(lib.GetParam, byref(s), getattr(lib, param))[2]
        elif param in lib('enums', 'QCam_ParamS32'):
            value = self.call(lib.SetParamS32, byref(s), getattr(lib, param))[2]
        elif param in lib('enums', 'QCam_Param64'):
            value = self.call(lib.SetParam64, byref(s), getattr(lib, param))[2]
        return value

    def setParam(self, param, value):
        s = self.readSettings()
        param = self.translateToCamera(param)
        if param in lib('enums', 'QCam_Param'):
            self.call(lib.SetParam, byref(s), getattr(lib, param), c_ulong(value))
        elif param in lib('enums', 'QCam_ParamS32'):
            self.call(lib.SetParamS32, byref(s), getattr(lib, param), c_long(value))
        elif param in lib('enums', 'QCam_Param64'):
            self.call(lib.SetParam64, byref(s), getattr(lib, param), c_ulonglong(value))
        with self.mutex:
            if self.stopSignal == True:
                #self.mutex.unlock()
                self.call(lib.SendSettingsToCam, self.handle, byref(s))
            if self.stopSignal == False:
                #self.mutex.unlock()
                self.call(lib.QueueSettings, self.handle, byref(s), null, lib.qcCallbackDone)

    #def getParams(self, *params):
    #    s = self.readSettings()
    #    dict = {}
    #    for param in params:
    #        param = self.translateToCamera(param)
    #        if param in lib('enums', 'QCam_Param'):
    #            value = self.call(lib.GetParam, byref(s), getattr(lib, param))[2]
    #        elif param in lib('enums', 'QCam_ParamS32'):
    #            value = self.call(lib.GetParamS32, byref(s), getattr(lib, param))[2]
    #        elif param in lib('enums', 'QCam_Param64'):
    #            value = self.call(lib.GetParam64, byref(s), getattr(lib, param))[2]
    #        param = self.translateToUser(param)
    #        dict[param]=value
    #    return dict
    
    def getParams(self, params, asList=False):
        """Get a list of parameter values. Return a dictionary of name: value pairs"""
        if asList:
            return [self.getParam(p) for p in params]
        else:
            return OrderedDict([(p, self.getParam(p)) for p in params])

    def setParams(self, **params): ### need to make this work with group params
        s = self.readSettings()
        for param in params:
            param = self.translateToCamera(param)
            if param in lib('enums', 'QCam_Param'):
                self.call(lib.SetParam, byref(s), getattr(lib, param), c_ulong(params[param]))
            elif param in lib('enums', 'QCam_ParamS32'):
                self.call(lib.SetParamS32, byref(s), getattr(lib, param), c_long(params[param]))
            elif param in lib('enums', 'QCam_Param64'):
                self.call(lib.SetParam64, byref(s), getattr(lib, param), c_ulonglong(params[param]))
        with self.mutex:
            if self.stopSignal == True:
                #self.mutex.unlock()
                self.call(lib.SendSettingsToCam, self.handle, byref(s))
            if self.stopSignal == False:
                #self.mutex.unlock()
                self.call(lib.QueueSettings, self.handle, byref(s), 0, lib.qcCallbackDone)

    def start(self):
        #global i, stopsignal
        #self.mutex.lock()
        #self.stopSignal = False
        #self.i = 0
        #self.mutex.unlock()
        with self.mutex:
            self.stopSignal = False
            self.i=0
        self.call(lib.SetStreaming, self.handle, 1)
        for x in range(10):
            f, a = self.mkFrame()
            self.frames.append(f)
            self.arrays.append(a)
        for x in range(2):
            self.call(lib.QueueFrame, self.handle, self.frames[self.i], self.fnp1, lib.qcCallbackDone, 0, self.i)
            self.mutex.lock()
            self.i += 1
            self.mutex.unlock()
    
    def callBack1(self, *args):
        #global i, lastframe
        if args[2] != 0:
            for x in lib('enums', 'QCam_Err'):
                if lib('enums', 'QCam_Err')[x] == args[2]:
                    raise QCamFunctionError(args[2], "There was an error during QueueFrame/Callback. Error code = %s" %(x))
        #self.mutex.lock()
        with self.mutex:
            self.lastFrame = (args[1], arrays[args[1]]) ### Need to figure out a way to attach settings info (binning, exposure gain region and offset) to frame!!!
            if self.i != 9:
                self.i += 1
            else:
                self.i = 0
            if self.stopSignal == False:
                #self.mutex.unlock()
                self.call(lib.QueueFrame, self.handle, self.frames[self.i], self.fnp1, lib.qcCallbackDone, 0, self.i)
            #else:
            #    self.mutex.unlock()

    def stop(self):
        #global stopsignal
        #self.mutex.lock()
        with self.mutex:
            self.stopSignal = True
            self.call(lib.Abort, self.handle)
        #self.mutex.unlock()

    def getLastFrame(self):
        #global lastframe
        with self.mutex:
            a = self.lastFrame
            #self.mutex.unlock()
            return a 
    
    
#loadDriver()
#cameras = listCameras()
#handle = openCamera(cameras[0])



#setParam(lib.qprmDoPostProcessing, 0)
#setParams(qprm64Exposure=10000000)
##setParams(qprmExposureRed=0, qprmExposureBlue=0)
#setParams(qprmReadoutSpeed=lib.qcReadout20M)
#
#setParams(qprmTriggerType=lib.qcTriggerFreerun, qprmImageFormat=lib.qfmtMono16)
#
#
#
#getCameraInfo()
#print camerainfo
#
#
#
#n = 0
#
#
#
#
#for i in range(5):
#    f, a = mkFrame()
#    frames.append(f)
#    arrays.append(a)
#    print "Queue frame..", id(f)
#    
#    print "Frame queued."
#   # time.sleep(0.3)

#time.sleep(1.0)
#print "starting app.."
#app = QtGui.QApplication([])
#print "app started."
#print a.shape, (camerainfo['qinfCcdWidth'], camerainfo['qinfCcdHeight'])
#a.shape = (camerainfo['qinfCcdHeight'], camerainfo['qinfCcdWidth'])
#print "create window"
#imw1 = gw.ImageWindow()
#imw1.setImage(a.transpose())
##imw1.setImage(concatenate([a.transpose()[newaxis] for a in arrays]))
#print "show window"
#imw1.show()
#
#print "Done."
#
#app.exec_()
