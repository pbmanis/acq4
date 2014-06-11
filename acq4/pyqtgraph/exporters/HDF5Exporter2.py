import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
from .Exporter import Exporter
from pyqtgraph.parametertree import Parameter
import numpy 
import h5py

__all__ = ['HDF5Exporter2']
    
    
class HDF5Exporter2(Exporter):
    Name = "HDF5 Export from plot data, single time base"
    windows = []
    allowCopy = False
    def __init__(self, item):
        Exporter.__init__(self, item)
        self.params = Parameter(name='params', type='group', children=[
            {'name': 'Name', 'type': 'str', 'value': 'Export',},
        ])
        
    def parameters(self):
        return self.params
    
    def export(self, fileName=None):
        
        if not isinstance(self.item, pg.PlotItem):
            raise Exception("Must have a PlotItem selected for HDF5 export.")
        
        if fileName is None:
            self.fileSaveDialog(filter=["*.h5", "*.hdf", "*.hd5"])
            return
        dsname = self.params['Name']
        fd = h5py.File(fileName, 'a') # forces append to file... 'w' doesn't seem to "delete/overwrite"
        data = []
        tb = False
        for c in self.item.curves:
            d = c.getData()
            if tb is False:
                data.append(d[0]) # do once.
                tb = True
            data.append(d[1])
        fdata = numpy.array(data).astype('double')
        dset = fd.create_dataset(dsname, data=fdata)
        fd.close()
