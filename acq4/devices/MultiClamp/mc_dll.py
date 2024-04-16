import os

from msl.loadlib import Client64
from msl.loadlib import LoadLibrary

mcpath =os.path.join("C:\\", "Axon", "MultiClamp 700A Commander", "3rd Party Support", "AxMultiClampMsg", "AxMultiClampMsg")
# mcpath =os.path.join("C:\\", "Axon", "MultiClamp 700A Commander", "AxHWControlPanel.dll")

cpp = LoadLibrary(mcpath)
print(dir(cpp.lib))