# script for acq4
# find mosaic files and write png versions of the images
# THIS SCRIPT MUST BE RUN FROM THE ACQ4 CONSOLE: 
# open up the Console under the Utilities section of the Modules pane in Acq4 Manager (main window)
# bring up the Mosaic Editor from the DataManager (NOT from the ACQ4 Manager window).
# Then:
#   import importlib
#   import plot_mosaics (that is this file)
#   (importlib.reload(plot_mosaics) if you make changes to the script here).
#   plot_mosaics.plot_mosaics(man)

# Note: if you do this multiple times, you will get multiple text items on the canvas.
# we try to get rid of this by setting them to "" at the end of the script, but it doesn't always work.
# ok in pngs, not in tiffs.

import glob
import pyqtgraph as pg
import pyqtgraph.exporters
from pathlib import Path
outdir = Path("/Users/pbmanis/Desktop/Python/Dandi/Thalamocortical_images")
def plot_mosaics(man):
    dm = man.getModule("Data Manager")
    cdir = dm.baseDir

    fs = glob.glob(cdir.name()+"/**/*.mosaic", recursive=True)
    for file in fs:
        print(file)

    me = man.getModule("MosaicEditor")
    me.mod.clear(ask=False)
    textitem = None
    for i, file in enumerate(fs):
    #	 file = "/Volumes/Pegasus_004/ManisLab_Data3/Kasten_Michael/Maness_Ank2_PFC_stim/Rig2(MRK)/L23_intrinsic/2024.06.03_000/slice_000/2024.06.03_s0.mosaic"
        # if i > 1:
        #     continue
        fh = man.dirHandle(fs[i])
        fp = Path(fs[i]).name
        outfile = Path(outdir, fp).with_suffix(".png")
        
        me.mod.clear(ask=False)
        me.mod.loadFileRequested([fh])
        canvas = me.mod.getElement("Canvas")
        # clearing the canvas scene removes too much stuff.... 
        if textitem is not None:
            textitem.setPlainText(str(fp))
        else:
            textitem = canvas.scene().addText(str(fp))
        canvas.scene().update()
        ex = pg.exporters.ImageExporter(canvas.scene())

        ex.export(str(outfile))
        print(f"Exported to {outfile!s}")
        if textitem is not None:
            textitem.setPlainText("")  # clear the text every time
