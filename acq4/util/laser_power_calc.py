"""Compute parameters for laser spots, assuming Gaussian shape.
Definitions:
67% of power is in the 1/e diameter (returned by acq4)
87% is at 1/e^2
95% is at ?  

"""
import numpy as np
import pint 
UR =  pint.registry.UnitRegistry()


def gaussian(x):
    """compute 

    Args:
        x (_type_): _description_
    """
def area(radius:float):
    # simple area from radius
    return np.pi*radius*radius

def mwpermm2_87(pwr:float, acq4diam:float):
    """Compute power in mW/mm2
        from acq4 diameter at 87% of area
    """
    e2 = np.sqrt(2)*acq4diam
    spot_area = area(e2/2.0)

    mwpmm2 = pwr/spot_area
    return mwpmm2


def mwpermm2(pwr:float, acq4diam:float, height:float=1/np.e):
    """Compute power in mW/mm2
        from acq4 diameter at selected height
    """
    e2 = np.sqrt(-np.log(height))*acq4diam
    spot_area = area(e2/2.0)
    mwpmm2 = pwr/spot_area
    return mwpmm2
    

def main():
    diam = 45*UR.um
    pwr = 2*UR.mW
    ht = 0.01*UR.dimensionless
    print(f" Inten: {mwpermm2(pwr, diam, height=ht).to(UR.mW/(UR.mm*UR.mm)):.3f} mW/mm2 for ht: {ht:.3f}")

if __name__ == '__main__':
    main()
