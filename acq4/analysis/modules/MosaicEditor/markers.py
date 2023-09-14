"""
Define markers for mosaic editor. 
Markers are in named groups with a sequence of locations that should be set by the user.
The locations are ordered (first element of tuple) for computing the area and distances.
The second and third elements of the tuple are the x and y coordinates of the marker, in meters.
If the first element is None, then the marker stands alone and is not used for computing area or distances.

"""

definedMarkers = {
    "DCN Transstrial":
        {
            "surface": (0, 100e-6, 0),
            "rostralsurface": (1, 75e-6, 175e-6),
            "rostralborder": (2, -150e-6, 250e-6),
            "medialborder": (3, -150e-6, 0),
            "caudalborder": (4, -150e-6, -200e-6),
            "caudalsurface": (5, 75e-6, -175e-6),
            "AN": (None, -80e-6, 50e-6),
        },
    "DCN Parasagittal": {
            "caudalsurface": (0, 300e-6, -200e-6),
            "dorsalsurface": (1, 0e-6, 300e-6),
            "rostralsurface": (2, -300e-6, 100e-6),
            "rostralborder": (3, -350e-6, 0),
            "medialborder": (4, -150e-6, -100e-6),
            "caudalborder": (5, 150e-6, -250e-6),
            "AN": (None, 0e-6, -150e-6),
    },
    "DCN Coronal": {
            "dorsal": (0, -100e-6, 500e-6),
            "medial": (1, -100e-6, 0e-6),
            "ventral": (2, -100e-6, -500e-6),
            "lateral1": (3, -50e-6, -400e-6),
            "lateral2": (4, 250e-6, -200e-6),
            "lateral3": (5, 350e-6, 50e-6),
            "lateral4": (6, 250e-6, 350e-6),
            },

    "VCN Horizontal": {
            "surface": (0, 100e-6, 0),
            "rostralsurface": (1, 75e-6, 175e-6),
            "caudalsurface": (2, 75e-6, -175e-6),
            "medialborder": (3, -150e-6, 0),
            "caudalborder": (4, -150e-6, -200e-6),
            "rostralborder": (5, -150e-6, 250e-6),
    },
    "VCN Parasagittal": {
            "rostralsurface": (0, -300e-6, 0e-6),
            "dorsalborder": (1, -100e-6, 300e-6),
            "medialborder": (2, 0e-6, 0e-6),
            "caudalborder": (3, 150e-6, -200e-6),
            "caudalAN": (4, 0e-6, -300e-6),
            "rostralAN": (5, -350e-6, 0),
            "AN": (None, 0e-6, -150e-6),
            "VNR": (None, -100e-6, -200e-6), 
    },
    "VCN Coronal": {
            "dorsomedial": (0, -100e-6, 500e-6),
            "medial": (1, -100e-6, 0e-6),
            "ventromedial": (2, -100e-6, -500e-6),
            "lateral1": (3, -50e-6, -400e-6),
            "lateral2": (4, 250e-6, -200e-6),
            "lateral3": (5, 350e-6, 50e-6),
            "lateral4": (6, 250e-6, 350e-6),
    },
    "Cortex Horizontal": {
            "surface": (0, 1000e-6, 0),
            "medialrostral": (1, -150e-6, 0),
            "lateralrostral": (2, 150e-6, 0),
            "medialcaudal": (3, -150e-6, -200e-6),
            "lateralcaudal": (4, -150e-6, 200e-6),
            "injectionsite": (None, 0, 200e-6),
            "hpcanteriorpole": (None, 0, -200e-6),
    },
    "Cortex Coronal": {
            "surface": (0, 1000e-6, 0),
            "medialdorsal": (1, -150e-6, 0),
            "lateraldorsal": (2, 150e-6, 0),
            "injectionsite": (None, -150e-6, -200e-6),
            "medialventral": (3, -150e-6, 200e-6),
            "lateralventral": (4, 0, 200e-6),
    },

}
