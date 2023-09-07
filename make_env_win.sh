set -e # force failure if anyting fails in script - ensuring completion
set -o errexit
ENVNAME="acq4_venv"
if [ -d $ENVNAME ]
then
    echo "Removing previous environment: $ENVNAME"
    # set +e
    # rsync -aR --remove-source-files $ENVNAME ~/.Trash/ || exit 1
    # set -e
    rm -Rf $ENVNAME
else
    echo "No previous environment - ok to proceed"
fi

#python3.11 -m venv $ENVNAME || exit 1
py -m venv $ENVNAME || exit 1
source $ENVNAME/Scripts/activate || exit 1
py -m pip install --upgrade pip # be sure pip is up to date in the new env.
pip3 install wheel  # seems to be missing (note singular)
pip3 install cython
pip3 install requests

# now get the dependencies
pip3 install -r requirements.txt || exit 1
source $ENVNAME/Scripts/activate

# build the mechanisms
# this may equire a separate install of the standard NEURON package
# with the same version as we have provided
# nrnivmodl cnmodel/mechanisms
py --version
py tools/rebuildUI.py acq4
#python tools/rebuildUI.py pyqt6 -d acq4/pyqtgraph -v -f
py setup.py develop || exit 1
source $ENVNAME/Scripts/activate
echo "Success in installing acq4 environment!"
