cd rcnode
sh rcnode.sh &
cd ..

export SLICE_PATH=${PWD}/interfaces
export PYTHONPATH=${PWD}/python:$PYTHONPATH
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$COPPELIASIM_ROOT
#export QT_QPA_PLATFORM_PLUGIN_PATH=$COPPELIASIM_ROOT

cd controller_V2
python3 src/controller.py etc/config

