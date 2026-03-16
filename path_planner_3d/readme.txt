
conda create -n qt5_env python=3.12
conda activate qt5_env

conda config --add channels conda-forge
conda config --set channel_priority strict

conda install pyqt pyqtgraph
conda install pyqt-tools


conda activate qt5_env
conda install pyopengl -c conda-forge
















