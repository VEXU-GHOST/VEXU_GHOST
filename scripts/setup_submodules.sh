#!/bin/bash
cd ~/VEXU_GHOST
git submodule update --recursive

# Build matplotlib-cpp
cd ghost_deps/matplotlib-cpp
mkdir build && cd build
cmake ..
make
sudo make install

# Build yaml-cpp
cd ../../yaml-cpp
mkdir build && cd build
cmake ..
make
sudo make install

# Build Mumps
cd ../../ThirdParty-Mumps
sudo apt install gfortran-10 liblapack-dev pkg-config --install-recommends
sudo apt install swig
./configure
make
sudo make install

# Build IPOPT
cd ../../Ipopt
./configure
make
make test
sudo make install

# Build Casadi
cd ../../casadi
mkdir build && cd build
cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON ..
make
sudo make install

# Build Casadi Tutorial CPP
cd ../../Casadi-Tutorial-CPP
mkdir build code_gen
cd build
cmake ..
make
sudo make install
