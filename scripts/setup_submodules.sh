#!/bin/bash
cd ~/VEXU_GHOST
git submodule update --recursive

# Build matplotlib-cpp
echo "--------------- MATPLOTLIB_CPP ---------------"
if [ ! -d "${HOME}/VEXU_GHOST/ghost_deps/matplotlib-cpp/build" ];
then
	cd $HOME/VEXU_GHOST/ghost_deps/matplotlib-cpp
	mkdir build && cd build
	cmake ..
	make
	sudo make install
	cd ../..
else
	echo "Build already exists"
fi
echo
echo

# Build Mumps
echo "--------------- MUMPS ---------------"
cd $HOME/VEXU_GHOST/ghost_deps/ThirdParty-Mumps
sudo apt install gfortran-10 liblapack-dev pkg-config --install-recommends -y
sudo apt install swig -y
./configure
make
sudo make install
cd ..

echo
echo

# Build IPOPT
echo "--------------- IPOPT ---------------"
cd Ipopt
./configure
make
make test
sudo make install

echo
echo

# Build Casadi
echo "--------------- CASADI ---------------"
if [ ! -d "${HOME}/VEXU_GHOST/ghost_deps/casadi/build" ];
then
	cd $HOME/VEXU_GHOST/ghost_deps/casadi
	mkdir build && cd build
	cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON ..
	make
	sudo make install
	cd ../..
else
        echo "Build already exists"
fi

echo
echo

# Build Casadi Tutorial CPP
echo "--------------- CASADI_TUTORIAL_CPP ---------------"
if [ ! -d "${HOME}/VEXU_GHOST/ghost_deps/Casadi-Tutorial-CPP/build" ];
then
	cd $HOME/VEXU_GHOST/ghost_deps/Casadi-Tutorial-CPP
	mkdir build code_gen
	cd build
	cmake ..
	make
	cd ../..
else
        echo "Build already exists"
fi

echo
echo
