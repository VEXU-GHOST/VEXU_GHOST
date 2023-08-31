#!/bin/bash
cd ~/VEXU_GHOST
git submodule update --recursive

# Build matplotlib-cpp
echo "--------------- MATPLOTLIB_CPP ---------------"
if [ ! -d "${HOME}/VEXU_GHOST/09_External/matplotlib-cpp/build" ];
then
	cd $HOME/VEXU_GHOST/09_External/matplotlib-cpp
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
cd $HOME/VEXU_GHOST/09_External/ThirdParty-Mumps
sudo apt install gfortran-10 liblapack-dev pkg-config --install-recommends -y
sudo apt install swig -y
export FC=$(which gfortran-10)
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
if [ ! -d "${HOME}/VEXU_GHOST/09_External/casadi/build" ];
then
	cd $HOME/VEXU_GHOST/09_External/casadi
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
if [ ! -d "${HOME}/VEXU_GHOST/09_External/Casadi-Tutorial-CPP/build" ];
then
	cd $HOME/VEXU_GHOST/09_External/Casadi-Tutorial-CPP
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
