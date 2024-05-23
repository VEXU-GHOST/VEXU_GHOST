#!/bin/bash

# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

cd $VEXU_HOME
git submodule update --recursive

# Get processor architecture to determine proper .deb source
$arch=$(uname -p)

# Build matplotlib-cpp
echo "--------------- MATPLOTLIB_CPP ---------------"
if [ ! -d "${VEXU_HOME}/09_External/matplotlib-cpp/build" ];
then
	cd $VEXU_HOME/09_External/matplotlib-cpp
	mkdir build && cd build

	cmake ..          || exit -1
	make              || exit -1
	sudo make install || exit -1

	cd ../..
else
	echo "Build already exists"
fi
echo
echo

# Build Mumps
echo "--------------- MUMPS ---------------"
cd $VEXU_HOME/09_External/ThirdParty-Mumps

sudo apt install gfortran-10 liblapack-dev pkg-config --install-recommends -y || exit -1
sudo apt install swig -y                                                      || exit -1

export FC=$(which gfortran-10)

./configure       || exit -1
make              || exit -1
sudo make install || exit -1

cd ..

echo
echo

# Build IPOPT
echo "--------------- IPOPT ---------------"
cd Ipopt

./configure       || exit -1
make              || exit -1
make test         || exit -1
sudo make install || exit -1

echo
echo

# Build Casadi
echo "--------------- CASADI ---------------"
# if [ ! -d "${VEXU_HOME}/09_External/casadi/build" ];
# then
# 	cd $VEXU_HOME/09_External/casadi
# 	mkdir build && cd build

# 	cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON .. || exit -1
# 	make                                                                        || exit -1
# 	sudo make install                                                           || exit -1

# 	cd ../..
# else
#         echo "Build already exists"
# fi

if [ "$arch" == 'x86_64' ]; then
    sudo wget https://github.com/VEXU-GHOST/ghost_dependencies/raw/22db13647a74e7911c249b5762ea3c6d12893aa4/deb/ghost-casadi-86.deb || exit -1
    sudo dpkg -i ~/Downloads/ghost-casadi-86.deb || exit -1

elif [ "$arch" == 'aarch64' ]; then
    echo "TODO(xander): add arm debian link here"
    echo "TODO(xander): add arm debian installation here"

else
    echo "Failure: Unexpected processure architecture."
    exit -1
fi


echo
echo

# Build Casadi Tutorial CPP
echo "--------------- CASADI_TUTORIAL_CPP ---------------"
if [ ! -d "${VEXU_HOME}/09_External/Casadi-Tutorial-CPP/build" ];
then
	cd $VEXU_HOME/09_External/Casadi-Tutorial-CPP
	mkdir build code_gen
	cd build

	cmake .. || exit -1
	make     || exit -1
    
	cd ../..
else
        echo "Build already exists"
fi

echo
echo
