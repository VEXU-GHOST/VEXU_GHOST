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
arch=$(dpkg --print-architecture)

cd $VEXU_HOME


# Build matplotlib-cpp
echo "--------------- MATPLOTLIB_CPP ---------------"

case $arch in

    'amd64')
        sudo wget https://github.com/VEXU-GHOST/ghost_dependencies/raw/main/deb/ghost-matplotlibcpp-amd64.deb || exit -1
        sudo --preserve-env=VEXU_HOME dpkg -i ghost-matplotlibcpp-amd64.deb || exit -1
        rm ghost-matplotlibcpp-amd64.deb
        ;;
    
    'TODO(xander): figure out the jetson arch in dpkg terms')
        echo "TODO(xander): add arm debian link here"
        echo "TODO(xander): add arm debian installation here"
        echo "TODO(xander): add arm debian removal here"
        ;;

    *)
        echo "Failure: Unexpected processure architecture."
        exit -1
        ;;
esac

echo
echo

# Build Mumps
echo "--------------- MUMPS ---------------"

sudo apt install gfortran-10 liblapack-dev pkg-config --install-recommends -y || exit -1
sudo apt install swig -y                                                      || exit -1

export FC=$(which gfortran-10)

case $arch in

    'amd64')
        sudo wget https://github.com/VEXU-GHOST/ghost_dependencies/raw/main/deb/ghost-mumps-amd64.deb || exit -1
        sudo --preserve-env=VEXU_HOME dpkg -i ghost-mumps-amd64.deb || exit -1
        rm ghost-mumps-amd64.deb
        ;;
    
    'TODO(xander): figure out the jetson arch in dpkg terms')
        echo "TODO(xander): add arm debian link here"
        echo "TODO(xander): add arm debian installation here"
        echo "TODO(xander): add arm debian removal here"
        ;;

    *)
        echo "Failure: Unexpected processure architecture."
        exit -1
        ;;
esac

echo
echo

# Build IPOPT
echo "--------------- IPOPT ---------------"

case $arch in

    'amd64')
        sudo wget https://github.com/VEXU-GHOST/ghost_dependencies/raw/main/deb/ghost-ipopt-amd64.deb || exit -1
        sudo --preserve-env=VEXU_HOME dpkg -i ghost-ipopt-amd64.deb || exit -1
        rm ghost-ipopt-amd64.deb
        ;;
    
    'TODO(xander): figure out the jetson arch in dpkg terms')
        echo "TODO(xander): add arm debian link here"
        echo "TODO(xander): add arm debian installation here"
        echo "TODO(xander): add arm debian removal here"
        ;;

    *)
        echo "Failure: Unexpected processure architecture."
        exit -1
        ;;
esac

echo
echo

# Build Casadi
echo "--------------- CASADI ---------------"

case $arch in

    'amd64')
        sudo wget https://github.com/VEXU-GHOST/ghost_dependencies/raw/main/deb/ghost-casadi-amd64.deb || exit -1
        sudo --preserve-env=VEXU_HOME dpkg -i ghost-casadi-amd64.deb || exit -1
        rm ghost-casadi-amd64.deb
        ;;
    
    'TODO(xander): figure out the jetson arch in dpkg terms')
        echo "TODO(xander): add arm debian link here"
        echo "TODO(xander): add arm debian installation here"
        echo "TODO(xander): add arm debian removal here"
        ;;

    *)
        echo "Failure: Unexpected processure architecture."
        exit -1
        ;;
esac

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
