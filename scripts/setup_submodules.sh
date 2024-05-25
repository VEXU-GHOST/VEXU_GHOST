#!/bin/bash

exit_unsupported_pkg() {
    pkg=$1

    echo "Failure: Unsupported package: $pkg. Please notify maintainers of ghost_dependencies!"
    echo "https://github.com/VEXU-GHOST/ghost_dependencies"
    exit -1
}

exit_unsupported_arch() {
    arch=$1

    echo "Failure: Unsupported processure architecture: $arch. Please notify maintainers of ghost_dependencies!"
    echo "https://github.com/VEXU-GHOST/ghost_dependencies"
    exit -1
}

install_submodule() {
    pkg=$1

    supported_pkgs=(
        'casadi'
        'ipopt'
        'matplotlibcpp'
        'mumps'
        'plotjuggler'
        'plotjuggler-ros'
        'rplidar'
        'bt-cpp'
        'bt-ros'
    )

    supported_archs=(
        'amd64'
        'TODO(xander): figure out the jetson arch in dpkg terms'
    )

    # Check if given pkg is supported
    if [[ ! " ${supported_pkgs[*]} " =~ " $pkg " ]]; then
        exit_unsupported_pkg $pkg
    fi

    # Check if given arch is supported 
    if [[ ! " ${supported_archs[*]} " =~ " $arch " ]]; then
        exit_unsupported_arch $arch
    fi

    sudo wget https://github.com/VEXU-GHOST/ghost_dependencies/raw/main/deb/ghost-$pkg-$arch.deb || exit -1
    sudo --preserve-env=VEXU_HOME dpkg -i ghost-$pkg-$arch.deb || exit -1
    sudo rm ghost-$pkg-$arch.deb
}


# Verify repo path is set
if [ -z "${VEXU_HOME}" ]
then
    echo "Failure: repository path variable VEXU_HOME is unset."
    exit -1
fi

cd $VEXU_HOME
git submodule init
git submodule update --recursive

# Get processor architecture to determine proper .deb source
arch=$(dpkg --print-architecture)


# Build matplotlib-cpp
echo "--------------- MATPLOTLIB_CPP ---------------"
install_submodule matplotlibcpp
echo; echo


# Build Mumps
echo "--------------- MUMPS ---------------"

sudo apt install gfortran-10 liblapack-dev pkg-config --install-recommends -y || exit -1
sudo apt install swig -y                                                      || exit -1

export FC=$(which gfortran-10)

install_submodule mumps
echo; echo


# Build IPOPT
echo "--------------- IPOPT ---------------"
install_submodule ipopt
echo; echo

# Build Casadi
echo "--------------- CASADI ---------------"
install_submodule casadi
echo; echo

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

echo; echo

echo "------------ PLOTJUGGLER -------------"
install_submodule plotjuggler
echo; echo

echo "----------- PLOTJUGGLER-ROS ----------"
install_submodule plotjuggler-ros
echo; echo

echo "-------------- RPLIDAR ---------------"
install_submodule rplidar
echo; echo

echo "--------------- BT-CPP ---------------"
install_submodule bt-cpp
echo; echo

echo "--------------- BT-ROS ---------------"
install_submodule bt-ros
echo; echo
