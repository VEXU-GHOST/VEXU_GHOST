#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

.SILENT:

all: build build/CMakeLists.txt.copy
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

# Sets the build type to Debug.
set_debug:
	$(eval build_type=Debug)

# Ensures that the build type is debug before running all target.
debug_all: | set_debug all

clean:
	rm -rf bin lib build

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) \
		$(TESTFLAGS) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

test: TESTFLAGS += -DGENERATE_SHARED_LIB_UNITTESTS=ON
test: all
	./build/amrl_shared_lib_tests

build:
	mkdir -p build

