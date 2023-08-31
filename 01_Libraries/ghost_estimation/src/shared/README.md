[![Test Status](https://github.com/ut-amrl/amrl_shared_lib/actions/workflows/buildTest.yml/badge.svg)](https://github.com/ut-amrl/amrl_shared_lib/actions)

# amrl_shared_lib
Shared library for all AMRL C++ projects

## Build
1. Install dependencies:
   ```
   sudo apt-get install libgtest-dev libgoogle-glog-dev cmake build-essential
   ```
1. Compile
   ```
   make -j`nproc`
   ```

## Usage
1. In your own project, add this as a git submodule to a subdir (`src/shared` in this example):
   ```
   git submodule add git@github.com:umass-amrl/amrl_shared_lib.git src/shared
   ```
   Alternatively, if using HTTPS:
   ```
   git submodule add https://github.com/ut-amrl/amrl_shared_lib.git src/shared
   ```
1. In the main `CMakeLists.txt` for your project, add the directory as a cmake subdir, and add the subdir as a location to search for includes (the order of these commands matters):
   ```
   INCLUDE_DIRECTORIES(src/shared)
   ADD_SUBDIRECTORY(src/shared)
   ```
1. Add `amrl_shared_lib` to the linker step for any executables (`my-program` in this example) that use the shared library:
   ```
   TARGET_LINK_LIBRARIES(my-program amrl_shared_lib)
   ```

To change the name of the shared library, set `AMRL_LIBRARY_NAME` prior to the  `INCLUDE_DIRECTORIES` call in Step 2, e.g.:

```
SET(AMRL_LIBRARY_NAME "alternate_link_name"
    CACHE STRING "Name of compiled library")
```

To generate the unit tests for the shared library, set `GENERATE_SHARED_LIB_UNITTESTS` to `ON` prior to the  `INCLUDE_DIRECTORIES` call in Step 2, e.g.:

```
SET(GENERATE_SHARED_LIB_UNITTESTS ON)
```
