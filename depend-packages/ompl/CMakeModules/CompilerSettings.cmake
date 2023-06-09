set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# force <boost/functional.hpp> to avoid deprecated use of
# std::unary_function and std::binary_function
add_compile_definitions(_HAS_AUTO_PTR_ETC=0)

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-W -Wall -Wextra -Wno-system-headers)
    # prepend optimizion flag (in case the default setting doesn't include one)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS_RELEASE}")
endif(CMAKE_COMPILER_IS_GNUCXX)
if(CMAKE_CXX_COMPILER_ID MATCHES "^(Apple)?Clang$")
    add_definitions(-W -Wall -Wextra -Wno-system-headers)
    # prepend optimizion flag (in case the default setting doesn't include one)
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS_RELEASE}")
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP /W1 -DNOMINMAX")
endif(MSVC)

if(CMAKE_CXX_COMPILER_ID STREQUAL "XL")
    set(IS_XLC 1)
else()
    set(IS_XLC 0)
endif()
if(IS_XLC)
    add_definitions(-qpic -q64 -qmaxmem=-1)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -q64")
    set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -q64")
endif(IS_XLC)

if(CMAKE_COMPILER_IS_GNUCXX AND NOT MINGW)
    add_definitions(-fPIC)
endif(CMAKE_COMPILER_IS_GNUCXX AND NOT MINGW)

option(OMPL_SKIP_RPATH "Don't set RPATH to the OMPL library" OFF)
if(NOT OMPL_SKIP_RPATH)
    # Set rpath, see https://gitlab.kitware.com/cmake/community/wikis/doc/cmake/RPATH-handling
    set(CMAKE_SKIP_BUILD_RPATH OFF)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH OFF)
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_FULL_LIBDIR}")
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
endif()

# no prefix needed for python modules
set(CMAKE_SHARED_MODULE_PREFIX "")
