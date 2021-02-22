# Copyright: (C) 2021, SIRRL Social and Intelligent Robotics Research Laboratory, University of Waterloo
# Authors: Austin Kothig <austin.kothig@uwaterloo.ca>
# CopyPolicy: Released under the terms of the BSD 3-Clause License.

cmake_minimum_required( VERSION 3.12 )

# Main project
project( HriPhysio LANGUAGES CXX C VERSION 0.0.1 )
set( PROJECT_DESCRIPTION "" )

############################################################

#
# Library configuration from:
#   https://github.com/robotology/how-to-export-cpp-library/blob/master/CMakeLists.txt
#

# Defines the CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR and many other useful macros.
# See https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html

include( GNUInstallDirs )


# Control where libraries and executables are placed during the build.
# With the following settings executables are placed in <the top level of the
# build tree>/bin and libraries/archives in <top level of the build tree>/lib.
# This is particularly useful to run ctests on libraries built on Windows
# machines: tests, which are executables, are placed in the same folders of
# dlls, which are treated as executables as well, so that they can properly
# find the libraries to run. This is a because of missing RPATH on Windows.

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}" )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}" )
set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}" )


# To build shared libraries in Windows, we set CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS to TRUE.
# See https://cmake.org/cmake/help/v3.4/variable/CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.html
# See https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/

set( CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON )


# Under MSVC, we set CMAKE_DEBUG_POSTFIX to "d" to add a trailing "d" to library
# built in debug mode. In this Windows user can compile, build and install the
# library in both Release and Debug configuration avoiding naming clashes in the
# installation directories.

if( MSVC )
    set( CMAKE_DEBUG_POSTFIX "d" )
endif( )


# Build position independent code.
# Position Independent Code (PIC) is commonly used for shared libraries so that
# the same shared library code can be loaded in each program address space in a
# location where it will not overlap with any other uses of such memory.
# In particular, this option avoids problems occurring when a process wants to
# load more than one shared library at the same virtual address.
# Since shared libraries cannot predict where other shared libraries could be
# loaded, this is an unavoidable problem with the traditional shared library
# concept.
# Generating position-independent code is often the default behavior for most
# modern compilers.
# Moreover linking a static library that is not built with PIC from a shared
# library will fail on some compiler/architecture combinations.
# Further details on PIC can be found here:
# https://eli.thegreenplace.net/2011/11/03/position-independent-code-pic-in-shared-libraries/

set( CMAKE_POSITION_INDEPENDENT_CODE ON )


# Disable C and C++ compiler extensions.
# C/CXX_EXTENSIONS are ON by default to allow the compilers to use extended
# variants of the C/CXX language.
# However, this could expose cross-platform bugs in user code or in the headers
# of third-party dependencies and thus it is strongly suggested to turn
# extensions off.

set( CMAKE_C_EXTENSIONS OFF )
set( CMAKE_CXX_EXTENSIONS OFF )

set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED 17)


# We use
# - InstallBasicPackageFiles (http://robotology.github.io/ycm/gh-pages/v0.8/module/InstallBasicPackageFiles.html)
# - AddUninstallTarget (http://robotology.github.io/ycm/gh-pages/v0.8/module/AddUninstallTarget.html)
# - AddInstallRPATHSupport (http://robotology.github.io/ycm/gh-pages/v0.8/module/AddInstallRPATHSupport.html)
# from YCM. Files are under the cmake subdirectory.
# See https://github.com/robotology/ycm/
# YCM aquired from icub-common (apt)

find_package( YCM QUIET )
if( YCM_FOUND )
    list(APPEND CMAKE_MODULE_PATH ${YCM_MODULE_PATH})
else( YCM_FOUND )
    list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
endif( YCM_FOUND )


############################################################
################## Options and Packages ####################
############################################################

# Shared/Dynamic or Static library?

option( BUILD_SHARED_LIBS "Build libraries as shared as opposed to static" ON )


# Build test related commands?

option( BUILD_TESTING "Create tests using CMake" OFF )
if( BUILD_TESTING )
    enable_testing( )
endif( )


# Enable RPATH support for installed binaries and libraries

include( AddInstallRPATHSupport )
add_install_rpath_support(
    BIN_DIRS          "${CMAKE_INSTALL_FULL_BINDIR}"
    LIB_DIRS          "${CMAKE_INSTALL_FULL_LIBDIR}"
    INSTALL_NAME_DIR  "${CMAKE_INSTALL_FULL_LIBDIR}"
    USE_LINK_PATH
)


# Encourage user to specify a build type (e.g. Release, Debug, etc.), otherwise set it to Release.

if( NOT CMAKE_CONFIGURATION_TYPES )
    if( NOT CMAKE_BUILD_TYPE )
        message( STATUS "Setting build type to 'Release' as none was specified." )
        set_property( CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release" )
    endif( )
endif( )


option(ENABLE_ROS   "Compile with support for ROS?"  OFF)
option(ENABLE_YARP  "Compile with support for YARP?" OFF)

find_package( yaml-cpp REQUIRED )


############################################################
##################### Library Settings #####################
############################################################

add_subdirectory( HriPhysioLib )

# Create and install CMake configuration files for your project that are
# necessary to for other projects to call find_package(LibTemplateCMake).
#
# Note that it is extremely important to use exactly the project name while
# installing configuration files (you can use PROJECT_NAME variable to avoid
# any possible error). This is required to allow find_package() to properly
# look for the installed library in system path, in particular in Windows when
# the installation is performed in the default path.
#
# install_basic_package_files() comes with many input parameters to customize
# the configuration files. The parameters used in the following call provide
# basic versions of CMake configuration files.
# See install_basic_package_files() documentation found in ./cmake folder.
#
# Note that if your library depends from other libraries, you are probably
# required to used the install_basic_package_files() DEPENDENCIES option.

include( InstallBasicPackageFiles )
install_basic_package_files(
    ${PROJECT_NAME}
    VERSION         ${${PROJECT_NAME}_VERSION}
    COMPATIBILITY   AnyNewerVersion
    VARS_PREFIX     ${PROJECT_NAME}
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)


# Add the uninstall target

include( AddUninstallTarget )


# Add integration tests (unit tests for each library should be in each sublibrary directory).

if( BUILD_TESTING )
    add_subdirectory( tests )
endif( )


############################################################

# find_package(ICUBcontrib REQUIRED)
# 
# list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
# 
# #include(ICUBcontribOptions)
# include(ICUBcontribHelpers)
# 
# icubcontrib_set_default_prefix()


############################################################
###################### Apps & Modules ######################
############################################################

#add_subdirectory( app )
add_subdirectory( modules )

############################################################
