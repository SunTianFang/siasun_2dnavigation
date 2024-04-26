cmake_minimum_required( VERSION 3.12 )


option(PLATFORM_E3845 "if platform is E3845" OFF)
option(PLATFORM_RK3399 "if platform is RK3399" OFF)
option(PLATFORM_PI "if platform is PI" OFF)
option(PLATFORM_MRC3588 "if platform is MRC3588" ON)
option(DEPLOYED_REMOTELY "Turn on for remote deployment" ON)

set(CMAKE_BUILD_TYPE "Release")


#configure_file (
#    "${PROJECT_SOURCE_DIR}/Config.h.in"
#    "${PROJECT_SOURCE_DIR}/BuildInclude/Config.h"
#    )

#RK3399
if(PLATFORM_RK3399)
    SET(CMAKE_C_COMPILER "/usr/bin/aarch64-linux-gnu-gcc")
    set(CMAKE_INSTALL_RPATH "/home/proembed/CarryBoy/")
endif()

#MRC3588
if(PLATFORM_MRC3588)
    SET(CMAKE_C_COMPILER "/opt/Mrc3588/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc")
    SET(CMAKE_CXX_COMPILER "/opt/Mrc3588/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++")
endif()

#Pi
if(PLATFORM_PI)
    SET(CMAKE_C_COMPILER "/opt/PI/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc")
    SET(CMAKE_CXX_COMPILER "/opt/PI/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++")
    set(CMAKE_INSTALL_RPATH "/home/siasun/CarryBoy/NAV/")
endif()

#E3845
if(PLATFORM_E3845)
    SET(CMAKE_C_COMPILER "/usr/bin/gcc")
    set(CMAKE_INSTALL_RPATH "/home/siasun/CarryBoy/")
endif()


set( CMAKE_CXX_STANDARD 14)

project (CNavgation)

message(STATUS "PROJECT_SOURCE_DIR " ${PROJECT_SOURCE_DIR})
message(STATUS "PROJECT_BINARY_DIR " ${PROJECT_BINARY_DIR})

add_subdirectory( src/Modules/ThirdParty )
add_subdirectory( src/Modules/Tools )
add_subdirectory( src/Modules/Geometry )
#add_subdirectory( src/Modules/Carto )
add_subdirectory( src/Modules/Csm )
add_subdirectory( src/Modules/Scan )
add_subdirectory( src/Modules/NavBase )
#add_subdirectory( src/Modules/Feature )
#add_subdirectory( src/Modules/Ndt/ndt_oru )
#add_subdirectory( src/Modules/Ndt/ndt_omp )
add_subdirectory( src/Modules/Methods )
add_subdirectory( src/Modules/Diagnosis )
add_subdirectory( src/Modules/SoftPls )
add_subdirectory( src/App/NavApp )



macro(add_deployment_file SRC DEST)
    file(RELATIVE_PATH path ${CMAKE_SOURCE_DIR}
         ${CMAKE_BINARY_DIR})
    file(APPEND "${CMAKE_BINARY_DIR}/QtCreatorDeployment.txt"
         "${path}/${SRC}:${DEST}\n")
endmacro()

macro(add_deployment_directory SRC DEST)
    file(GLOB_RECURSE files RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
         "${SRC}/*")
    foreach(filename ${files})
        get_filename_component(path ${filename} PATH)
        add_deployment_file("${filename}" "${DEST}/${path}")
    endforeach(filename)
endmacro()

if(DEPLOYED_REMOTELY)
    if(PLATFORM_RK3399)
        set(CMAKE_INSTALL_PREFIX "/home/proembed/")
    elseif(PLATFORM_MRC3588)
        set(CMAKE_INSTALL_PREFIX "/userdata/")
    elseif(PLATFORM_PI)
        set(CMAKE_INSTALL_PREFIX "/home/siasun/")
    endif()
endif()

if(DEPLOYED_REMOTELY)
    # Write base installation path as first line.
    file(WRITE "${CMAKE_BINARY_DIR}/QtCreatorDeployment.txt"
         "${CMAKE_INSTALL_PREFIX}\n")
    # Append mapping for single library file.
    if(PLATFORM_RK3399)
        add_deployment_file("release/robo_localization" "/home/proembed/")
        add_deployment_file("release/libCarto.so" "CarryBoy")
        add_deployment_file("release/libCsm.so" "CarryBoy")
        add_deployment_file("release/libDiagnosis.so" "CarryBoy")
        add_deployment_file("release/libFeature.so" "CarryBoy")
        add_deployment_file("release/libGeometry.so" "CarryBoy")
        add_deployment_file("release/libMethods.so" "CarryBoy")
        add_deployment_file("release/libNavBase.so" "CarryBoy")
        add_deployment_file("release/libNav_ThirdParty.so" "CarryBoy")
        add_deployment_file("release/libndt_omp.so" "CarryBoy")
        add_deployment_file("release/libndt_oru.so" "CarryBoy")
        add_deployment_file("release/libScan.so" "CarryBoy")
        add_deployment_file("release/libSoftPls.so" "CarryBoy")
        add_deployment_file("release/libTools.so" "CarryBoy")
        add_deployment_file("../Parameters/libCalibrateTool_RK3399.so" "CarryBoy")
    elseif(PLATFORM_MRC3588)
        add_deployment_file("release/robo_localization" "CarryBoy/NAV")
        add_deployment_file("release/libCarto.so" "CarryBoy/NAV")
        add_deployment_file("release/libCsm.so" "CarryBoy/NAV")
        add_deployment_file("release/libDiagnosis.so" "CarryBoy/NAV")
        add_deployment_file("release/libFeature.so" "CarryBoy/NAV")
        add_deployment_file("release/libGeometry.so" "CarryBoy/NAV")
        add_deployment_file("release/libMethods.so" "CarryBoy/NAV")
        add_deployment_file("release/libNavBase.so" "CarryBoy/NAV")
        add_deployment_file("release/libNav_ThirdParty.so" "CarryBoy/NAV")
        add_deployment_file("release/libndt_omp.so" "CarryBoy/NAV")
        add_deployment_file("release/libndt_oru.so" "CarryBoy/NAV")
        add_deployment_file("release/libScan.so" "CarryBoy/NAV")
        add_deployment_file("release/libSoftPls.so" "CarryBoy/NAV")
        add_deployment_file("release/libTools.so" "CarryBoy/NAV")
        add_deployment_file("../Parameters/libcamAPI.so" "CarryBoy/NAV")
	add_deployment_file("../Parameters/libCamera.so" "CarryBoy/NAV")
        add_deployment_file("../Parameters/libopencv_world.so.4.5" "CarryBoy/NAV")
        add_deployment_file("../Parameters/libjpeg.so.8" "CarryBoy/NAV")
	add_deployment_file("../Parameters/libCalibrateTool_MRC3588.so" "CarryBoy/NAV")
    elseif(PLATFORM_PI)
        add_deployment_file("release/robo_localization" "CarryBoy/NAV")
        add_deployment_file("release/libCarto.so" "CarryBoy/NAV")
        add_deployment_file("release/libCsm.so" "CarryBoy/NAV")
        add_deployment_file("release/libDiagnosis.so" "CarryBoy/NAV")
        add_deployment_file("release/libFeature.so" "CarryBoy/NAV")
        add_deployment_file("release/libGeometry.so" "CarryBoy/NAV")
        add_deployment_file("release/libMethods.so" "CarryBoy/NAV")
        add_deployment_file("release/libNavBase.so" "CarryBoy/NAV")
        add_deployment_file("release/libNav_ThirdParty.so" "CarryBoy/NAV")
        add_deployment_file("release/libndt_omp.so" "CarryBoy/NAV")
        add_deployment_file("release/libndt_oru.so" "CarryBoy/NAV")
        add_deployment_file("release/libScan.so" "CarryBoy/NAV")
        add_deployment_file("release/libSoftPls.so" "CarryBoy/NAV")
        add_deployment_file("release/libTools.so" "CarryBoy/NAV")
        add_deployment_file("../Parameters/libCalibrateTool_PI.so" "CarryBoy/NAV")
    endif()
endif()

























