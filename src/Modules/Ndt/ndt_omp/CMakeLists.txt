set ( LIBRARY_OUTPUT_PATH ../../../../release )

set ( ndt_omp_SRCS
        src/voxel_grid_covariance_omp.cpp
        src/ndt_omp.cpp
        src/NdtLocalization_omp.cpp
        )

include_directories(include
    ../../../lib/MiniEigen/include
    ../../../Modules/Tools/include
    ../../../Modules/Scan/include
    ../../../lib/Navigation/Slam/include
    ../../../Modules/Ndt/ndt_fuser/include
    ../../../Modules/Ndt/ndt_oru/include
    ../../../Modules/Common/Platform/include
    ../../Geometry/include
    ../../ThirdParty/eigen3

    )
if(PLATFORM_RK3399)
    add_compile_definitions(_RK3399_ARM_64)
    set(PCL_DIR "/opt/RK3399/pcl_1_9_0/share/pcl-1.9")
    find_package(PCL 1.9 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
endif()

if(PLATFORM_E3845)
    find_package(PCL 1.7 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    add_compile_definitions(_E3845_LINUX64)
    add_compile_definitions(_LINUX6)
    add_compile_definitions(_SERVICE_HARDWARE)
    add_compile_definitions(NAV_APP)
endif()




#MRC3588
if(PLATFORM_MRC3588)
    add_compile_definitions(_RK3588_LINUX64)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)

    set(PCL_DIR "/opt/Mrc3588/pcl/include/pcl-1.11")
    include_directories(${PCL_DIR}
        /opt/Mrc3588/boost/include
        /opt/Mrc3588/pcl/include
        ../../Modules/ThirdParty/yaml-cpp/include

        )

endif()


#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)


    set(PCL_DIR "/opt/PI/pcl/include/pcl-1.11")
    include_directories(${PCL_DIR}
        /opt/PI/boost/include
        /opt/PI/pcl/include
        ../../ThirdParty/yaml-cpp/include
        )

endif()

add_library ( ndt_omp SHARED ${ndt_omp_SRCS}  )

