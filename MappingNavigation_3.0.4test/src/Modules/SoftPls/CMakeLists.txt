set ( LIBRARY_OUTPUT_PATH ../../../release )

include_directories(include
    ../Scan/include
    ../Tools/include
    ../Geometry/include
    ../Csm/include
    ../Csm/include/csm
    ../Csm/src
    ../Csm/src/csm
    ../ThirdParty/eigen3
    ../Carto/mapping/include
    ../Carto/common/include
    ../Carto/transform/include
    ../Carto/sensor/include
    ../NavBase/Mapping/include
    ../NavBase/Protocol/include
    ../NavBase/BlackBox/include
    ../NavBase/Network/include
    ../NavBase/LaserSensor/include
    ../NavBase/LaserDrivers/pf_r2000/include
    ../NavBase/LaserDrivers/fr_R2/include
    ../NavBase/LaserDrivers/Hokuyo_uam05lp/include
    ../NavBase/LaserDrivers/Sick_nanoScan3/include
    ../NavBase/LaserDrivers/WJ/include
    ../NavBase/LaserDrivers/Leimou_f30/include
    ../NavBase/LaserDrivers/BlueSea/sdk
    ../NavBase/LaserDrivers/BlueSea/sdk/service
    ../Feature/FeatureObject/include
    ../Feature/FeatureMatch/include
    ../ThirdParty/jsoncpp/include
    ../ThirdParty/lcm/include


    )

set ( SoftPls_SRCS
	src/AvoidObstacle.cpp
	src/PLS.cpp
	)

if(PLATFORM_RK3399)
    add_compile_definitions(_RK3399_ARM_64)
endif()

if(PLATFORM_E3845)
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



add_library ( SoftPls SHARED ${SoftPls_SRCS})


