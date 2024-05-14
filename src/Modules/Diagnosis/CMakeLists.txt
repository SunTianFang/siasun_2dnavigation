set ( LIBRARY_OUTPUT_PATH ../../../release )
add_definitions(-DMG_ENABLE_HTTP_STREAMING_MULTIPART)


include_directories(include/
    ../Geometry/include
    ../World/include
    ../Ndt/ndt_oru/include
    ../Ndt/ndt_fuser/include
    ../Common/Platform/include
    ../Scan/include
    ../Methods/Manager/include
    ../Methods/NdtMethod/include
    ../Methods/FeatureMethod/include
    ../Methods/TemplateMethod/include
    ../Methods/SlamMethod/include
    ../../lib/Navigation/Slam/include
    /src/WebtoolServer
    ../../App/NavApp/include
    ../Csm/include
    ../Csm/include/csm
    ../Csm/src/csm
    ../Tools/include
    ../Methods/FastMatchMethod/include
    ../ThirdParty/eigen3/
    ../Carto
    ../Carto/mapping/include
    ../Carto/common/include
    ../Carto/transform/include
    ../Carto/sensor/include
    ../Carto/myceres/include
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
    ../ThirdParty/DDS/include
    ../ThirdParty/yaml-cpp/include/yaml-cpp
    ../ThirdParty/yaml-cpp/include
    ../NavBase/Mapping/include
    )


set ( Diagnosis_SRCS
	src/LCMTask.cpp
	src/Localization_Msg.c
	src/LocalizationTest_Msg.c
	src/FeatureMatchInfoLcm_.c
	src/Diagnosis.cpp
	src/LcmCloudAdjusted.c
	src/robot_control_t.c
        src/robot_control_t_new.c
        src/DDSTask.cpp
        src/WebtoolServer/api/archive/ctrlsetting.cpp
	src/WebtoolServer/api/archive/datalocation.cpp
	src/WebtoolServer/api/archive/datapcloud.cpp
	src/WebtoolServer/api/archive/datasetting.cpp
	src/WebtoolServer/api/apimanager.cpp
	src/WebtoolServer/api/ctrlapi.cpp
	src/WebtoolServer/api/dataapi.cpp
	src/WebtoolServer/api/downloadapi.cpp
	src/WebtoolServer/server/httpServer.cpp
	src/WebtoolServer/mongoose/mongoose.c
	src/WebtoolServer/api/archive/dataparm.cpp
	src/WebtoolServer/api/archive/datadxscandata.cpp
	src/WebtoolServer/api/archive/datadxlaserparm.cpp
	src/WebtoolServer/api/archive/datafinishrecorddx.cpp
	src/WebtoolServer/api/archive/datastartrecorddx.cpp

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
    set(DDS_INCLUDE_DIR "/opt/Mrc3588/sysrootnew/usr/local/include")

    include_directories(${PCL_DIR}
        /opt/Mrc3588/boost/include
        /opt/Mrc3588/pcl/include
#        ../ThirdParty/DDS/include/
        ${DDS_INCLUDE_DIR}
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
        )

endif()

add_library ( Diagnosis SHARED ${Diagnosis_SRCS} )

