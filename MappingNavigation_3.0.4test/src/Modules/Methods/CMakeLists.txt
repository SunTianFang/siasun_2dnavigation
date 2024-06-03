set ( LIBRARY_OUTPUT_PATH ../../../release )

include_directories(../Tools/include
    ../Geometry/include
    ../Scan/include
    ../Csm/include/csm
    ../Csm/include
    ../Csm/src/csm
    ../Feature/FeatureObject/include
    ../Feature/FeatureMatch/include
    ../World/include
    ../../lib/MiniEigen/include
    ../../lib/Navigation/Slam/include
    ../../Modules/Common/Platform/include
    ../Ndt/ndt_oru/include
    ../Ndt/ndt_fuser/include
    NdtMethod/include
    FeatureMethod/include
    TemplateMethod/include
    FastMatchMethod/include
    ScanMatchMethod/include
    Manager/include
    ../ThirdParty/eigen3
    ../ThirdParty/jsoncpp/include
    ../ThirdParty/lcm/include
    ../../App/NavApp/include
    ../Carto/transform/include
    ../Carto/sensor/include
    ../Carto/common/include
    ../Carto/mapping/include
    ../Carto/myceres/include
    ../NavBase/Protocol/include
    ../NavBase/BlackBox/include
    ../NavBase/Mapping/include
    ../Diagnosis/include
    ../NavBase/Network/include
     Manager/include
    SlamMethod/include
    ../Carto
    )


set ( Methods_SRCS
	NdtMethod/src/NdtLocArea.cpp
	NdtMethod/src/NdtMethod.cpp
	FeatureMethod/src/FeatureLocArea.cpp
	FeatureMethod/src/FeatureMethod.cpp
	TemplateMethod/src/TemplateLocArea.cpp
	TemplateMethod/src/TemplateMethod.cpp
	Manager/src/LocalizationMethods.cpp
	Manager/src/LocalizationPlan.cpp
	Manager/src/LocalizationManager.cpp
	FeatureMethod/src/FeatureScan.cpp
	FeatureMethod/src/FeatureLocalization.cpp
	FastMatchMethod/src/FastMatchMethod.cpp
	ScanMatchMethod/src/ScanMatchMethod.cpp
        SlamMethod/src/SlamLocArea.cpp
        SlamMethod/src/SlamMethod.cpp
        ScanMatchMethod/src/ScanMatchLocArea.cpp

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
        ../../Modules/ThirdParty/yaml-cpp/include

        )

endif()


add_library ( Methods SHARED ${Methods_SRCS})

