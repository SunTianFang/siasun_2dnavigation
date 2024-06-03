set ( LIBRARY_OUTPUT_PATH ../../../../release )

include_directories(include
    ../../Tools/include
    ../../Geometry/include
    ../../Scan/include
    ../../Csm/include/csm
    ../../Csm/include
    ../../Csm/src/csm
    ../../Feature/include
    ../../World/include
    ../../../lib/Pcl/include
    ../../../lib/MiniEigen/include
    ../../../lib/Navigation/Slam/include
    ../../../Modules/Common/Platform/include
    ../ndt_fuser/include
    ../ndt_omp/include
    ../../ThirdParty/eigen3/
    ../../ThirdParty/lcm/include
    ../../ThirdParty/jsoncpp/include
    ../../NavBase/Protocol/include
    ../../NavBase/BlackBox/include
    ../../NavBase/Network/include
    ../../NavBase/LaserSensor/include
    ../../NavBase/LaserDrivers/pf_r2000/include
    ../../NavBase/LaserDrivers/fr_R2/include
    ../../NavBase/LaserDrivers/Hokuyo_uam05lp/include
    ../../NavBase/LaserDrivers/Sick_nanoScan3/include
    ../../NavBase/LaserDrivers/WJ/include
    ../../NavBase/LaserDrivers/Leimou_f30/include
    ../../NavBase/LaserDrivers/BlueSea/sdk
    ../../NavBase/LaserDrivers/BlueSea/sdk/service
    ../../NavBase/LaserSensor/include
    ../../NavBase/Mapping/include
    ../../Feature/FeatureObject/include
    ../../Carto
    ../../Carto/transform/include
    ../../Carto/sensor/include
    ../../Carto/common/include
    ../../Diagnosis/include
    ../../Carto/mapping/include
    ../../Carto/myceres/include
    )



set ( ndt_oru_SRCS
	src/ndt_pointcloud.cpp
	src/ndt_cell_oru.cpp
	src/lazy_grid_oru.cpp
	src/ndt_map_oru.cpp
	src/ndt_maps_oru.cpp
	src/ndt_matcher_d2d_2d.cpp
	src/NdtBaseLocalization_oru.cpp
	src/NdtExtLocalization_oru.cpp
	src/ndt_inform_oru.cpp
	../ndt_fuser/src/NdtCellEditable.cpp
	../ndt_fuser/src/NdtMapEditable.cpp
	../ndt_fuser/src/NdtMapsEditable.cpp
	../ndt_fuser/src/DatasetLocalization.cpp
	../ndt_fuser/src/MapFuser.cpp
	../ndt_fuser/src/VectPose.cpp
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
        ../../ThirdParty/yaml-cpp/include

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

add_library ( ndt_oru SHARED ${ndt_oru_SRCS}  )

