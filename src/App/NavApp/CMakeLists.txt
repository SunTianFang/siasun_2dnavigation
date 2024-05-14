project( robo_localization )

set ( EXECUTABLE_OUTPUT_PATH ../../../release )

include_directories(
        ../../../BuildInclude
        include
        ../../Modules/Geometry/include
        ../../Modules/Traj/include
        ../../Modules/World/include
        ../../Modules/Feature/FeatureObject/include
        ../../Modules/Feature/FeatureMatch/include
        ../../Modules/Csm/include
        ../../Modules/Csm/include/csm
        ../../Modules/Csm/src/csm
        ../../Modules/Ndt/ndt_oru/include
        ../../Modules/Ndt/ndt_omp/include
        ../../Modules/Ndt/interface/include
        ../../Modules/Ndt/ndt_fuser/include
        ../../Modules/Common/Platform/include
        ../../Modules/Tools/include
        ../../Modules/Scan/include
        ../../Modules/Diagnosis/include
        ../../Modules/Methods/Manager/include
        ../../Modules/Methods/NdtMethod/include
        ../../Modules/Methods/FeatureMethod/include
        ../../Modules/Methods/TemplateMethod/include
        ../../Modules/Methods/FastMatchMethod/include
        ../../Modules/Methods/ScanMatchMethod/include
        ../../Modules/Methods/SlamMethod/include
        ../../ModulesModules/Editable/Feature/include
        ../../Modules/Editable/Methods/include
        ../../Modules/Editable/StaticObjs/include
        ../../Modules/Editable/Manager/include
        ../../lib/Navigation/Tools/include
        ../../lib/Navigation/Slam/include
        ../../Modules/SoftPls/include
        ../../Modules/NavBase/Protocol/include
        ../../Modules/NavBase/Mapping/include
        ../../Modules/NavBase/LaserSensor/include
        ../../Modules/NavBase/BlackBox/include
        ../../Modules/NavBase/CanDev/include
        ../../Modules/Diagnosis/src/WebtoolServer
        ../../Modules/ThirdParty/eigen3

        ../../Modules/Carto/
        ../../Modules/Carto/mapping/include
        ../../Modules/Carto/common/include
        ../../Modules/Carto/transform/include
        ../../Modules/Carto/sensor/include
        ../../Modules/Carto/myceres/include
        ../../Modules/NavBase/Network/include
        ../../Modules/NavBase/LaserDrivers/pf_r2000/include
        ../../Modules/NavBase/LaserDrivers/fr_R2/include
        ../../Modules/NavBase/LaserDrivers/Hokuyo_uam05lp/include
        ../../Modules/NavBase/LaserDrivers/Sick_nanoScan3/include
        ../../Modules/NavBase/CanDev/src/CanOpen/include
        ../../Modules/NavBase/LaserDrivers/WJ/include
        ../../Modules/NavBase/LaserDrivers/Leimou_f30/include
        ../../Modules/NavBase/LaserDrivers/BlueSea/sdk
        ../../Modules/NavBase/LaserDrivers/BlueSea/sdk/service
        ../../Modules/Feature/FeatureObject/include
        ../../Modules/Feature/FeatureMatch/include
        ../../Modules/ThirdParty/jsoncpp/include
        ../../Modules/ThirdParty/lcm/include
        ../../Modules/Csm/src
        )

set ( robo_localization_SRCS
	src/LocalizeFactory.cpp
        src/main.cpp
	src/RoboManager.cpp
        ../../Modules/NavBase/Protocol/src/httpcommunicationglobaldata.cpp

        ../../Modules/Csm/src/csm/CsmMatcher.cpp

        )

#RK3399
if(PLATFORM_RK3399)
    add_compile_definitions(_RK3399_ARM_64)
    add_compile_definitions(NAV_APP)

    #boost
    set(BOOST_INCLUDE_DIR "/opt/RK3399/boostlib/include")
    set(BOOST_LIB_DIR "/opt/RK3399/boostlib/lib")
    include_directories(${BOOST_INCLUDE_DIR})
    link_directories(${BOOST_LIB_DIR})
    #pcl
    set(PCL_DIR "/opt/RK3399/pcl_1_9_0/share/pcl-1.9")
    find_package(PCL 1.9 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    #glib
    set(GLIB_DIR "/opt/RK3399/glib_rk3399/glib-2.0" "/opt/RK3399/glib_rk3399/include")
    set(GLIB_LIB_DIR "/opt/RK3399/glib_rk3399/glib-2.0" "/opt/RK3399/glib_rk3399")
    include_directories(${GLIB_DIR})
    link_directories(${GLIB_LIB_DIR})
endif()

#E3845
if(PLATFORM_E3845)
    add_compile_definitions(_E3845_LINUX64)
    add_compile_definitions(NAV_APP)

    #pcl
    find_package(PCL 1.7 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
endif()


#MRC3588
if(PLATFORM_MRC3588)
    add_compile_definitions(_RK3588_LINUX64)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)

    #PCL
    set(PCL_DIR "/opt/Mrc3588/pcl/include/pcl-1.11")
    include_directories(${PCL_DIR}
        /opt/Mrc3588/boost/include
        /opt/Mrc3588/pcl/include
        ../../Modules/ThirdParty/yaml-cpp/include
        )
    set(PCL_LIB_DIR "/opt/Mrc3588/pcl/lib")
    link_directories(${PCL_LIB_DIR})


    set(BOOST_INCLUDE_DIR "/opt/Mrc3588/boost/include" )
    set(BOOST_LIB_DIR "/opt/Mrc3588/boost/lib")
    include_directories(${BOOST_INCLUDE_DIR})
    link_directories(${BOOST_LIB_DIR})


    set(GLIB_INCLUDE_DIR "/opt/Mrc3588/glib/include/glib-2.0" "/opt/Mrc3588/glib/include")
    set(GLIB_LIB_DIR "/opt/Mrc3588/glib/lib" )
    include_directories(${GLIB_INCLUDE_DIR})
    link_directories(${GLIB_LIB_DIR})

    #DDS
    #set(DDS_INCLUDE_DIR "/opt/RK3588Debian/sysroot/usr/local/include")
    set(DDS_INCLUDE_DIR "/opt/Mrc3588/sysrootnew/usr/local/include")
    include_directories(
        ../../Modules/ThirdParty/DDS/include
        ${DDS_INCLUDE_DIR}
        )

    #glibc
#    set(GLIBC_LIB_DIR "/opt/Mrc3588/glib/libc" )
#    link_directories(${GLIBC_LIB_DIR})
endif()

#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)

    #PCL
    set(PCL_DIR "/opt/PI/pcl/include/pcl-1.11")
    include_directories(${PCL_DIR}
        /opt/PI/boost/include
        /opt/PI/pcl/include
        ../../Modules/ThirdParty/yaml-cpp/include

        )
    set(PCL_LIB_DIR "/opt/PI/pcl/lib")
    link_directories(${PCL_LIB_DIR})


    set(BOOST_INCLUDE_DIR "/opt/PI/boost/include" )
    set(BOOST_LIB_DIR "/opt/PI/boost/lib")
    include_directories(${BOOST_INCLUDE_DIR})
    link_directories(${BOOST_LIB_DIR})


    set(GLIB_INCLUDE_DIR "/opt/PI/glib/include/glib-2.0" "/opt/PI/glib/include")
    set(GLIB_LIB_DIR "/opt/PI/glib/lib" )
    include_directories(${GLIB_INCLUDE_DIR})
    link_directories(${GLIB_LIB_DIR})

    #glibc
    set(GLIBC_LIB_DIR "/opt/PI/glib/libc" )
    link_directories(${GLIBC_LIB_DIR})

endif()


add_executable ( robo_localization src/main.cpp ${robo_localization_SRCS})
target_link_libraries(robo_localization PRIVATE
    Nav_ThirdParty
    Tools
    Geometry
#    Carto
    Csm
    Scan
    NavBase
#    Feature
#    ndt_oru
#    ndt_omp
    Methods
    Diagnosis
    SoftPls
    dl
#    pthread rt
    boost_system
    boost_thread
    ${PCL_LIBRARIES}
    )

if(PLATFORM_RK3399)
    set_target_properties(robo_localization PROPERTIES LINK_FLAGS "-Wl,-rpath=/home/proembed/CarryBoy")
endif()

if(PLATFORM_E3845)
    set_target_properties(robo_localization PROPERTIES LINK_FLAGS "-Wl,-rpath=/home/siasun/CarryBoy")
endif()

if(PLATFORM_MRC3588)
    target_link_libraries(robo_localization PRIVATE
        -L/opt/Mrc3588/glib/libc rt pthread c
        libglib-2.0.so libpcre.so liblz4.so
        libboost_system.so   libboost_thread.so
        libpcl_common.so     libpcl_ml.so                libpcl_search.so
        libpcl_features.so   libpcl_octree.so            libpcl_segmentation.so
        libpcl_filters.so    libpcl_stereo.so            libpcl_sample_consensus.so
        libpcl_io_ply.so     libpcl_people.so            libpcl_tracking.so
        libpcl_kdtree.so     libpcl_registration.so      libpcl_keypoints.so
        /lib/aarch64-linux-gnu/libc.so

        /home/sft/test_new/MappingNavigation_3.0.4test/Parameters/libFeature.so
        /home/sft/test_new/MappingNavigation_3.0.4test/Parameters/libndt_oru.so
        /home/sft/test_new/MappingNavigation_3.0.4test/Parameters/libndt_omp.so
        /home/sft/test_new/MappingNavigation_3.0.4test/Parameters/libCarto.so

#        /opt/Mrc3588/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libc.so.6
#        /opt/Mrc3588/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libm.so
#        /opt/Mrc3588/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libm.so.6
#        /opt/Mrc3588/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libm.a
#        /opt/Mrc3588/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libstdc++.so.6
#        libm.a               libm.so                     libm.so.6
#        libc.so              libc.so.6
#        /opt/Mrc3588/glib/libstdc++/libstdc++.so.6
        /opt/Mrc3588/sysrootnew/usr/lib/aarch64-linux-gnu/libstdc++.so.6
        )





    set_target_properties(robo_localization PROPERTIES LINK_FLAGS "-Wl,-rpath=/userdata/CarryBoy/NAV/")

endif()

if(PLATFORM_PI)
    target_link_libraries(robo_localization PRIVATE
        -L/opt/PI/glib/libc/ rt pthread c
        libglib-2.0.so libpcre.so liblz4.so
        libboost_system.so   libboost_thread.so
        libpcl_common.so     libpcl_ml.so                libpcl_search.so
        libpcl_features.so   libpcl_octree.so            libpcl_segmentation.so
        libpcl_filters.so    libpcl_stereo.so            libpcl_sample_consensus.so
        libpcl_io_ply.so     libpcl_people.so            libpcl_tracking.so
        libpcl_kdtree.so     libpcl_registration.so      libpcl_keypoints.so
        libm.a               libm.so                     libm.so.6
        libc.so              libc.so.6
        /opt/PI/glib/libstdc++/libstdc++.so.6
        )

    set_target_properties(robo_localization PROPERTIES LINK_FLAGS "-Wl,-rpath=/home/siasun/CarryBoy/NAV")

endif()

