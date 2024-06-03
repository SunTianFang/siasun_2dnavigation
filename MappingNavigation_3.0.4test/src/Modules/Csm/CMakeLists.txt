set ( LIBRARY_OUTPUT_PATH ../../../release )

include_directories(include/csm
    ../Geometry/include
    ../Tools/include
    ../ThirdParty/eigen3
    include
    ../Scan/include
    src/csm
    src/
    ../NavBase/Protocol/include
    ../NavBase/BlackBox/include
    ../NavBase/Mapping/include
    )

set ( Csm_SRCS
	src/csm/CorrList.cpp
	src/csm/CsmMatcher.cpp
	src/csm/CsmScan.cpp
	src/csm/fastMatcher.cpp
	src/csm/math_utils.cpp
	src/csm/math_utils_gsl.cpp
	src/egsl/egsl.cpp
	src/egsl/egsl_conversions.cpp
	src/egsl/egsl_misc.cpp
	src/egsl/egsl_ops.cpp
	src/gpc/gpc.cpp
	src/gpc/gpc_utils.cpp
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
        )
endif()


#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)

    set(PCL_DIR "/opt/PI/pcl/include/pcl-1.11")
    include_directories(${PCL_DIR}
        /opt/PI/boost/include
        /opt/PI/pcl/include
        )

endif()


add_library ( Csm SHARED ${Csm_SRCS}  )
target_link_libraries(Csm  ${PCL_LIBRARIES})

