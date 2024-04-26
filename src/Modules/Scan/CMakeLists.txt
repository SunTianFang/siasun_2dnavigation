set ( LIBRARY_OUTPUT_PATH ../../../release )

include_directories(include
    ../Scan/include
    ../Tools/include
    ../Geometry/include
    ../Csm/include
    ../Csm/include/csm
    ../Csm/src
    ../Csm/src/csm
    ../ThirdParty/eigen3/
    ../ThirdParty/jsoncpp/include
    ../ThirdParty/yaml-cpp/include

    )


set ( Scan_SRCS
	src/ScanPointCloud.cpp
	src/Scan.cpp
	src/ScannerParam.cpp
	src/misc.cpp
	src/RawScan.cpp
	src/LineElement.cpp
	src/CircleElement.cpp
	src/BasObject.cpp
	src/StockedObjects.cpp
	src/StaticObjects.cpp
        src/LiveObjects.cpp
        ../ThirdParty/yaml-cpp/src/node_data.cpp
        ../ThirdParty/yaml-cpp/src/exceptions.cpp
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

endif()


#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)


endif()

add_library ( Scan SHARED ${Scan_SRCS}   )
target_link_libraries(Scan   Nav_ThirdParty
 Tools Csm Geometry)


#MRC3588
#if(PLATFORM_MRC3588)
#    target_link_libraries(Scan /opt/Mrc3588/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libglib-2.0.so
#        )

#endif()

