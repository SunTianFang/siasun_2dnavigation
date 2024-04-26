set ( LIBRARY_OUTPUT_PATH ../../../release )

include_directories(include
    ../Tools/include
    ../Geometry/include
    include/RK3399

    )


set ( Tools_SRCS
	src/Archive.cpp
	src/Tools.cpp
	src/time_patch.cpp
	src/Combination.cpp
	src/AbstractParameter.cpp
	src/Gpio.cpp
	src/LED.cpp
	src/systemInfo.cpp
	src/WatchDog.cpp
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

endif()


#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)

endif()


add_library ( Tools SHARED ${Tools_SRCS}  )
