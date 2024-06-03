set ( LIBRARY_OUTPUT_PATH ../../../release )


if(PLATFORM_RK3399)
    include_directories(/opt/RK3399/glib_rk3399/glib-2.0
            /opt/RK3399/glib_rk3399/include
            /opt/RK3399/glib_rk3399

            )
endif()

include_directories(../Tools/include
        include
        ../ThirdParty/eigen3
        )


set ( Geometry_SRCS
	src/Angle.cpp
	src/Arc.cpp
	src/Bezier.cpp
	src/Circle.cpp
	src/Direct.cpp
	src/Ellipse.cpp
	src/Frame.cpp
	src/Line.cpp
	src/LineBase.cpp
	src/MultiSegLine.cpp
	src/NewCurve.cpp
	src/Pnt.cpp
	src/PolyRegion.cpp
	src/Posture.cpp
	src/DataRange.cpp
	src/Rectangle.cpp
	src/ScrnRef.cpp
	src/Spline.cpp
	src/Scp.cpp
	src/Spp.cpp
	src/Transfor.cpp
	src/AppArea.cpp
	src/AffinePosture.cpp
	src/MatchInfo.cpp
	)

if(PLATFORM_RK3399)
    add_compile_definitions(_RK3399_ARM_64)
    #glib
    set(GLIB_DIR "/opt/RK3399/glib_rk3399/glib-2.0" "/opt/RK3399/glib_rk3399/include" "/opt/RK3399/glib_rk3399")
    set(GLIB_LIB_DIR "/opt/RK3399/glib_rk3399/glib-2.0" "/opt/RK3399/glib_rk3399")
    include_directories(${GLIB_DIR})
    link_directories(${GLIB_LIB_DIR})
endif()

if(PLATFORM_E3845)
    add_compile_definitions(_E3845_LINUX64)
endif()


#MRC3588
if(PLATFORM_MRC3588)
    add_compile_definitions(_RK3588_LINUX64)
    add_compile_definitions(NAV_APP)

    #glib

endif()


#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)

endif()

add_library ( Geometry SHARED ${Geometry_SRCS}  )
link_libraries(${GLIB_LIB_DIR})
