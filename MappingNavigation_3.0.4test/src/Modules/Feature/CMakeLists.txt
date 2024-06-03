set ( LIBRARY_OUTPUT_PATH ../../../release )

include_directories(../Tools/include
    ../Scan/include
    ../Geometry/include
    ../Csm
    ../Csm/src/csm
    ../Csm/include
    ../Csm/include/csm
    ../World/include
    FeatureObject/include
    FeatureMatch/include
    ../ThirdParty/eigen3

    )

set ( Feature_SRCS
	FeatureObject/src/PointFeature.cpp
	FeatureObject/src/ShortLineFeature.cpp
	FeatureObject/src/FlatReflectorFeature.cpp
	FeatureObject/src/CornerPointFeature.cpp
	FeatureObject/src/CylinderFeature.cpp
	FeatureObject/src/RectFeature.cpp
	FeatureObject/src/LineFeature.cpp
	FeatureObject/src/LineFeatureSet.cpp
	FeatureObject/src/PointFeatureSet.cpp
	FeatureObject/src/FeatureSet.cpp
	FeatureObject/src/FeatureMap.cpp
	FeatureMatch/src/CMatrix.cpp
	FeatureMatch/src/LeastSquareMethod.cpp
	FeatureMatch/src/LineMatchPair.cpp
	FeatureMatch/src/LineMatchList.cpp
	FeatureMatch/src/PointMatchPair.cpp
	FeatureMatch/src/PointMatchList.cpp
	FeatureMatch/src/MatchTabSet.cpp
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


    include_directories(../../Modules/ThirdParty/yaml-cpp/include
        )
endif()


#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)


    include_directories(../../Modules/ThirdParty/yaml-cpp/include
        )

endif()

add_library ( Feature SHARED ${Feature_SRCS}  )

