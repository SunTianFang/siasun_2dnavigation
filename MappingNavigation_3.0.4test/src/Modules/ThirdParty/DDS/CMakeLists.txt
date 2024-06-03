cmake_minimum_required(VERSION 2.8.3)
# project(dds_demo)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# set(SYSROOT_PATH  "/opt/RK3588Debian/sysroot")
# set(CMAKE_SYSROOT "${SYSROOT_PATH}")

if(PLATFORM_MRC3588)
    set(DDS_INCLUDE_DIR "/opt/RK3588Debian/sysroot/usr/include")
    set(DDS_LOCAL_INCLUDE_DIR "/opt/RK3588Debian/sysroot/usr/local/include")
    include_directories(
        ${PROJECT_SOURCE_DIR}/DDS/include
        ${DDS_INCLUDE_DIR}
        ${DDS_LOCAL_INCLUDE_DIR}
    )
endif(PLATFORM_MRC3588)

#if(USE_DESKTOP)
#    # Find requirements
#    find_package(Boost REQUIRED)
#    find_package(fastcdr REQUIRED)
#    find_package(fastrtps REQUIRED)
#endif(USE_DESKTOP)

file(GLOB DDS_DIR src/*.cxx src/*.cpp)

add_library(DDS_Data SHARED ${DDS_DIR})

#if(USE_DESKTOP)
#    target_link_libraries(DDS_Data fastcdr fastrtps)
#endif(USE_DESKTOP)

if(PLATFORM_MRC3588)
    target_link_libraries(
        DDS_Data
        # fastcdr
        # fastrtps
        /opt/RK3588Debian/sysroot/usr/local/lib/libfastcdr.so.1.0.26
        /opt/RK3588Debian/sysroot/usr/local/lib/libfastrtps.so.2.9.0
        /opt/RK3588Debian/sysroot/usr/local/lib/libfoonathan_memory-0.7.1.so
        /opt/RK3588Debian/sysroot/usr/lib/aarch64-linux-gnu/libpcre.so.3
        /opt/RK3588Debian/sysroot/usr/lib/aarch64-linux-gnu/libtinyxml2.so.8
        /opt/RK3588Debian/sysroot/usr/lib/aarch64-linux-gnu/libssl.so.1.1
        /opt/RK3588Debian/sysroot/usr/lib/aarch64-linux-gnu/libcrypto.so.1.1
        /opt/RK3588Debian/sysroot/usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.74.0
        /opt/RK3588Debian/sysroot/usr/lib/aarch64-linux-gnu/libboost_thread.so.1.74.0
        )
endif(PLATFORM_MRC3588)


	

	
	

