#-------------------------------------------------
#
# Project created by QtCreator 2021-10-28T16:20:39
#
#-------------------------------------------------

QT       -= core gui
CONFIG +=  c++14

TARGET = ThirdParty
TEMPLATE = lib

DEFINES += THIRDPARTY_LIBRARY

include($$PWD/../../../Pri/common.pri)
DESTDIR = $$LIFELONG_SLAM_DIR

INCLUDEPATH += \
            jsoncpp/include/ \
            jsoncpp/ \
            jsoncpp/src/lib_json/ \
            yaml-cpp/include \
            lcm/include

SOURCES += \
            jsoncpp/src/lib_json/json_reader.cpp \
            jsoncpp/src/lib_json/json_value.cpp \
            jsoncpp/src/lib_json/json_valueiterator.inl \
            jsoncpp/src/lib_json/json_writer.cpp \
            yaml-cpp/src/binary.cpp \
            yaml-cpp/src/directives.cpp \
            yaml-cpp/src/emit.cpp \
            yaml-cpp/src/emitfromevents.cpp \
            yaml-cpp/src/emitter.cpp \
            yaml-cpp/src/emitterstate.cpp \
            yaml-cpp/src/emitterutils.cpp \
            yaml-cpp/src/exceptions.cpp \
            yaml-cpp/src/exp.cpp \
            yaml-cpp/src/memory.cpp \
            yaml-cpp/src/node.cpp \
            yaml-cpp/src/node_data.cpp \
            yaml-cpp/src/nodebuilder.cpp \
            yaml-cpp/src/nodeevents.cpp \
            yaml-cpp/src/null.cpp \
            yaml-cpp/src/ostream_wrapper.cpp \
            yaml-cpp/src/parse.cpp \
            yaml-cpp/src/parser.cpp \
            yaml-cpp/src/regex_yaml.cpp \
            yaml-cpp/src/scanner.cpp \
            yaml-cpp/src/scanscalar.cpp \
            yaml-cpp/src/scantag.cpp \
            yaml-cpp/src/scantoken.cpp \
            yaml-cpp/src/simplekey.cpp \
            yaml-cpp/src/singledocparser.cpp \
            yaml-cpp/src/stream.cpp \
            yaml-cpp/src/tag.cpp \
            lcm/src/channel_port_map_update_t.c \
            lcm/src/channel_to_port_t.c \
            lcm/src/eventlog.c \
            lcm/src/lcm.c \
            lcm/src/lcm_file.c \
            lcm/src/lcm_memq.c \
            lcm/src/lcm_mpudpm.c \
            lcm/src/lcm_tcpq.c \
            lcm/src/lcm_udpm.c \
            lcm/src/ringbuffer.c \
            lcm/src/udpm_util.c



Desktop{
DEFINES += DesktopRun

INCLUDEPATH +=  /usr/include/glib-2.0 \
                /usr/lib/x86_64-linux-gnu/glib-2.0/include

LIBS += -lglib-2.0
}


RK3399{
DEFINES+=_RK3399_ARM_64

INCLUDEPATH +=  /opt/RK3399/glib_rk3399/glib-2.0 \
                /opt/RK3399/glib_rk3399/include
LIBS += /opt/RK3399/glib_rk3399/libglib-2.0*.so

unix{
    target.path = /home/proembed/CarryBoy/
    INSTALLS += target
  }
}

E3845{
DEFINES += _LINUX64
DEFINES += _E3845_LINUX64

INCLUDEPATH +=  /usr/include/glib-2.0 \
                /usr/lib/x86_64-linux-gnu/glib-2.0/include
LIBS += -lglib-2.0

unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}
PICM4{

    CONFIG +=  c++17
    DEFINES += _PiCM4_LINUX32


    INCLUDEPATH +=  /opt/raspberrypi/sysroot/usr/include/glib-2.0 \
                    /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/glib-2.0/include
    LIBS += /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/libglib-2.0*.so

unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}
