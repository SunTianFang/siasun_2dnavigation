#-------------------------------------------------
#
# Project created by QtCreator 2021-09-01T15:21:03
#
#-------------------------------------------------

TARGET = CanDev
TEMPLATE = lib

DEFINES += CANDEV_LIBRARY

include($$PWD/../../../common.pri)
include($$PWD/../../../Tools.pri)
include($$PWD/../../../BlackBox.pri)
include($$PWD/../../../ThirdParty.pri)
DESTDIR = $$LIFELONG_SLAM_DIR

SOURCES += src/CanOpen/rsp/vcil/bus_msg_handler.cpp \
            src/CanOpen/rsp/vcil/can.cpp \
            src/CanOpen/rsp/vcil/cmd_handler.cpp \
            src/CanOpen/rsp/vcil/common.cpp \
            src/CanOpen/rsp/vcil/DebugLog.cpp \
            src/CanOpen/rsp/vcil/j1939.cpp \
            src/CanOpen/rsp/vcil/obd2.cpp \
            src/CanOpen/rsp/vcil/susi_api_mgr.cpp \
            src/CanOpen/rsp/vcil/vcil_cmd.cpp \
            src/CanOpen/rsp/vcil/vcil_ctx.cpp \
            src/CanOpen/rsp/vcil/vcil_mgr.cpp \
            src/CanOpen/rsp/vcil/vcil_port.cpp \
            src/CanOpen/rsp/vcil/vmsg.cpp \
            src/CanOpen/rsp/rsp_dev_adv_e3845/mapping_file_opt.c \
            src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_canDev_vcil.c \
            src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_clockDev_hpet.c \
            src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_dev_stub.c \
            src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_feram_mmap.c \
            src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_gpio.c \
            src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_uartDev.c \
            src/CanOpen/SocketCanLib/libsocketcan.c \
            src/CanOpen/CanChannel.cpp \
            src/CanMan.cpp \
            src/CanGyro.cpp

LIBS +=  -lpthread \
         -ldl\
         -lrt\
         -fopenmp

INCLUDEPATH +=  src/CanOpen/include/ \
                include/
