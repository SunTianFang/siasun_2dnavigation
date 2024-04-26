#-------------------------------------------------
#
# Project created by QtCreator 2021-09-01T13:44:35
#
#-------------------------------------------------

QT       += core gui
QMAKE_CXXFLAGS_RELEASE += -O2

TARGET = Network
TEMPLATE = lib

DEFINES += NETWORK_LIBRARY

include($$PWD/../../../common.pri)
include($$PWD/../../../Tools.pri)


DESTDIR = $$LIFELONG_SLAM_DIR


SOURCES += \
    src/Base/Channel.cpp \
    src/Base/CSocket.cpp \
    src/Base/FrmChan.cpp \
    src/Base/TcpChannel.cpp \
    src/Base/TcpSock.cpp \
    src/Base/TimeOutSock.cpp \
    src/Base/Tools.cpp \
    src/Base/UdpChannel.cpp \
    src/Base/UdpSock.cpp \
    src/Client/AgvUdpSock.cpp \
    src/Client/ClntChannel.cpp \
    src/Server/LstnSock.cpp \
    src/Server/TcpSrvChannel.cpp \
    src/Server/TcpSrvCom.cpp \
    src/Server/UdpSrvCom.cpp

INCLUDEPATH += \
            include
