#-------------------------------------------------
#
# Project created by QtCreator 2021-10-07T14:51:24
#
#-------------------------------------------------

QT       += core gui
CONFIG   +=  c++14

TARGET = NavBase
TEMPLATE = lib

#QMAKE_CXXFLAGS_RELEASE += -O0
########For Qt Release Debug#############
#QMAKE_CXXFLAGS_RELEASE = $$QMAKE_CFLAGS_RELEASE_WITH_DEBUGINFO
#QMAKE_LFLAGS_RELEASE = $$QMAKE_LFLAGS_RELEASE_WITH_DEBUGINFO
#########################################

DEFINES += NAVBASE_LIBRARY

include($$PWD/../../../Pri/common.pri)
include($$PWD/../../../Pri/Geometry.pri)
include($$PWD/../../../Pri/Tools.pri)
include($$PWD/../../../Pri/Scan.pri)
include($$PWD/../../../Pri/ThirdParty.pri)
include($$PWD/../../../Pri/ndt_oru.pri)
include($$PWD/../../../Pri/Carto.pri)
include($$PWD/../../../Pri/Diagnosis.pri)


DESTDIR = $$LIFELONG_SLAM_DIR


INCLUDEPATH += \
            BlackBox/include \
            CanDev/src/CanOpen/include/ \
            CanDev/include/ \
            LaserDrivers/pf_r2000/include/ \
            LaserDrivers/pf_r2000/include/pepperl_fuchs_r2000/ \
            LaserDrivers/fr_R2/include/ \
            LaserDrivers/fr_R2/include/pepperl_fuchs_r2000_fr/ \
            LaserDrivers/Hokuyo_uam05lp/include/ \
            LaserDrivers/Sick_nanoScan3/include/ \
            LaserDrivers/WJ/include/ \
            LaserDrivers/Leimou_f30/include/ \
            LaserSensor/include/ \
            Network/include \
            Protocol/include \
            Mapping/include \
            ../ThirdParty/eigen3/ \
            ../Csm/include \
            ../Csm/include/csm \
            ../Csm/src\
            ../Csm/src/csm \
            ../Ndt/ndt_oru/include \
            ../Common/Platform/include \
            ../../lib/Navigation/Slam/include \
            ../Ndt/ndt_fuser/include \
            ../Methods/Manager/include \
            ../Methods/NdtMethod/include \
            ../Methods/FeatureMethod/include \
            ../Methods/TemplateMethod/include \
            ../Methods/SlamMethod/include \
            ../Methods/FastMatchMethod/include \
            ../Feature/FeatureObject/include \
            ../World/include \
            ../Feature/FeatureMatch/include \
            ../../App/NavApp/include \
            ../NavBase/Mapping/include


SOURCES += LaserDrivers/WJ//src/wj_716N_lidar_protocol.cpp \
     LaserDrivers/WJ//src/async_client.cpp \
     LaserDrivers/WJ//src/wj_719_lidar_protocol.cpp \
     Mapping/src/Calibrate.cpp \
     LaserDrivers/fr_R2/src/http_command_interface_fr.cpp \
     LaserDrivers/fr_R2/src/r2000_driver_fr.cpp \
     LaserDrivers/fr_R2/src/scan_data_receiver_fr.cpp \
     LaserDrivers/Leimou_f30/src/leimou_f30_driver.cpp \
     LaserSensor/src/LeimouF30LaserScanner.cpp \
     LaserSensor/src/FRR2LaserScanner.cpp

SOURCES += \
    BlackBox/src/BlackBox.cpp \
    BlackBox/src/MemFileMap.cpp \
    BlackBox/src/ShareMem.cpp \
    CanDev/src/CanOpen/SocketCanLib/libsocketcan.c \
    CanDev/src/CanOpen/CanChannel.cpp \
    CanDev/src/CanMan.cpp \
    CanDev/src/CanGyro.cpp \
    LaserDrivers/pf_r2000/src/http_command_interface.cpp \
    LaserDrivers/pf_r2000/src/r2000_driver.cpp \
    LaserDrivers/pf_r2000/src/scan_data_receiver.cpp \
    LaserDrivers/Hokuyo_uam05lp/samples/cpp/Connection_information.cpp \
    LaserDrivers/Hokuyo_uam05lp/src/ticks.cpp \
    LaserDrivers/Hokuyo_uam05lp/src/urg_connection.c \
    LaserDrivers/Hokuyo_uam05lp/src/urg_debug.c \
    LaserDrivers/Hokuyo_uam05lp/src/Urg_driver.cpp \
    LaserDrivers/Hokuyo_uam05lp/src/urg_ring_buffer.c \
    LaserDrivers/Hokuyo_uam05lp/src/urg_sensor.c \
    LaserDrivers/Hokuyo_uam05lp/src/urg_serial.c \
    LaserDrivers/Hokuyo_uam05lp/src/urg_serial_utils.c \
    LaserDrivers/Hokuyo_uam05lp/src/urg_tcpclient.c \
    LaserDrivers/Hokuyo_uam05lp/src/urg_utils.c \
    LaserDrivers/Sick_nanoScan3/src/cola2/ApplicationNameVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/ChangeCommSettingsCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/CloseSession.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/Cola2Session.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/Command.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/ConfigMetadataVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/CreateSession.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/DeviceNameVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/DeviceStatusVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/FieldGeometryVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/FieldHeaderVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/FieldSetsVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/FindMeCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/FirmwareVersionVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/LatestTelegramVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/MeasurementCurrentConfigVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/MeasurementPersistentConfigVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/MethodCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/MonitoringCaseTableHeaderVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/MonitoringCaseVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/OrderNumberVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/ProjectNameVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/RequiredUserActionVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/SerialNumberVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/StatusOverviewVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/TypeCodeVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/UserNameVariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/cola2/VariableCommand.cpp \
    LaserDrivers/Sick_nanoScan3/src/communication/AsyncTCPClient.cpp \
    LaserDrivers/Sick_nanoScan3/src/communication/AsyncUDPClient.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseApplicationData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseApplicationNameData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseConfigMetadata.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDataHeader.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDatagramHeader.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDerivedValues.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDeviceName.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDeviceStatus.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseFieldGeometryData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseFieldHeaderData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseFieldSetsData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseFirmwareVersion.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseGeneralSystemState.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseIntrusionData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseMeasurementCurrentConfigData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseMeasurementData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseMeasurementPersistentConfigData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseMonitoringCaseData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseOrderNumber.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseProjectName.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseRequiredUserAction.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseSerialNumber.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseStatusOverview.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseTCPPacket.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseTypeCodeData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/ParseUserNameData.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/TCPPacketMerger.cpp \
    LaserDrivers/Sick_nanoScan3/src/data_processing/UDPPacketMerger.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ApplicationData.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ApplicationInputs.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ApplicationName.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ApplicationOutputs.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/CommSettings.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ConfigData.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ConfigMetadata.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/Data.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/DataHeader.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/DatagramHeader.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/DerivedValues.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/DeviceName.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/DeviceStatus.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/FieldData.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/FieldSets.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/FirmwareVersion.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/GeneralSystemState.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/IntrusionData.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/IntrusionDatum.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/LatestTelegram.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/MeasurementData.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/MonitoringCaseData.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/OrderNumber.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/PacketBuffer.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ParsedPacketBuffer.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ProjectName.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/RequiredUserAction.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/ScanPoint.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/SerialNumber.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/StatusOverview.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/TypeCode.cpp \
    LaserDrivers/Sick_nanoScan3/src/datastructure/UserName.cpp \
    LaserDrivers/Sick_nanoScan3/src/SickSafetyscannersBase.cpp \
    LaserSensor/src/HokuyoLaserScanner.cpp \
    LaserSensor/src/PfR2000LaserScanner.cpp \
    LaserSensor/src/RangeScanner.cpp \
    LaserSensor/src/SensorFamily.cpp \
    LaserSensor/src/Sick581LaserScanner.cpp \
    LaserSensor/src/SickSafetyLaserScanner.cpp \
    LaserSensor/src/laser_t.c \
    Network/src/Base/Channel.cpp \
    Network/src/Base/CSocket.cpp \
    Network/src/Base/FrmChan.cpp \
    Network/src/Base/TcpChannel.cpp \
    Network/src/Base/TcpSock.cpp \
    Network/src/Base/TimeOutSock.cpp \
    Network/src/Base/UdpChannel.cpp \
    Network/src/Base/UdpSock.cpp \
    Network/src/Client/AgvUdpSock.cpp \
    Network/src/Client/ClntChannel.cpp \
    Network/src/Server/LstnSock.cpp \
    Network/src/Server/TcpSrvChannel.cpp \
    Network/src/Server/TcpSrvCom.cpp \
    Network/src/Server/UdpSrvCom.cpp \
    Protocol/src/AutoOutPutBlackBox.cpp \
    Protocol/src/CPing.cpp \
    Protocol/src/ParameterObject.cpp \
    Protocol/src/RoboLocClnt.cpp \
    Protocol/src/RoboLocProto.cpp \
    Mapping/src/BaseOdometry.cpp \
    Mapping/src/LaserMapping.cpp \
    Mapping/src/LaserOdometry.cpp \
    Mapping/src/RawMap.cpp \
    Mapping/src/LaserAutoMapping.cpp \
    Mapping/src/SendMap.cpp \
    Mapping/src/BuildMap.cpp \
    LaserSensor/src/WJ716LaserScanner.cpp \
    LaserSensor/src/WJ719LaserScanner.cpp
# By Yu Add.
SOURCES += \
        CanDev/src/CanOpen/rsp/vcil/bus_msg_handler.cpp \
        CanDev/src/CanOpen/rsp/vcil/can.cpp \
        CanDev/src/CanOpen/rsp/vcil/cmd_handler.cpp \
        CanDev/src/CanOpen/rsp/vcil/common.cpp \
        CanDev/src/CanOpen/rsp/vcil/DebugLog.cpp \
        CanDev/src/CanOpen/rsp/vcil/j1939.cpp \
        CanDev/src/CanOpen/rsp/vcil/obd2.cpp \
        CanDev/src/CanOpen/rsp/vcil/susi_api_mgr.cpp \
        CanDev/src/CanOpen/rsp/vcil/vcil_cmd.cpp \
        CanDev/src/CanOpen/rsp/vcil/vcil_ctx.cpp \
        CanDev/src/CanOpen/rsp/vcil/vcil_mgr.cpp \
        CanDev/src/CanOpen/rsp/vcil/vcil_port.cpp \
        CanDev/src/CanOpen/rsp/vcil/vmsg.cpp \
        CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/mapping_file_opt.c \
        CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_canDev_vcil.c \
        CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_clockDev_hpet.c \
        CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_dev_stub.c \
        CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_feram_mmap.c \
        CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_gpio.c \
        CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_uartDev.c




Desktop{
DEFINES += DesktopRun
unix:!macx: LIBS += -lboost_system \
                    -lboost_filesystem \
                    -lboost_thread \
                    -lboost_atomic

INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9 \
                /opt/glog/glog_h/
LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so




#INCLUDEPATH +=  /usr/include/glib-2.0 \
#                /usr/lib/x86_64-linux-gnu/glib-2.0/include

LIBS += -lglib-2.0
LIBS +=$$PWD/../../Modules/NavBase/Mapping/lib/libCalibrateTool_x86.so
}

RK3399{
    DEFINES += _LINUX64
    DEFINES+=_RK3399_ARM_64
    DEFINES += NAV_APP
    INCLUDEPATH += /opt/RK3399/boostlib/include/
    LIBS += -L /opt/RK3399/boostlib/lib/ -lboost_system -lboost_thread -lboost_atomic

    INCLUDEPATH +=  /opt/RK3399/glib_rk3399/glib-2.0 \
                    /opt/RK3399/glib_rk3399/include
    LIBS += /opt/RK3399/glib_rk3399/libglib-2.0*.so

    INCLUDEPATH += /opt/RK3399/pcl_1_9_0/include/pcl-1.9
    LIBS += /opt/RK3399/pcl_1_9_0/lib/libpcl_*.so

LIBS +=$$PWD/../../../Parameters/libCalibrateTool_RK3399.so

unix{
    target.path = /home/proembed/CarryBoy/
    INSTALLS += target
  }


}

E3845{
DEFINES += _LINUX64
DEFINES += _E3845_LINUX64
DEFINES += _SERVICE_HARDWARE
DEFINES += NAV_APP

unix:!macx: LIBS += -lboost_system \
                    -lboost_filesystem \
                    -lboost_thread \
                    -lboost_atomic

INCLUDEPATH +=  /usr/include/glib-2.0 \
                     /usr/lib/x86_64-linux-gnu/glib-2.0/include \

LIBS += -lglib-2.0
INCLUDEPATH += /opt/pcl_1_9_0/include/pcl-1.9
LIBS += /opt/pcl_1_9_0/lib/libpcl_*.so

LIBS +=$$PWD/../../Modules/NavBase/Mapping/lib/libCalibrateTool_x86.so

unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}
PICM4{
    CONFIG +=  c++17

    DEFINES += _PiCM4_LINUX32

    INCLUDEPATH += /opt/raspberrypi/sysroot/usr/include/boost/
    LIBS += -L /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/ -lboost_system -lboost_thread -lboost_atomic

    INCLUDEPATH +=  /opt/raspberrypi/sysroot/usr/include/glib-2.0 \
                    /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/glib-2.0/include
    LIBS += /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/libglib-2.0*.so

    INCLUDEPATH += /opt/raspberrypi/sysroot/usr/include/pcl-1.9
    LIBS += /opt/raspberrypi/sysroot/usr/lib/arm-linux-gnueabihf/libpcl_*.so

unix {
        target.path = /home/siasun/CarryBoy/
        INSTALLS += target
     }
}
