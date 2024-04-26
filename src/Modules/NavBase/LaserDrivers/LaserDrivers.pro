#-------------------------------------------------
#
# Project created by QtCreator 2021-09-01T13:50:11
#
#-------------------------------------------------


QT       += core gui

QMAKE_CXXFLAGS_RELEASE += -O2
CONFIG   += c++11

TARGET = LaserDrivers
TEMPLATE = lib

DEFINES += LASERDRIVERS_LIBRARY



LIBS +=  -lpthread \
         -ldl\
         -lrt\
         -fopenmp \


include($$PWD/../../../common.pri)
include($$PWD/../../../Tools.pri)

DESTDIR = $$LIFELONG_SLAM_DIR

INCLUDEPATH += pf_r2000/include/ \
               pf_r2000/include/pepperl_fuchs_r2000/ \
               Hokuyo_uam05lp/include/ \
               Sick_nanoScan3/include/

SOURCES += pf_r2000/src/http_command_interface.cpp \
    pf_r2000/src/r2000_driver.cpp \
    pf_r2000/src/scan_data_receiver.cpp
SOURCES +=    \
    Hokuyo_uam05lp/samples/cpp/Connection_information.cpp \
    Hokuyo_uam05lp/src/ticks.cpp \
    Hokuyo_uam05lp/src/urg_connection.c \
    Hokuyo_uam05lp/src/urg_debug.c \
    Hokuyo_uam05lp/src/Urg_driver.cpp \
    Hokuyo_uam05lp/src/urg_ring_buffer.c \
    Hokuyo_uam05lp/src/urg_sensor.c \
    Hokuyo_uam05lp/src/urg_serial.c \
    Hokuyo_uam05lp/src/urg_serial_utils.c \
    Hokuyo_uam05lp/src/urg_tcpclient.c \
    Hokuyo_uam05lp/src/urg_utils.c
SOURCES +=    Sick_nanoScan3/src/cola2/ApplicationNameVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/ChangeCommSettingsCommand.cpp \
    Sick_nanoScan3/src/cola2/CloseSession.cpp \
    Sick_nanoScan3/src/cola2/Cola2Session.cpp \
    Sick_nanoScan3/src/cola2/Command.cpp \
    Sick_nanoScan3/src/cola2/ConfigMetadataVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/CreateSession.cpp \
    Sick_nanoScan3/src/cola2/DeviceNameVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/DeviceStatusVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/FieldGeometryVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/FieldHeaderVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/FieldSetsVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/FindMeCommand.cpp \
    Sick_nanoScan3/src/cola2/FirmwareVersionVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/LatestTelegramVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/MeasurementCurrentConfigVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/MeasurementPersistentConfigVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/MethodCommand.cpp \
    Sick_nanoScan3/src/cola2/MonitoringCaseTableHeaderVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/MonitoringCaseVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/OrderNumberVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/ProjectNameVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/RequiredUserActionVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/SerialNumberVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/StatusOverviewVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/TypeCodeVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/UserNameVariableCommand.cpp \
    Sick_nanoScan3/src/cola2/VariableCommand.cpp \
    Sick_nanoScan3/src/communication/AsyncTCPClient.cpp \
    Sick_nanoScan3/src/communication/AsyncUDPClient.cpp

SOURCES +=    Sick_nanoScan3/src/data_processing/ParseApplicationData.cpp \
    Sick_nanoScan3/src/data_processing/ParseApplicationNameData.cpp \
    Sick_nanoScan3/src/data_processing/ParseConfigMetadata.cpp \
    Sick_nanoScan3/src/data_processing/ParseData.cpp \
    Sick_nanoScan3/src/data_processing/ParseDataHeader.cpp \
    Sick_nanoScan3/src/data_processing/ParseDatagramHeader.cpp \
    Sick_nanoScan3/src/data_processing/ParseDerivedValues.cpp \
    Sick_nanoScan3/src/data_processing/ParseDeviceName.cpp \
    Sick_nanoScan3/src/data_processing/ParseDeviceStatus.cpp \
    Sick_nanoScan3/src/data_processing/ParseFieldGeometryData.cpp \
    Sick_nanoScan3/src/data_processing/ParseFieldHeaderData.cpp \
    Sick_nanoScan3/src/data_processing/ParseFieldSetsData.cpp \
    Sick_nanoScan3/src/data_processing/ParseFirmwareVersion.cpp \
    Sick_nanoScan3/src/data_processing/ParseGeneralSystemState.cpp \
    Sick_nanoScan3/src/data_processing/ParseIntrusionData.cpp \
    Sick_nanoScan3/src/data_processing/ParseMeasurementCurrentConfigData.cpp \
    Sick_nanoScan3/src/data_processing/ParseMeasurementData.cpp \
    Sick_nanoScan3/src/data_processing/ParseMeasurementPersistentConfigData.cpp \
    Sick_nanoScan3/src/data_processing/ParseMonitoringCaseData.cpp \
    Sick_nanoScan3/src/data_processing/ParseOrderNumber.cpp \
    Sick_nanoScan3/src/data_processing/ParseProjectName.cpp \
    Sick_nanoScan3/src/data_processing/ParseRequiredUserAction.cpp \
    Sick_nanoScan3/src/data_processing/ParseSerialNumber.cpp \
    Sick_nanoScan3/src/data_processing/ParseStatusOverview.cpp \
    Sick_nanoScan3/src/data_processing/ParseTCPPacket.cpp \
    Sick_nanoScan3/src/data_processing/ParseTypeCodeData.cpp \
    Sick_nanoScan3/src/data_processing/ParseUserNameData.cpp \
    Sick_nanoScan3/src/data_processing/TCPPacketMerger.cpp \
    Sick_nanoScan3/src/data_processing/UDPPacketMerger.cpp

SOURCES +=    Sick_nanoScan3/src/datastructure/ApplicationData.cpp \
    Sick_nanoScan3/src/datastructure/ApplicationInputs.cpp \
    Sick_nanoScan3/src/datastructure/ApplicationName.cpp \
    Sick_nanoScan3/src/datastructure/ApplicationOutputs.cpp \
    Sick_nanoScan3/src/datastructure/CommSettings.cpp \
    Sick_nanoScan3/src/datastructure/ConfigData.cpp \
    Sick_nanoScan3/src/datastructure/ConfigMetadata.cpp \
    Sick_nanoScan3/src/datastructure/Data.cpp \
    Sick_nanoScan3/src/datastructure/DataHeader.cpp \
    Sick_nanoScan3/src/datastructure/DatagramHeader.cpp \
    Sick_nanoScan3/src/datastructure/DerivedValues.cpp \
    Sick_nanoScan3/src/datastructure/DeviceName.cpp \
    Sick_nanoScan3/src/datastructure/DeviceStatus.cpp \
    Sick_nanoScan3/src/datastructure/FieldData.cpp \
    Sick_nanoScan3/src/datastructure/FieldSets.cpp \
    Sick_nanoScan3/src/datastructure/FirmwareVersion.cpp \
    Sick_nanoScan3/src/datastructure/GeneralSystemState.cpp \
    Sick_nanoScan3/src/datastructure/IntrusionData.cpp \
    Sick_nanoScan3/src/datastructure/IntrusionDatum.cpp \
    Sick_nanoScan3/src/datastructure/LatestTelegram.cpp \
    Sick_nanoScan3/src/datastructure/MeasurementData.cpp \
    Sick_nanoScan3/src/datastructure/MonitoringCaseData.cpp \
    Sick_nanoScan3/src/datastructure/OrderNumber.cpp \
    Sick_nanoScan3/src/datastructure/PacketBuffer.cpp \
    Sick_nanoScan3/src/datastructure/ParsedPacketBuffer.cpp \
    Sick_nanoScan3/src/datastructure/ProjectName.cpp \
    Sick_nanoScan3/src/datastructure/RequiredUserAction.cpp \
    Sick_nanoScan3/src/datastructure/ScanPoint.cpp \
    Sick_nanoScan3/src/datastructure/SerialNumber.cpp \
    Sick_nanoScan3/src/datastructure/StatusOverview.cpp \
    Sick_nanoScan3/src/datastructure/TypeCode.cpp \
    Sick_nanoScan3/src/datastructure/UserName.cpp \
    Sick_nanoScan3/src/SickSafetyscannersBase.cpp


unix:!macx: LIBS += -lboost_system \
                    -lboost_filesystem \
                    -lboost_thread \
                    -lboost_atomic
