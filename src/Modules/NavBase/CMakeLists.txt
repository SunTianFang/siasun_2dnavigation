set ( LIBRARY_OUTPUT_PATH ../../../release )

include_directories(BlackBox/include
    CanDev/src/CanOpen/include/
    CanDev/include/
    LaserDrivers/pf_r2000/include/
    LaserDrivers/pf_r2000/include/pepperl_fuchs_r2000/
    LaserDrivers/fr_R2/include/
    LaserDrivers/fr_R2/include/pepperl_fuchs_r2000_fr/
    LaserDrivers/Hokuyo_uam05lp/include/
    LaserDrivers/Sick_nanoScan3/include/
    LaserDrivers/WJ/include/
    LaserDrivers/Leimou_f30/include/
    LaserDrivers/BlueSea/sdk/
    LaserDrivers/BlueSea/sdk/service
    LaserSensor/include/
    Network/include
    Protocol/include
    Mapping/include
    ../ThirdParty/eigen3/
    ../Csm/include
    ../Csm/include/csm
    ../Csm/src
    ../Csm/src/csm
    ../Ndt/ndt_oru/include
    ../Common/Platform/include
    ../../lib/Navigation/Slam/include
    ../Ndt/ndt_fuser/include
    ../Methods/Manager/include
    ../Methods/NdtMethod/include
    ../Methods/FeatureMethod/include
    ../Methods/TemplateMethod/include
    ../Methods/FastMatchMethod/include
    ../Feature/FeatureObject/include
    ../Methods/SlamMethod/include
    ../World/include
    ../Feature/FeatureMatch/include
    ../../App/NavApp/include
    ../Tools/include
    ../ThirdParty/jsoncpp/include
    ../ThirdParty/lcm/include
    ../Geometry/include
    ../Scan/include
    ../Carto
    ../Carto/transform/include
    ../Carto/sensor/include
    ../Carto/common/include
    ../Diagnosis/include
    ../Carto/mapping/include
    ../Carto/myceres/include
    ../NavBase/Mapping/include
    )

set ( NavBase_SRCS
        LaserDrivers/Leimou_f30/src/leimou_f30_driver.cpp
        LaserDrivers/WJ/src/wj_716N_lidar_protocol.cpp
        LaserDrivers/WJ/src/async_client.cpp
        LaserDrivers/WJ/src/wj_719_lidar_protocol.cpp
	BlackBox/src/BlackBox.cpp
	BlackBox/src/MemFileMap.cpp
	BlackBox/src/ShareMem.cpp
	CanDev/src/CanOpen/SocketCanLib/libsocketcan.c
	CanDev/src/CanOpen/CanChannel.cpp
	CanDev/src/CanMan.cpp
	CanDev/src/CanGyro.cpp
	LaserDrivers/pf_r2000/src/http_command_interface.cpp
	LaserDrivers/pf_r2000/src/r2000_driver.cpp
	LaserDrivers/pf_r2000/src/scan_data_receiver.cpp
        LaserDrivers/fr_R2/src/http_command_interface_fr.cpp
        LaserDrivers/fr_R2/src/r2000_driver_fr.cpp
        LaserDrivers/fr_R2/src/scan_data_receiver_fr.cpp
	LaserDrivers/Hokuyo_uam05lp/samples/cpp/Connection_information.cpp
	LaserDrivers/Hokuyo_uam05lp/src/ticks.cpp
	LaserDrivers/Hokuyo_uam05lp/src/urg_connection.c
	LaserDrivers/Hokuyo_uam05lp/src/urg_debug.c
	LaserDrivers/Hokuyo_uam05lp/src/Urg_driver.cpp
	LaserDrivers/Hokuyo_uam05lp/src/urg_ring_buffer.c
	LaserDrivers/Hokuyo_uam05lp/src/urg_sensor.c
	LaserDrivers/Hokuyo_uam05lp/src/urg_serial.c
	LaserDrivers/Hokuyo_uam05lp/src/urg_serial_utils.c
	LaserDrivers/Hokuyo_uam05lp/src/urg_tcpclient.c
	LaserDrivers/Hokuyo_uam05lp/src/urg_utils.c
	LaserDrivers/Sick_nanoScan3/src/cola2/ApplicationNameVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/ChangeCommSettingsCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/CloseSession.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/Cola2Session.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/Command.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/ConfigMetadataVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/CreateSession.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/DeviceNameVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/DeviceStatusVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/FieldGeometryVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/FieldHeaderVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/FieldSetsVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/FindMeCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/FirmwareVersionVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/LatestTelegramVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/MeasurementCurrentConfigVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/MeasurementPersistentConfigVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/MethodCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/MonitoringCaseTableHeaderVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/MonitoringCaseVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/OrderNumberVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/ProjectNameVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/RequiredUserActionVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/SerialNumberVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/StatusOverviewVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/TypeCodeVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/UserNameVariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/cola2/VariableCommand.cpp
	LaserDrivers/Sick_nanoScan3/src/communication/AsyncTCPClient.cpp
	LaserDrivers/Sick_nanoScan3/src/communication/AsyncUDPClient.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseApplicationData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseApplicationNameData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseConfigMetadata.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDataHeader.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDatagramHeader.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDerivedValues.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDeviceName.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseDeviceStatus.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseFieldGeometryData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseFieldHeaderData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseFieldSetsData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseFirmwareVersion.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseGeneralSystemState.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseIntrusionData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseMeasurementCurrentConfigData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseMeasurementData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseMeasurementPersistentConfigData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseMonitoringCaseData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseOrderNumber.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseProjectName.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseRequiredUserAction.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseSerialNumber.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseStatusOverview.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseTCPPacket.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseTypeCodeData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/ParseUserNameData.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/TCPPacketMerger.cpp
	LaserDrivers/Sick_nanoScan3/src/data_processing/UDPPacketMerger.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ApplicationData.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ApplicationInputs.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ApplicationName.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ApplicationOutputs.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/CommSettings.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ConfigData.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ConfigMetadata.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/Data.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/DataHeader.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/DatagramHeader.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/DerivedValues.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/DeviceName.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/DeviceStatus.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/FieldData.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/FieldSets.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/FirmwareVersion.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/GeneralSystemState.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/IntrusionData.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/IntrusionDatum.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/LatestTelegram.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/MeasurementData.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/MonitoringCaseData.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/OrderNumber.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/PacketBuffer.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ParsedPacketBuffer.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ProjectName.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/RequiredUserAction.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/ScanPoint.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/SerialNumber.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/StatusOverview.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/TypeCode.cpp
	LaserDrivers/Sick_nanoScan3/src/datastructure/UserName.cpp
	LaserDrivers/Sick_nanoScan3/src/SickSafetyscannersBase.cpp
	LaserDrivers/BlueSea/sdk/Global.cpp
	LaserDrivers/BlueSea/sdk/LidarDataProcess.cpp
	LaserDrivers/BlueSea/sdk/standard_interface.cpp
	LaserDrivers/BlueSea/sdk/service/LidarCheckService.cpp
	LaserDrivers/BlueSea/sdk/uart.c
	LaserSensor/src/HokuyoLaserScanner.cpp
	LaserSensor/src/PfR2000LaserScanner.cpp
        LaserSensor/src/FRR2LaserScanner.cpp
	LaserSensor/src/RangeScanner.cpp
	LaserSensor/src/SensorFamily.cpp
	LaserSensor/src/Sick581LaserScanner.cpp
	LaserSensor/src/SickSafetyLaserScanner.cpp
        LaserSensor/src/FRR2LaserScanner.cpp
        LaserSensor/src/LeimouF30LaserScanner.cpp
	LaserSensor/src/BlueSeaLaser.cpp
	LaserSensor/src/laser_t.c
	Network/src/Base/Channel.cpp
	Network/src/Base/CSocket.cpp
	Network/src/Base/FrmChan.cpp
	Network/src/Base/TcpChannel.cpp
	Network/src/Base/TcpSock.cpp
	Network/src/Base/TimeOutSock.cpp
	Network/src/Base/UdpChannel.cpp
	Network/src/Base/UdpSock.cpp
	Network/src/Client/AgvUdpSock.cpp
	Network/src/Client/ClntChannel.cpp
	Network/src/Server/LstnSock.cpp
	Network/src/Server/TcpSrvChannel.cpp
	Network/src/Server/TcpSrvCom.cpp
	Network/src/Server/UdpSrvCom.cpp
	Protocol/src/AutoOutPutBlackBox.cpp
	Protocol/src/CPing.cpp
	Protocol/src/ParameterObject.cpp
	Protocol/src/RoboLocClnt.cpp
	Protocol/src/RoboLocProto.cpp
	Mapping/src/BaseOdometry.cpp
	Mapping/src/LaserMapping.cpp
	Mapping/src/LaserOdometry.cpp
	Mapping/src/RawMap.cpp
	Mapping/src/LaserAutoMapping.cpp
	Mapping/src/SendMap.cpp
	Mapping/src/BuildMap.cpp
        Mapping/src/Calibrate.cpp
	LaserSensor/src/WJ716LaserScanner.cpp
	LaserSensor/src/WJ719LaserScanner.cpp
	CanDev/src/CanOpen/rsp/vcil/bus_msg_handler.cpp
	CanDev/src/CanOpen/rsp/vcil/can.cpp
	CanDev/src/CanOpen/rsp/vcil/cmd_handler.cpp
	CanDev/src/CanOpen/rsp/vcil/common.cpp
	CanDev/src/CanOpen/rsp/vcil/DebugLog.cpp
	CanDev/src/CanOpen/rsp/vcil/j1939.cpp
	CanDev/src/CanOpen/rsp/vcil/obd2.cpp
	CanDev/src/CanOpen/rsp/vcil/susi_api_mgr.cpp
	CanDev/src/CanOpen/rsp/vcil/vcil_cmd.cpp
	CanDev/src/CanOpen/rsp/vcil/vcil_ctx.cpp
	CanDev/src/CanOpen/rsp/vcil/vcil_mgr.cpp
	CanDev/src/CanOpen/rsp/vcil/vcil_port.cpp
	CanDev/src/CanOpen/rsp/vcil/vmsg.cpp
	CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/mapping_file_opt.c
	CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_canDev_vcil.c
	CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_clockDev_hpet.c
	CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_dev_stub.c
	CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_feram_mmap.c
	CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_gpio.c
	CanDev/src/CanOpen/rsp/rsp_dev_adv_e3845/rsp_uartDev.c
	)

if(PLATFORM_RK3399)
    add_compile_definitions(_RK3399_ARM_64)
    set(PCL_DIR "/opt/RK3399/pcl_1_9_0/share/pcl-1.9")
    find_package(PCL 1.9 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
endif()


if(PLATFORM_E3845)
    find_package(PCL 1.7 REQUIRED)
    include_directories(${PCL_INCLUDE_DIRS})
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

    set(PCL_DIR "/opt/Mrc3588/pcl/include/pcl-1.11")
    include_directories(${PCL_DIR}
        /opt/Mrc3588/boost/include
        /opt/Mrc3588/pcl/include
        ../../Modules/ThirdParty/yaml-cpp/include

        )

endif()

#Pi
if(PLATFORM_PI)
    add_compile_definitions(_PiCM4_LINUX32)
    add_compile_definitions(NAV_APP)
    add_compile_definitions(_LINUX64)


    set(PCL_DIR "/opt/PI/pcl/include/pcl-1.11")
    include_directories(${PCL_DIR}
        /opt/PI/boost/include
        /opt/PI/pcl/include
         ../../Modules/ThirdParty/yaml-cpp/include
        )

endif()

add_library ( NavBase SHARED ${NavBase_SRCS} )


#MRC3588
if(PLATFORM_MRC3588)
#    target_link_libraries(NavBase  /opt/Mrc3588/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libglib-2.0.so)
    target_link_libraries(NavBase  ${PROJECT_SOURCE_DIR}/Parameters/libCalibrateTool_MRC3588.so)
    target_link_libraries(NavBase  ${PROJECT_SOURCE_DIR}/Parameters/libcamAPI.so)
    target_link_libraries(NavBase  ${PROJECT_SOURCE_DIR}/Parameters/libCamera.so)
    target_link_libraries(NavBase  ${PROJECT_SOURCE_DIR}/Parameters/libopencv_world.so.4.5)
    target_link_libraries(NavBase  ${PROJECT_SOURCE_DIR}/Parameters/libjpeg.so.8)

    target_link_libraries(NavBase  /opt/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libz.so.1.2.11)

#    target_link_libraries(NavBase  /opt/Mrc05Sdk/sysroot/usr/lib/aarch64-linux-gnu/libpng16.so.16)
endif()

#RK3399
if(PLATFORM_RK3399)
    target_link_libraries(NavBase  ${PROJECT_SOURCE_DIR}/Parameters/libCalibrateTool_RK3399.so)
endif()

#PI
if(PLATFORM_PI)
    target_link_libraries(NavBase  ${PROJECT_SOURCE_DIR}/Parameters/libCalibrateTool_PI.so)
endif()

#E3845
if(PLATFORM_E3845)
    target_link_libraries(NavBase  ${PROJECT_SOURCE_DIR}/Parameters/libCalibrateTool_x86.so)
endif()
