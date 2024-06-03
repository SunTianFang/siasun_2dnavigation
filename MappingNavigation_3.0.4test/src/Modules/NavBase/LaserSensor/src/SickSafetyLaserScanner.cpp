//                           - PfR2000LaserScanner.cpp -
//
//   The interface of class "cHokuyoLaserScanner".
//
//   Author: sfe1012
//   Date:   2019. 01. 28
//


#include"SickSafetyLaserScanner.h"
#include"LinuxSetting.h"
#include"Project.h"
#include"blackboxhelper.hpp"

#ifdef USE_BLACK_BOX
extern CBlackBox LocBox;
#endif


namespace sick {

cSickSafetyLaserScanner::cSickSafetyLaserScanner(float fAngRes, float fStartAng, float fEndAng):
    CRangeScanner(fAngRes, fStartAng, fEndAng, SICK)
//  : m_time_offset(0.0)
//  , m_range_min(0.0)
//  , m_range_max(0.0)
//  , m_angle_offset(-90.0)
//  , m_use_pers_conf(false)
{
    m_time_offset = 0.0f;
    m_range_min = m_range_max = 0.0f;
    m_angle_offset = -90.0f;
    m_use_pers_conf = false;
    m_nDequeCount = 0;
    //readParameters();
    m_bStarted = false;
}

bool cSickSafetyLaserScanner::readParameters()
{
   // return true;
    //load form LaserParm.jason
 //std::string sensor_ip_adress = "192.168.1.102";
//  if (!m_private_nh.getParam("sensor_ip", sensor_ip_adress))
//  {
//    //    sensor_ip_adress = sick_safetyscanners::SickSafetyscannersConfiguration_sensor_ip;
//    ROS_WARN("Using default sensor IP: %s", sensor_ip_adress.c_str());
//  }
// m_communication_settings.setSensorIp(sensor_ip_adress);


 // std::string host_ip_adress = "192.168.1.101";
//  if (!m_private_nh.getParam("host_ip", host_ip_adress))
//  {
//    ROS_WARN("Using default host IP: %s", host_ip_adress.c_str());
//  }
 // m_communication_settings.setHostIp(host_ip_adress);

 // int host_udp_port = 6060;
//  if (!m_private_nh.getParam("host_udp_port", host_udp_port))
//  {
//    ROS_WARN("Using default host UDP Port: %i", host_udp_port);
//  }
  //m_communication_settings.setHostUdpPort(host_udp_port);

//  ROS_WARN("If not further specified the default values for the dynamic reconfigurable parameters "
//           "will be loaded.");


//  int channel = 0;
//  m_private_nh.getParam("channel", channel);
 // m_communication_settings.setChannel(channel);

//  bool enabled;
//  m_private_nh.getParam("channel_enabled", enabled);
 // m_communication_settings.setEnabled(enabled);

  int skip = 0;
//  m_private_nh.getParam("skip", skip);
  m_communication_settings.setPublishingFrequency(skip + 1);

// float angle_start = 0.0f;
//  m_private_nh.getParam("angle_start", angle_start);

 // float angle_end = 0.0f;
//  m_private_nh.getParam("angle_end", angle_end);

//  // Included check before calculations to prevent rounding errors while calculating
//  if (angle_start == angle_end)
//  {
//    m_communication_settings.setStartAngle(sick::radToDeg(0));
//    m_communication_settings.setEndAngle(sick::radToDeg(0));
//  }
//  else
//  {
//    m_communication_settings.setStartAngle(sick::radToDeg(angle_start) - m_angle_offset);
//    m_communication_settings.setEndAngle(sick::radToDeg(angle_end) - m_angle_offset);
//  }

//  bool general_system_state;
//  m_private_nh.getParam("general_system_state", general_system_state);

//  bool derived_settings;
//  m_private_nh.getParam("derived_settings", derived_settings);

//  bool measurement_data;
//  m_private_nh.getParam("measurement_data", measurement_data);

//  bool intrusion_data;
//  m_private_nh.getParam("intrusion_data", intrusion_data);

//  bool application_io_data;
//  m_private_nh.getParam("application_io_data", application_io_data);

//  m_communication_settings.setFeatures(
//    general_system_state, derived_settings, measurement_data, intrusion_data, application_io_data);

//  m_private_nh.getParam("frame_id", m_frame_id);

//  m_private_nh.getParam("use_persistent_config", m_use_pers_conf);

//  return true;
//}
}

void cSickSafetyLaserScanner::receivedUDPPacket(const sick::datastructure::Data& data)
{
    if (!m_bStarted){
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "-->RecvUDPPacket(), Stop!");
#endif
        return;
    }

    if(!data.getMeasurementDataPtr() || !data.getDerivedValuesPtr()){
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "-->RecvUDPPacket(), Ptr is null!");
#endif
        return;
    }

    if (!data.getMeasurementDataPtr()->isEmpty() && !data.getDerivedValuesPtr()->isEmpty())
    {
        vector<float> distence;
        vector<float> intensity;

        uint32_t num_scan_points1 = data.getDerivedValuesPtr()->getNumberOfBeams();
        std::vector<sick::datastructure::ScanPoint> scan_points =
                data.getMeasurementDataPtr()->getScanPointsVector();
        uint32_t num_scan_points2 = static_cast<uint32_t>(scan_points.size());
        uint32_t num_scan_points3 = data.getMeasurementDataPtr()->getNumberOfBeams();

        uint32_t max_beams_cnt = 3600;
        uint32_t num_scan_points = 0;
        //TODO: 需要确认:　num_scan_points1,num_scan_points2,num_scan_points3　正常情况下这三个数值是否相等？
        if(num_scan_points1 != num_scan_points2 || num_scan_points3 != num_scan_points2){
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "-->RecvUDPPacket(),scan points num is not match! ", (int)num_scan_points1,
                          ",", (int)num_scan_points2, ",", (int)num_scan_points3);
#endif
            return;
        }
        num_scan_points = num_scan_points1;
        if(num_scan_points > max_beams_cnt || num_scan_points == 0){
#ifdef USE_BLACK_BOX
            FILE_BlackBox(LocBox, "-->RecvUDPPacket(),scan points num is err! ", (int)num_scan_points);
#endif
            return;
        }

        distence.reserve(num_scan_points);
        intensity.reserve(num_scan_points);
       // std::cout <<"num: "<< num_scan_points << std::endl;

        for (uint32_t i = 0; i < num_scan_points; ++i)
        {
            const sick::datastructure::ScanPoint scan_point = scan_points.at(i);
            float a = 0.0, b = 0.0;

            a = static_cast<float>(scan_point.getDistance()) *
                    data.getDerivedValuesPtr()->getMultiplicationFactor(); // mm -> m
            b = static_cast<float>(scan_point.getReflectivity());
            if(scan_point.getReflectorBit())
                b = 255.0;

            //std::cout <<"i: "<< i << " nDist: " << a <<  ", " << "nIntensity: " << b << std::endl;

            distence.push_back(a);
            intensity.push_back(b);
        }

        unsigned long long raw_time = GetTickCount();
        AddRawPointCloud(distence, intensity, raw_time, raw_time);
    }
}

void cSickSafetyLaserScanner::readTypeCodeSettings()
{
  //ROS_INFO("Reading Type code settings");
  sick::datastructure::TypeCode type_code;
  m_device->requestTypeCode(m_communication_settings, type_code);
  m_communication_settings.setEInterfaceType(type_code.getInterfaceType());
  m_range_min = 0.1;
  m_range_max = type_code.getMaxRange();
}

void cSickSafetyLaserScanner::readPersistentConfig()
{
  //ROS_INFO("Reading Persistent Configuration");
  sick::datastructure::ConfigData config_data;
  m_device->requestPersistentConfig(m_communication_settings, config_data);
  m_communication_settings.setStartAngle(config_data.getStartAngle());
  m_communication_settings.setEndAngle(config_data.getEndAngle());
}

BOOL cSickSafetyLaserScanner::Start(const char*device_name,
                                    const char*host_name,
                                    const int laserid,
                                    const int iPort,
                                    const int iNetType,
                                    const int iScanFrequency,
                                    const int iSamplesPerScan)
{
    if (m_bStarted){
        return true;
    }
    m_LaserScannerIp = device_name;
    m_nLaserId = laserid;
    m_nFrequency = iScanFrequency;
    m_nConnectTime = GetTickCount();

    std::string device_ip_adress = device_name;
    m_communication_settings.setSensorIp(device_ip_adress);
    std::string host_ip_adress = host_name;
    m_communication_settings.setHostIp(host_ip_adress);

    m_communication_settings.setPublishingFrequency(1);

    std::cout << "start nano3" << std::endl;
    Start();

    m_hKillThread = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hKillThread != NULL);
    m_hThreadDead = CreateEvent(NULL, FALSE, FALSE, NULL);
    ASSERT(m_hThreadDead != NULL);
}

bool cSickSafetyLaserScanner::Start()
{
    // tcp port can not be changed in the sensor configuration, therefore it is hardcoded
    m_communication_settings.setSensorTcpPort(2122);

    m_device = std::make_shared<sick::SickSafetyscannersBase>(
      boost::bind(&cSickSafetyLaserScanner::receivedUDPPacket, this, _1), &m_communication_settings);

    if(!m_device){
#ifdef USE_BLACK_BOX
        FILE_BlackBox(LocBox, "-->SickScanner::Start(), m_device is null!");
#endif
        return false;
    }
    m_device->run();
//    m_get_raw_data_thread_ptr.reset(receivedUDPPacket());
//    m_get_raw_data_thread_ptr.join();
//    readTypeCodeSettings();

//    if (m_use_pers_conf)
//    {
//      readPersistentConfig();
//    }

//    m_device->changeSensorSettings(m_communication_settings);
    m_bStarted = true;
    readTypeCodeSettings();
    m_device->changeSensorSettings(m_communication_settings);

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "-->SickScanner::Start(), m_bStarted = ", (int)m_bStarted);
#endif

    return m_bStarted;
}

bool cSickSafetyLaserScanner::Stop()
{
    if (!m_bStarted){
        return true;
    }

    if(m_device){
        m_device.reset();
        m_device = nullptr;
    }

    m_bStarted = false;

#ifdef USE_BLACK_BOX
    FILE_BlackBox(LocBox, "-->SickScanner::Stop(), m_bStarted = ", (int)m_bStarted);
#endif

    return true;
}

}//namespace sick
