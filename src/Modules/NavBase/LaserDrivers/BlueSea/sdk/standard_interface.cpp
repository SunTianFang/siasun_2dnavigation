#include "standard_interface.h"
BlueSeaLidarSDK *BlueSeaLidarSDK::m_sdk = new (std::nothrow) BlueSeaLidarSDK();;

BlueSeaLidarSDK * BlueSeaLidarSDK ::getInstance()
{
	return m_sdk;
}
void BlueSeaLidarSDK::deleteInstance()
{
	if (m_sdk)
    {
        delete m_sdk;
        m_sdk = NULL;
    }
}

BlueSeaLidarSDK::BlueSeaLidarSDK()
{
	m_idx = 0;
	m_checkservice = NULL;
}

BlueSeaLidarSDK::~BlueSeaLidarSDK()
{
}
RunConfig *BlueSeaLidarSDK::getLidar(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
			return m_lidars.at(i);
	}

	return NULL;
}
int BlueSeaLidarSDK::addLidarByPath(const char *cfg_file_name)
{
	RunConfig *cfg=new RunConfig;
	memset(cfg, 0, sizeof(RunConfig));
	if (readConfig(cfg_file_name, cfg->runscript))
	{
		m_idx++;
		cfg->ID = m_idx;
		m_lidars.push_back(cfg);
		return m_idx;
	}
	else
		return 0;
}

int BlueSeaLidarSDK::addUDPLidar(const char *device_name,const int port)
{
    RunConfig *cfg=new RunConfig;
    memset(cfg, 0, sizeof(RunConfig));

    //setLidarParam
//    cfg->runscript.connectArg = device_name;//ip
    // 使用字符串复制函数将device_name的值复制到connectArg中
    strncpy(cfg->runscript.connectArg, device_name, sizeof(cfg->runscript.connectArg) - 1);
    cfg->runscript.connectArg[sizeof(cfg->runscript.connectArg) - 1] = '\0'; // 添加字符串结尾的空字符

    cfg->runscript.connectArg2 = port; //port

    strncpy(cfg->runscript.type, "udp", sizeof(cfg->runscript.type) - 1);
    cfg->runscript.type[sizeof(cfg->runscript.type) - 1] = '\0'; // 添加字符串结尾的空字符

//    cfg->runscript.type = "udp";
    //cfg->runscript.local_port = 6668;


    if(port == 6543)
       {
          cfg->runscript.local_port = 6668;

       }else if(port == 6540)
       {
         cfg->runscript.local_port = 6888;
       }
//    cfg->runscript.local_port = 6668;



    cfg->runscript.is_group_listener = 0;
    strncpy(cfg->runscript.group_ip, "224.0.0.99", sizeof(cfg->runscript.group_ip) - 1);
    cfg->runscript.group_ip[sizeof(cfg->runscript.group_ip) - 1] = '\0'; // 添加字符串结尾的空字符
//    cfg->runscript.group_ip = "224.0.0.99";
    cfg->runscript.output_360 = 1;
    cfg->runscript.from_zero = 0;
    cfg->runscript.service_port = 8888;
    cfg->runscript.is_open_service = 1;
    cfg->runscript.error_circle = 3;
    cfg->runscript.error_scale = 0.9;
    cfg->runscript.uuid = 1;
    cfg->runscript.model = 1;
    cfg->runscript.version = -1;
    cfg->runscript.with_smooth = -1;
    cfg->runscript.with_deshadow = -1;
    cfg->runscript.resample_res = -1;
    cfg->runscript.alarm_msg = -1;
    cfg->runscript.rpm = -1;
    cfg->runscript.direction = -1;
    cfg->runscript.ats = -1;
    cfg->runscript.with_start = -1;



    if (cfg->runscript.error_scale == 0)
        cfg->runscript.error_scale = 0.9;

    if (cfg->runscript.error_circle == 0)
        cfg->runscript.error_circle = 3;

    m_idx++;
    cfg->ID = m_idx;
    m_lidars.push_back(cfg);

    return m_idx;


}

bool BlueSeaLidarSDK::delLidarByID(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			if (m_lidars.at(i)->state == WORK || m_lidars.at(i)->state == WORK_AND_WEB)
			{
				m_lidars.at(i)->state = STOP_ALL;
				sleep(1);
			}
			m_lidars.erase(m_lidars.begin() + i);
			return true;
		}
	}
	return false;
}

void BlueSeaLidarSDK::setCallBackPtr(int ID, printfMsg ptr)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			m_lidars.at(i)->callback = ptr;
		}
	}
}

bool BlueSeaLidarSDK::openDev(int ID)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;
	if (strcmp(lidar->runscript.type, "uart") == 0 || strcmp(lidar->runscript.type, "vpc") == 0)
	{
		int fd = SystemAPI::open_serial_port(lidar->runscript.connectArg, lidar->runscript.connectArg2);
		if (fd <= 0)
		{
			return false;
		}
		lidar->fd = fd;

#ifdef __unix__
		if (pthread_create(&lidar->thread_data, NULL, lidar_thread_proc_uart, lidar) != 0)
			return false;
#elif _WIN32
		if ((int)CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)lidar_thread_proc_uart, lidar, 0, &lidar->thread_data) < 0)
			return false;

#endif

	}
	else if (strcmp(lidar->runscript.type, "udp") == 0)
	{
		int fd = SystemAPI::open_socket_port(lidar->runscript.local_port);
		if (fd <= 0)
		{
			return false;
		}
		lidar->fd = fd;
#ifdef __unix__
		if (pthread_create(&lidar->thread_data, NULL, lidar_thread_proc_udp, lidar) != 0)
			return false;
#elif _WIN32
		if ((int)CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)lidar_thread_proc_udp, lidar, 0, &lidar->thread_data) < 0)
			return false;
#endif
	}
	else
	{
		return false;
	}
	lidar->state = WORK;

	//判定数据线程是否正常运行
	int index = 100;
	while (lidar->action < ONLINE && index > 0)
	{
		msleep(100);
		index--;
	}
	if (lidar->action >= ONLINE)
		return true;


	return false;
}

bool BlueSeaLidarSDK::GetDevInfo(int ID, EEpromV101 *eepromv101)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	lidar->action = GETALLPARAMS;
	int index = 30;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(200);
		index--;
	}

	if (lidar->action == FINISH)
	{
		if(strcmp(lidar->recv_cmd,"OK")==0)
		{
			memcpy(eepromv101, &lidar->eepromv101, sizeof(EEpromV101));
			return true;
		}
	}
	return false;
}
bool BlueSeaLidarSDK::SetDevInfo(RunConfig *lidar, int num, char *cmd,int mode)
{
	lidar->mode = mode;
	lidar->send_len = num;
	memset(lidar->recv_cmd, 0,sizeof(lidar->recv_cmd));
	lidar->recv_len=0;
	strcpy(lidar->send_cmd, cmd);
	lidar->action = SETPARAM;
	int index = 20;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(100);
		index--;
	}
	if (lidar->action == FINISH)
	{
		if (strcmp(lidar->recv_cmd, "OK")==0)
		{
			printf("%s OK\n",lidar->send_cmd);
			return true;
		}
		else if (strcmp(lidar->recv_cmd, "NG")==0)
		{
			printf("%s NG\n",lidar->send_cmd);
			return false;
		}
			
	}
	return false;
}

// 释放连接
void BlueSeaLidarSDK::StopDev(int ID)
{
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			m_lidars.at(i)->action = OFFLINE;
		}
	}
}

const char *BlueSeaLidarSDK::getVersion()
{
	return SDKVERSION;
}



bool BlueSeaLidarSDK::OpenHeartService()
{
	if (m_checkservice == NULL)
	{
		m_checkservice = new LidarCheckService();

	}
	m_checkservice->run();
	return true;;
}

bool BlueSeaLidarSDK::CloseHeartService()
{
	m_checkservice->stop();
	return true;
}


// 启停雷达测距
bool BlueSeaLidarSDK::ControlDrv(int ID, int num, char *cmd)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	lidar->mode = C_PACK;
	lidar->send_len = num;
	strcpy(lidar->send_cmd, cmd);
	lidar->action = CONTROL;

	int index = 30;
	while (lidar->action != FINISH && index > 0)
	{
		msleep(100);
		index--;
	}
	if (lidar->action == FINISH)
	{
		printf("%s OK\n",lidar->send_cmd);
		return true;
	}
	else if(index==0&&strcmp(lidar->send_cmd,"LRESTH")==0)
	{
		printf("%s OK\n",lidar->send_cmd);
		return true;
	}

	return false;
}

bool BlueSeaLidarSDK::ZoneSection(int ID, char section)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	// 判断传入的防区是否合法
	if ((section >= 48 && section <= 57) || (section >= 65 && section <= 70))
	{
		char tmp[12] = {0};
		sprintf(tmp, "LSAZN:%cH", section);
		return SetDevInfo(lidar, strlen(tmp), tmp,S_PACK);
	}

	return false;
}

bool BlueSeaLidarSDK::SetUDP(int ID, char *ip, char *mask, char *gateway, int port)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char result[64] = {0};
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(1, ip, mask, gateway, port, result))
	{
		return false;
	}
	char tmp[64] = {0};
	sprintf(tmp, "LSUDP:%sH", result);
	lidar->mode = S_PACK;
	lidar->send_len = strlen(tmp);
	strcpy(lidar->send_cmd, tmp);
	lidar->action = SETPARAM;
	// 修改ip后没有返回值
	return true;
}

bool BlueSeaLidarSDK::SetDST(int ID, char *ip, int port)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char result[50] = {0};
	// 对传入的格式校验
	if (!BaseAPI::checkAndMerge(0, ip, "", "", port, result))
	{
		return false;
	}
	char tmp[64] = {0};
	sprintf(tmp, "LSDST:%sH", result);
	return SetDevInfo(lidar, strlen(tmp), tmp,S_PACK);
}

bool BlueSeaLidarSDK::SetRPM(int ID, int RPM)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	// 对传入的格式校验
	if (RPM < 300 || RPM > 3000)
	{
		return false;
	}
	char cmd[16] = {0};
	sprintf(cmd, "LSRPM:%dH", RPM);
	return  SetDevInfo(lidar, strlen(cmd), cmd,C_PACK);
	
}

bool BlueSeaLidarSDK::SetTFX(int ID, bool tfx)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSTFX:%dH", tfx);
	return SetDevInfo(lidar, strlen(cmd), cmd,S_PACK);
}

bool BlueSeaLidarSDK::SetDSW(int ID, bool dsw)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LFFF%dH", dsw);
	return SetDevInfo(lidar, strlen(cmd), cmd,C_PACK);
}

bool BlueSeaLidarSDK::SetSMT(int ID, bool smt)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSSS%dH", smt);
	return SetDevInfo(lidar, strlen(cmd), cmd,C_PACK);
}

bool BlueSeaLidarSDK::SetPST(int ID, int mode)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	if (mode <= 0 || mode > 3)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSPST:%dH", mode);
	return SetDevInfo(lidar, strlen(cmd), cmd,S_PACK);
}

bool BlueSeaLidarSDK::SetDID(int ID, unsigned int did)
{
	RunConfig *lidar = NULL;
	for (unsigned int i = 0; i < m_lidars.size(); i++)
	{
		if (m_lidars.at(i)->ID == ID)
		{
			lidar = m_lidars.at(i);
		}
	}
	if (lidar == NULL)
		return false;

	char cmd[16] = {0};
	sprintf(cmd, "LSDID:%dH", did);
	return SetDevInfo(lidar, strlen(cmd), cmd,S_PACK);
}

std::vector<DevConnInfo> BlueSeaLidarSDK::getLidarsList()
{
	return m_checkservice->getLidarsList();
}
