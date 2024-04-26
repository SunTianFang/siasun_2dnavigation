//                           - CANGYRO.CPP -
//
//   Programmer: Ma xifeng
//   Date:       2012.12.15
//

#include "stdafx.h"
#include "CanGyro.h"
#include "Project.h"
#include "Tools.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define GYRO_CODE_TX 0x02
#define GYRO_CODE_RX 0x00

#define CAN_GYRO_TIMEOUT        500 //500ms
#define MAX_GYRO_BUFF_COUNT     500
#define GYRO_DATA_VALID_TIME    300 //300ms


float SIASUN_LIB_DECL_EXPORT fRefAngle = 0.0f;
int SIASUN_LIB_DECL_EXPORT nGyroAngleJump = 0;

//   Can ID:
//   0x132   gyro to vcu
//   0x32    vcu  to gyro

//////////////////////////////////////////////////////////////////////////////

//
//   Default constructor.
//
CCanGyroUnit::CCanGyroUnit()
{
	m_dwLastRefresh = GetTickCount();
	m_pCan = NULL;
	m_uCanID = 0xFFFF;
   m_nUnitID = -1;

   m_bLostPowerInSleep = TRUE;
	m_bInSleep = FALSE;

	m_bDataUpdated = FALSE;

	m_lJumpCount = 0;

	m_fSpinProgReal  = 0.0f;
	m_fSpinVelReal  = 0.0f;
	m_fLastSpinAngle  = 0.0f;

    gyro_buff_data.clear();
    gyro_buff_count = 0;
}

// The constructor
CCanGyroUnit::CCanGyroUnit(USHORT uCanID, int nUnitID, BOOL bLostPowerInSleep)
{
	m_dwLastRefresh = GetTickCount();
   m_pCan = NULL;
   m_uCanID = uCanID;
   m_nUnitID = nUnitID;

   m_Angle.Update(0);

   m_bLostPowerInSleep = bLostPowerInSleep;
   m_bInSleep = FALSE;

   m_bDataUpdated = FALSE;

   m_lJumpCount = 0;

   gyro_buff_data.clear();
   gyro_buff_count = 0;
}

//
//   Process the received packet.
//
void CCanGyroUnit::ProcessReceivedMsg(CCanMsg& Packet)
{
	float* f = (float*)(&Packet.m_uchData[0]);
	float fAngle = (float) *f;

	m_CritSection.Lock();
	if (m_bDataUpdated)
	{
		/*m_lJumpCount++;
		if (m_lJumpCount > 30000)
		{
			m_lJumpCount = 0;
		}*/

		float fAngleDif = FabsAngleDiff(fAngle, m_Angle.m_fData);

		if (fAngleDif > MAX_ANGLE_DIFFERENCE)
		{
			nGyroAngleJump++;
		}
		else
		{
			m_Angle.Update(fAngle);
            UpdateGyroData(m_Angle);

            //cout<<"fangle= "<<fAngle<<endl;
            if(!m_bSpinSuspend)
            {
                float fDiff = FabsAngleDiff(fAngle, m_fLastSpinAngle);
                m_fSpinProgReal += fDiff;
                m_fSpinVelReal = fDiff / 0.01f;
            }

			m_fLastSpinAngle = fAngle;
		}
	}
	else
	{
		m_Angle.Update(fAngle);
        UpdateGyroData(m_Angle);
		m_bDataUpdated = TRUE;
	}
	m_CritSection.Unlock();

	fRefAngle = fAngle;  //added by gaohd at 2016.06.02 for Debug

	BOOL bInCompensate = FALSE;
	bInCompensate = ((Packet.m_uchData[6] & 0x01) != 0);

	m_CritSection.Lock();
	m_InCompensate.Update(bInCompensate);
	m_CritSection.Unlock();
    unsigned char* b = (unsigned char*)(&Packet.m_uchData[6]);
    unsigned char bStatus = (unsigned char) *b;
	m_CritSection.Lock();
	m_Status.Update(bStatus);
	m_CritSection.Unlock();

	m_dwLastRefresh = GetTickCount();
}

//
//Suspend spin angle
//
BOOL CCanGyroUnit::SuspendSpinAngle(BOOL bSuspend)
{
    m_bSpinSuspend = bSuspend;
    return TRUE;
}

//
//   Read the guide error.
//
BOOL CCanGyroUnit::ReadAngle(CFloatDataInfo& Angle)
{

	BOOL bResult = FALSE;
	m_CritSection.Lock();

	if (m_Angle.IsNew(CAN_GYRO_TIMEOUT))
	{
		Angle = m_Angle;
		bResult = TRUE;
	}

	m_CritSection.Unlock();

	return bResult;
}

BOOL CCanGyroUnit::ReadSpinProgAngle(float& fAngle)
{
	BOOL bResult = FALSE;
	m_CritSection.Lock();

	if (m_Angle.IsNew(CAN_GYRO_TIMEOUT))
	{
		fAngle = m_fSpinProgReal;
		bResult = TRUE;
	}

	m_CritSection.Unlock();

	return bResult;
}

BOOL CCanGyroUnit::ReadSpinVel(float& fVel)
{
	BOOL bResult = FALSE;
	m_CritSection.Lock();

	if (m_Angle.IsNew(CAN_GYRO_TIMEOUT))
	{
		fVel = m_fSpinVelReal;
		bResult = TRUE;
	}

	m_CritSection.Unlock();

	return bResult;
}

BOOL CCanGyroUnit::ReadStatus(unsigned char& bStatus)
{

	BOOL bResult = FALSE;
	m_CritSection.Lock();

	if (m_Status.IsNew(CAN_GYRO_TIMEOUT))
	{
		bStatus = m_Status.m_bData;
		bResult = TRUE;
	}

	m_CritSection.Unlock();

	return bResult;
}
int CCanGyroUnit::InCompensate()
{
	int nResult = -1;
	m_CritSection.Lock();

	if (m_InCompensate.IsNew(CAN_GYRO_TIMEOUT))
	{
		nResult = (int)m_InCompensate.m_bData;
	}

	m_CritSection.Unlock();

	return nResult;
}

BOOL CCanGyroUnit::RefreshTimeOut(DWORD dwTimeOut)
{
	return (GetTickCount() - m_dwLastRefresh > dwTimeOut);
}

void CCanGyroUnit::SetSleepMode(BOOL bInSleep)
{
	m_bInSleep = bInSleep;
}

BOOL CCanGyroUnit::SetSpinInitAngle()
{
	BOOL bResult = FALSE;
	m_CritSection.Lock();

	if (m_Angle.IsNew(CAN_GYRO_TIMEOUT))
	{
		m_fLastSpinAngle = m_Angle.m_fData;
		bResult = TRUE;

		m_fSpinProgReal = 0.0f;
		m_fSpinVelReal = 0.0f;
	}

	m_CritSection.Unlock();

	return bResult;
}

void CCanGyroUnit::DriftCompensate(BOOL bStart)
{
	if (m_pCan == NULL)
		return;

	if (m_bLostPowerInSleep && m_bInSleep)
		return;

	CCanMsg Packet = {m_uCanID, 0, 8};

	if (bStart)
		Packet.m_uchData[0] = 1;
	else
		Packet.m_uchData[0] = 0;

	m_pCan->SendMsg(&Packet);
}

bool CCanGyroUnit::UpdateGyroData(CFloatDataInfo& gyro_data)
{
    {
        std::lock_guard<std::mutex> lock(gyro_buff_mtx);
        if(!gyro_buff_data.empty()) {
            if(gyro_buff_data.back().m_dwTimeStamp >= gyro_data.m_dwTimeStamp) {
                return false;
            }
        }

        if(gyro_buff_count >= MAX_GYRO_BUFF_COUNT){
            gyro_buff_data.pop_front();
            gyro_buff_data.push_back(gyro_data);
            gyro_buff_count = MAX_GYRO_BUFF_COUNT;
        }
        else{
            gyro_buff_data.push_back(gyro_data);
            gyro_buff_count += 1;
        }
    }
    return true;
}

bool CCanGyroUnit::GetSyncData(unsigned long long sync_time, CFloatDataInfo& sync_data)
{
    CFloatDataInfo front_data;
    CFloatDataInfo back_data;
    sync_data.clear();
    front_data.clear();
    back_data.clear();
    if(gyro_buff_data.empty()) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(gyro_buff_mtx);

        // time out
        if(!gyro_buff_data.back().IsNew(CAN_GYRO_TIMEOUT)){
            return false;
        }

        if(sync_time >= gyro_buff_data.back().m_dwTimeStamp) {
            sync_data = gyro_buff_data.back();
            if((sync_time - sync_data.m_dwTimeStamp) < GYRO_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        if(sync_time <= gyro_buff_data.front().m_dwTimeStamp) {
            sync_data = gyro_buff_data.front();
            if((sync_data.m_dwTimeStamp - sync_time) < GYRO_DATA_VALID_TIME) {
                return true;
            }
            else {
                return false;
            }
        }

        // 陀螺仪数据按时间序列排列，在陀螺仪数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做插值
        long lSize = static_cast<long>(gyro_buff_data.size());
        for (long i = lSize - 1; i >= 0; --i){
            if(sync_time < gyro_buff_data[i].m_dwTimeStamp){
                continue;
            }
            if(sync_time == gyro_buff_data[i].m_dwTimeStamp){
                sync_data = gyro_buff_data[i];
                return true;
            }
            if((sync_time - gyro_buff_data[i].m_dwTimeStamp) > GYRO_DATA_VALID_TIME){
                return false;
            }

            if((i + 1) < lSize){
                if(sync_time > gyro_buff_data[i + 1].m_dwTimeStamp
                        || (gyro_buff_data[i + 1].m_dwTimeStamp - sync_time) > GYRO_DATA_VALID_TIME){
                    return false;
                }
                front_data = gyro_buff_data[i + 1];
                back_data = gyro_buff_data[i];
                break;
            }
        }
    }

    float ratioFront = 0.0;
    float ratioBack = 0.0;
    if(front_data.m_dwTimeStamp <= back_data.m_dwTimeStamp || sync_time < back_data.m_dwTimeStamp
            || sync_time > front_data.m_dwTimeStamp) {
        ratioFront = 1.0;
        ratioBack = 0.0;
    }
    else {
        ratioFront = static_cast<float>(sync_time - back_data.m_dwTimeStamp) / static_cast<float>(front_data.m_dwTimeStamp - back_data.m_dwTimeStamp);
        ratioBack = static_cast<float>(front_data.m_dwTimeStamp - sync_time) / static_cast<float>(front_data.m_dwTimeStamp - back_data.m_dwTimeStamp);
    }

    if(ratioFront > ratioBack){
        sync_data.m_fData = front_data.m_fData;
        sync_data.m_dwTimeStamp = front_data.m_dwTimeStamp;
    }
    else {
        sync_data.m_fData = back_data.m_fData;
        sync_data.m_dwTimeStamp = back_data.m_dwTimeStamp;
    }
    // 使用线性插值,角度需要归一化
    /*if(fabs(front_data.m_fData - back_data.m_fData) > 1000.0 * PI){
        if(ratioFront > ratioBack){
            sync_data.m_fData = front_data.m_fData;
        }
        else {
            sync_data.m_fData = back_data.m_fData;
        }
    }
    else{
        while ((front_data.m_fData - back_data.m_fData) >= PI){
            back_data.m_fData += 2*PI;
        }
        while((front_data.m_fData - back_data.m_fData) < -PI){
            back_data.m_fData -= 2*PI;
        }
        sync_data.m_fData = front_data.m_fData * ratioFront + back_data.m_fData * ratioBack;
    }*/

    return true;
}

///////////////////////////////////////////////////////////////////////////////
//   Implementation of class "CCanGyro".

CCanGyro::CCanGyro()
{
	m_nUnitCount = 0;
	for (int i = 0; i < 2; i++)
		m_pUnit[i] = NULL;
}

CCanGyro::~CCanGyro()
{
	for (int i = 0; i < m_nUnitCount; i++)
		if (m_pUnit[i] != NULL)
			delete m_pUnit[i];
}

//
//   Create the CCanGyro object.
//
BOOL CCanGyro::Init(CCanChannel* pCan)
{
	ASSERT(pCan != NULL);
	if (pCan == NULL)
		return FALSE;

	m_pCan = pCan;

	for (int i = 0; i < m_nUnitCount; i++)
		m_pUnit[i]->SetCanChannel(pCan);
	return TRUE;
}

//
//   Start the device.
//
void CCanGyro::StartDevice()
{
	return;
}

//
//   Stop the device.
//
void CCanGyro::StopDevice()
{
	return;
}

void CCanGyro::RefreshInputs()
{
	return;
}

//
//   Add a new CAN-OPEN unit to the system.
//
BOOL CCanGyro::AddUnit(BOOL bLostPowerInSleep, USHORT uCanID, int nUnitID)
{
	// Allocate memory for the new CANOpen unit
	CCanGyroUnit* pUnit = new CCanGyroUnit(uCanID, nUnitID, bLostPowerInSleep);
	if (pUnit == NULL)
		return FALSE;

	m_pUnit[m_nUnitCount++] = pUnit;
	return TRUE;
}

//
//   Delete a CAN-OPEN unit from the system.
//
BOOL CCanGyro::DelUnit(int nUnitID)
{
	if (m_nUnitCount == 0 || nUnitID >= m_nUnitCount)
		return FALSE;

   delete m_pUnit[nUnitID];

   for (USHORT i = nUnitID; i < m_nUnitCount-1; i++)
		m_pUnit[i] = m_pUnit[i+1];

	m_nUnitCount--;
	return TRUE;
}

//
//   Check whether the received packet is a CAN-OPEN message.
//
BOOL CCanGyro::CheckPacket(CCanMsg& Packet)
{
	USHORT uFunCode = Packet.id >> 7;     // The higher 4 bits is function code
   USHORT uCanID = Packet.id & 0x7F;
	if ((uFunCode == GYRO_CODE_TX) && 
       (uCanID >= CAN_GYRO_MIN_ID) && (uCanID <= CAN_GYRO_MAX_ID))
		return TRUE;
	else
		return FALSE;
}

//
//   Process the received CAN-GUIDE message.
//
void CCanGyro::ProcessReceivedMsg(CCanMsg& Packet)
{
	USHORT uFunCode = Packet.id >> 7;     // The higher 4 bits is function code
	USHORT uCanID = Packet.id & 0x7F;     // The lower 7 bits is CAN-ID

	int nUnitID = -1;
	for (int i = 0; i < m_nUnitCount; i++)
	{
		if (m_pUnit[i]->m_uCanID == uCanID)
		{
			nUnitID = i;
			break;
		}
	}

   ASSERT(nUnitID != -1);
	if (nUnitID == -1)
      return;

   m_pUnit[nUnitID]->ProcessReceivedMsg(Packet);
}

//
//   Read the deviation from the guide path.
//
BOOL CCanGyro::ReadAngle(int nUnitID, CFloatDataInfo& Angle)
{
	return m_pUnit[nUnitID]->ReadAngle(Angle);
}

bool CCanGyro::GetSyncAngle(int nUnitID, unsigned long long sync_time, CFloatDataInfo& sync_angle)
{
    return m_pUnit[nUnitID]->GetSyncData(sync_time, sync_angle);
}

BOOL CCanGyro::SuspendSpinAngle(int nUnitID, BOOL bSuspend)
{
    return m_pUnit[nUnitID]->SuspendSpinAngle(bSuspend);
}


BOOL CCanGyro::ReadSpinProgAngle(int nUnitID, float& fAngle)
{
	return m_pUnit[nUnitID]->ReadSpinProgAngle(fAngle);
}

BOOL CCanGyro::ReadSpinVel(int nUnitID, float& fVel)
{
	return m_pUnit[nUnitID]->ReadSpinVel(fVel);
}

BOOL CCanGyro::ReadStatus(int nUnitID, unsigned char& bStatus)
{
	return m_pUnit[nUnitID]->ReadStatus(bStatus);
}
void CCanGyro::DriftCompensate(int nUnitID, BOOL bStart)
{
	return m_pUnit[nUnitID]->DriftCompensate(bStart);
}

BOOL CCanGyro::SetSpinInitAngle(int nUnitID)
{
	return m_pUnit[nUnitID]->SetSpinInitAngle();
}

int CCanGyro::InCompensate(int nUnitID)
{
	return m_pUnit[nUnitID]->InCompensate();
}

USHORT CCanGyro::RefreshTimeOut(DWORD dwTimeOut)
{
	for (int i = 0; i < m_nUnitCount; i++)
		if (m_pUnit[i]->RefreshTimeOut(dwTimeOut))
			return i;

	return 0xFFFF;
}

void CCanGyro::SetSleepMode(BOOL bInSleep)
{
	for (int i = 0; i < m_nUnitCount; i++)
		m_pUnit[i]->SetSleepMode(bInSleep);
}

BOOL CCanGyro::InitCanGyro(int nCanId, int nUnitID, bool bLostPowerInSleep)
{
    //int nCount, nUnitID;
   // USHORT uCanID;
   // BOOL bLostPowerInSleep;
    for (int j = 0; j < m_nUnitCount; j++)
        if (m_pUnit[j] != NULL)
            delete m_pUnit[j];
     m_nUnitCount = 0;
    int nCount;
    nCount = 1 ;
    for (int i = 0; i < nCount; i++)
    {
//        uCanID = 70;
//        nUnitID = 0;
//        bLostPowerInSleep = 1;
        VERIFY(AddUnit(bLostPowerInSleep, nCanId, nUnitID));
    }
    return TRUE;
}

//CArchive& operator >> (CArchive& ar, CCanGyro& Obj)
//{
//	int nCount, nUnitID;
//	USHORT uCanID;
//	BOOL bLostPowerInSleep;

//	for (int j = 0; j < Obj.m_nUnitCount; j++)
//		if (Obj.m_pUnit[j] != NULL)
//			delete Obj.m_pUnit[j];
//   Obj.m_nUnitCount = 0;

//	ar >> nCount;
//	for (int i = 0; i < nCount; i++)
//	{
//		ar >> uCanID >> nUnitID >> bLostPowerInSleep;
//		VERIFY(Obj.AddUnit(bLostPowerInSleep, uCanID, nUnitID));
//	}

//	return ar;
//}

//CArchive& operator << (CArchive& ar, CCanGyro& Obj)
//{
//	ar << Obj.m_nUnitCount;
//	for (int i = 0; i < Obj.m_nUnitCount; i++)
//	{
//		CCanGyroUnit* pUnit = Obj.m_pUnit[i];
//		ar << pUnit->m_uCanID << pUnit->m_nUnitID << pUnit->m_bLostPowerInSleep;
//	}

//	return ar;
//}
