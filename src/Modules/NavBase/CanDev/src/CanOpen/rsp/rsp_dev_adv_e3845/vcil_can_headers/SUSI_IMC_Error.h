#ifndef _SUSI_IMC_ERROR_H
#define _SUSI_IMC_ERROR_H

//	Related the CAN BUS API error code set.
#define IMC_CAN_SET_BIT_TIMING_ERROR                                 0xC010
#define IMC_CAN_RX_NOT_READY                                         0xC012
#define IMC_CAN_TX_WRITE_ERR                                         0xC014

//Reserve 0xC100 ~ 0xC10F for Android JNI Error
#define IMC_ERR_ANDROID_JNI_EVENT_INIT_FAILED                        0xC100
#define IMC_ERR_ANDROID_JNI_EVENT_DEINIT_FAILED                      0xC101
#define IMC_ERR_ANDROID_JNI_EVENT_LISTENING_THREAD_CREATE_FAILED     0xC102
#define IMC_ERR_ANDROID_JNI_EVENT_LISTENING_THREAD_ALREADY_RUNNING   0xC103

//Reserve 0xC110 ~ 0xC11F for Android Service Error
#define IMC_ERR_ANDROID_SERVICE_NULL_POINTER                         0xC110
#define IMC_ERR_ANDROID_SERVICE_UNKNWON_EXCEPTION                    0xC111
#define IMC_ERR_ANDROID_SERVICE_REMOTE_CALLBACK_LIST_NOT_FOUND       0xC112
#define IMC_ERR_ANDROID_SERVICE_REMOTE_CALLBACK_REGISTER_FAILED      0xC113
#define IMC_ERR_ANDROID_SERVICE_REMOTE_CALLBACK_UNREGISTER_FAILED    0xC114
#define IMC_ERR_ANDROID_SERVICE_REMOTE_CALLBACK_UPDATE_FAILED        0xC115

//Reserve 0xC120 ~ 0xC12F for Android Service Client Error
#define IMC_ERR_ANDROID_CLIENT_NULL_POINTER                          0xC120
#define IMC_ERR_ANDROID_CLIENT_FAILED_TO_BIND_SERVICE                0xC121
#define IMC_ERR_ANDROID_CLIENT_SERVICE_ALREADY_CONNECTED             0xC122
#define IMC_ERR_ANDROID_CLIENT_SERVICE_DISCONNECTED                  0xC123
#define IMC_ERR_ANDROID_CLIENT_REMOTE_EXCEPTION                      0xC124


#endif	//_SUSI_IMC_ERROR_H
