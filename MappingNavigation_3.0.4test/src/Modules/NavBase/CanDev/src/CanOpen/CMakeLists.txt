# build the CanOpen library
set(RSP_VCIL_SRCS
	rsp/vcil/bus_msg_handler.cpp
	rsp/vcil/can.cpp
	rsp/vcil/cmd_handler.cpp
	rsp/vcil/common.cpp
	rsp/vcil/DebugLog.cpp
	rsp/vcil/j1939.cpp
	rsp/vcil/obd2.cpp
	rsp/vcil/susi_api_mgr.cpp
	rsp/vcil/vcil_cmd.cpp
	rsp/vcil/vcil_ctx.cpp
	rsp/vcil/vcil_mgr.cpp
	rsp/vcil/vcil_port.cpp
	rsp/vcil/vmsg.cpp
)

set(RSP_E3845_SRCS
	rsp/rsp_dev_adv_e3845/mapping_file_opt.c
	rsp/rsp_dev_adv_e3845/rsp_canDev_vcil.c
	rsp/rsp_dev_adv_e3845/rsp_clockDev_hpet.c
	rsp/rsp_dev_adv_e3845/rsp_dev_stub.c
	rsp/rsp_dev_adv_e3845/rsp_feram_mmap.c
	rsp/rsp_dev_adv_e3845/rsp_gpio.c
	rsp/rsp_dev_adv_e3845/rsp_uartDev.c
)

set(CAN_CHANNEL_SRCS
	SocketCanLib/libsocketcan.c
	CanChannel.cpp
)

set (CANOPEN_LIB_SRCS
	${RSP_VCIL_SRCS}
	${RSP_E3845_SRCS}
	${CAN_CHANNEL_SRCS}
)

add_library(canopen_lib ${CANOPEN_LIB_SRCS})

set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set_target_properties(canopen_lib PROPERTIES OUTPUT_NAME "canopen")

