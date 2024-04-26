//                     - NETPORT.H - 
//
//   Defines the well-known ports used by the AGV software.
//
//   Programmer: Zhanglei
//   Date:       2006. 11. 10
//   

#ifndef __NET_PORT
#define __NET_PORT
#ifdef AGV_LINUX_DEBUG
// On AGV client side
#define AGV_TCP_PORT                     2001     // AGV TCP client port
#define AGV_VMI_PORT                     2002     // AGV TCP master port (VMI)

// On master side
#define MASTER_TCP_PORT                  2000     // Master TCP port
#define MASTER_UDP_PORT                  2000     // Master UDP port

// On maintainence software side
#define MAINTAIN_UDP_PORT                12345     // Maintainence software UDP port

#define LW_UDP_PORT                      2004     // Layout Wizard UDP port

#define RL_CLIENT_UDP_PORT               10112 //8101    // 定位控制器客户端的端口号
#define RL_SERVER_UDP_PORT               1001/*3001*//*10111*/    // 定位控制器服务器端的端口号

#else

// On AGV client side
#define AGV_TCP_PORT                     1001     // AGV TCP client port
#define AGV_VMI_PORT                     1002     // AGV TCP master port (VMI)

// On master side
#define MASTER_TCP_PORT                  1000     // Master TCP port
#define MASTER_UDP_PORT                  1000     // Master UDP port

// On maintainence software side
#define MAINTAIN_UDP_PORT                12345     // Maintainence software UDP port

#define LW_UDP_PORT                      1004     // Layout Wizard UDP port

#define RL_CLIENT_UDP_PORT               10112 //8101    // 定位控制器客户端的端口号
#define RL_SERVER_UDP_PORT               1001/*3001*//*10111*/    // 定位控制器服务器端的端口号,桌面版本端口号位2001,MRC04控制器版本端口号位1001,PLC控制器版本端口号为10111.

#endif
#endif
