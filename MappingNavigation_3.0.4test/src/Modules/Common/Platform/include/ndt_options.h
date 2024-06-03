#ifndef __NDT1_OPTIONS
#define __NDT1_OPTIONS

//#define NDT1_USE_MINI_EIGEN    // 如果定义了此宏，则NDT的全部实现都仅使用MiniEigen(不使用Eigen)

#ifndef NDT1_USE_MINI_EIGEN    // 如果使用MiniEigen库，则不启用相关的OMP加速
#define NDT1_USE_OMP           // 是否在NDT的实现中采用OMP并行处理
#define NDT1_THREAD_NUM 2      // OMP并行处理中使用的线程数
#endif

#endif
