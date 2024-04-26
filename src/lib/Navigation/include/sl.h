#ifndef SL_H
#define SL_H

enum
{
    SLERR_MEM = -1,              /* not enough memory */
    SLERR_NOSCANS = -2,              /* missing scans */
    SLERR_NOTENOUGHPOINTSMATCHED = -3,   /* not enough correspondences found */
    SLERR_LEASTSQUARE = -4,          /* error from least square */
    SLERR_MATRIXINVERSE = -5,    /* error on getting inverse matrix */
    SLERR_NEGATIVEMATRIX = -6,       /* resulting error matrix invalid! */
    SLERR_DETZERO = -7,              /* determinante is zero */
    SLERR_UNRELIABLE = -8,       /* scan match unreliable */
    SLERR_AMBIGOUS = -9,     /* ambigous situation */
};

#define SL_MIN_POINTS_IN_SCAN 6

extern short slMaxIterations;
extern float slEpsD, slEpsA;

#endif
