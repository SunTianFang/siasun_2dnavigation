#ifndef _SUSI_IMC_OBD2_DEF_H
#define _SUSI_IMC_OBD2_DEF_H

#ifndef MAX_OBD2_MESSAGE_BUFFER_SIZE
#define	MAX_OBD2_MESSAGE_BUFFER_SIZE 64
#endif

#define VCIL_OBD2_TYPE_PHYSICAL       218
#define VCIL_OBD2_TAPE_FUNCTIONAL     219

typedef struct
{
    BYTE dst;              /* destination address */
    BYTE src;              /* source address */
    BYTE pri;              /* priority */
    BYTE tat;              /* target address type */
    BYTE buf[MAX_OBD2_MESSAGE_BUFFER_SIZE];
    USHORT buf_len;
} IMC_OBD2_MSG_OBJECT, *PIMC_OBD2_MSG_OBJECT;

#endif
