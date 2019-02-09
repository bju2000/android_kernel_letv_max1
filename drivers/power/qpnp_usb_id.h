#ifndef _QPNP_USB_ID_H_
#define _QPNP_USB_ID_H_

/* Letv define RID state */
enum rid_state {
	RID_GROUND = 0,
	RID_FLOAT,
	RID_MHL,
	RID_UNKNOW,
};
extern int dw3_id_state;

#endif
