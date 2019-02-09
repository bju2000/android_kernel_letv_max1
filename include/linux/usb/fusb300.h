#ifndef __LINUX_FUSB300_H
#define __LINUX_FUSB300_H

extern int fusb300_set_msm_usb_host_mode(bool on);
extern unsigned long fusb300_get_checktime(void);

enum fusb_work_mode {
	MD_UNKNOW = 0,
	MD_UFP,
	MD_DFP,
	MD_HALT,
	MD_LPW,
};

enum ccpin_state {
	CC_UNKNOW = 0,
	RA = 0x1,
	RD = 0x10,
	OPEN = 0x100,
};

void set_fusb300_mode(enum fusb_work_mode fusb_mode);
#endif
