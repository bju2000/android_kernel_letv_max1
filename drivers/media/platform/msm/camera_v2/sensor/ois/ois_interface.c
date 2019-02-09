#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <media/msm_cam_sensor.h>
#include "ois_interface.h"
#include "OisDef.h"
#include "Ois.h"
/* ADD-E: add new ois for ofilm module, compatible with sharp module */
#include "OisDef_OFILM.h"
#include "Ois_OFILM.h"
/* ADD-S: add new ois for ofilm module, compatible with sharp module */

#define OISDBG(fmt, args...) pr_err(fmt, ##args)

#define LC898122_CHECK_RIGISTER 0x027F
#define LC898122_CHECK_DATA 0xAC
#define INIT_FAILED -1
#define INIT_SUCCESS 1

static struct msm_camera_i2c_client *g_i2c_clinet;
/*  write interface for AF  OIS*/
void RegWriteA(unsigned short RegAddr, unsigned char RegData)
{
	int32_t  rc = 0;
	uint16_t w_data;
	uint32_t w_addr;
	w_addr = (RegAddr & 0xFFFF);
	w_data = (RegData & 0x00FF);
	rc = msm_camera_cci_i2c_write(g_i2c_clinet,
			w_addr, w_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_err("%s: %x: RegWriteA    failed\n", __func__, RegAddr);
}

void RegReadA(unsigned short RegAddr, unsigned char *RegData)
{
	int32_t rc = 0;
	uint16_t r_data;
	uint32_t w_addr;
	w_addr = (RegAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read(g_i2c_clinet,
			w_addr, &r_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_err("%s: %x: RegReadA read  failed\n", __func__, RegAddr);
	*RegData = (r_data & 0x00FF);
}

void RamWriteA(unsigned short RameAddr, unsigned short RamData)
{
	int32_t rc = 0;
	uint16_t w_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	w_data = (RamData & 0xFFFF);
	rc = msm_camera_cci_i2c_write(g_i2c_clinet,
			w_addr, w_data,
			MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0)
		pr_err("%s: %x: RamWriteA  write failed\n", __func__, RameAddr);
}

void RamReadA(unsigned short RameAddr, unsigned short *RamData)
{
	int32_t rc = 0;
	uint16_t r_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read(g_i2c_clinet,
			w_addr, &r_data,
			MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0)
		pr_err("%s: %x: RamReadA read  failed\n", __func__, RameAddr);
	*RamData = (r_data & 0xFFFF);
}

void RamWrite32A(unsigned short RameAddr, unsigned long RamData)
{
	int32_t  rc = 0;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	uint32_t w_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	w_data = (RamData & 0xFFFFFFFF);
	reg_setting.reg_addr = w_addr;
	reg_setting.reg_data[0] = (uint8_t)((w_data & 0xFF000000) >> 24);
	reg_setting.reg_data[1] = (uint8_t)((w_data & 0x00FF0000) >> 16);
	reg_setting.reg_data[2] = (uint8_t)((w_data & 0x0000FF00) >> 8);
	reg_setting.reg_data[3] = (uint8_t)(w_data & 0x000000FF);
	reg_setting.reg_data_size = 4;
	rc = msm_camera_cci_i2c_write_seq(g_i2c_clinet,
		reg_setting.reg_addr,
		reg_setting.reg_data,
		reg_setting.reg_data_size);

	if (rc < 0)
		pr_err("%s: %x: RamWrite32A write failed\n",
			__func__, RameAddr);
}

void RamRead32A(unsigned short RameAddr, unsigned long *RamData)
{

	int32_t rc = 0;
	uint32_t t_data;
	uint32_t w_addr;
	uint8_t r_data[5];
	w_addr = (RameAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read_seq(g_i2c_clinet,
		w_addr, r_data, 4);
	if (rc < 0)
		pr_err(" %s: %x: RamRead32A RameAddr read failed\n",
			__func__, RameAddr);
	t_data = (r_data[0]<<24)|(r_data[1]<<16)|(r_data[2]<<8)|r_data[3];
	*RamData = (t_data & 0xFFFFFFFF);
}

/* ADD-S: add new ois for ofilm module, compatible with sharp module */
void RegWriteA_Ofilm(unsigned short RegAddr, unsigned char RegData)
{
	int32_t  rc = 0;
	uint16_t w_data;
	uint32_t w_addr;
	w_addr = (RegAddr & 0xFFFF);
	w_data = (RegData & 0x00FF);
	rc = msm_camera_cci_i2c_write(g_i2c_clinet,
			w_addr, w_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_err("%s: %x: RegWriteA    failed\n", __func__, RegAddr);
}

void RegReadA_Ofilm(unsigned short RegAddr, unsigned char *RegData)
{
	int32_t rc = 0;
	uint16_t r_data;
	uint32_t w_addr;
	w_addr = (RegAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read(g_i2c_clinet,
			w_addr, &r_data, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0)
		pr_err("%s: %x: RegReadA read  failed\n", __func__, RegAddr);
	*RegData = (r_data & 0x00FF);
}

void RamWriteA_Ofilm(unsigned short RameAddr, unsigned short RamData)
{
	int32_t rc = 0;
	uint16_t w_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	w_data = (RamData & 0xFFFF);
	rc = msm_camera_cci_i2c_write(g_i2c_clinet,
			w_addr, w_data,
			MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0)
		pr_err("%s: %x: RamWriteA  write failed\n", __func__, RameAddr);
}

void RamReadA_Ofilm(unsigned short RameAddr, unsigned short *RamData)
{
	int32_t rc = 0;
	uint16_t r_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read(g_i2c_clinet,
			w_addr, &r_data,
			MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0)
		pr_err("%s: %x: RamReadA read  failed\n", __func__, RameAddr);
	*RamData = (r_data & 0xFFFF);
}

void RamWrite32A_Ofilm(unsigned short RameAddr, unsigned long RamData)
{
	int32_t  rc = 0;
	struct msm_camera_i2c_seq_reg_array reg_setting;
	uint32_t w_data;
	uint32_t w_addr;
	w_addr = (RameAddr & 0xFFFF);
	w_data = (RamData & 0xFFFFFFFF);
	reg_setting.reg_addr = w_addr;
	reg_setting.reg_data[0] = (uint8_t)((w_data & 0xFF000000) >> 24);
	reg_setting.reg_data[1] = (uint8_t)((w_data & 0x00FF0000) >> 16);
	reg_setting.reg_data[2] = (uint8_t)((w_data & 0x0000FF00) >> 8);
	reg_setting.reg_data[3] = (uint8_t)(w_data & 0x000000FF);
	reg_setting.reg_data_size = 4;
	rc = msm_camera_cci_i2c_write_seq(g_i2c_clinet,
		reg_setting.reg_addr,
		reg_setting.reg_data,
		reg_setting.reg_data_size);

	if (rc < 0)
		pr_err("%s: %x: RamWrite32A write failed\n",
			__func__, RameAddr);
}

void RamRead32A_Ofilm(unsigned short RameAddr, unsigned long *RamData)
{

	int32_t rc = 0;
	uint32_t t_data;
	uint32_t w_addr;
	uint8_t r_data[5];
	w_addr = (RameAddr & 0xFFFF);
	rc = msm_camera_cci_i2c_read_seq(g_i2c_clinet,
		w_addr, r_data, 4);
	if (rc < 0)
		pr_err(" %s: %x: RamRead32A RameAddr read failed\n",
			__func__, RameAddr);
	t_data = (r_data[0]<<24)|(r_data[1]<<16)|(r_data[2]<<8)|r_data[3];
	*RamData = (t_data & 0xFFFFFFFF);
}
/* ADD-E: add new ois for ofilm module, compatible with sharp module */

static int Ois_Write_OTP_sharp_imx230(void)
{
	int rc = 0;
	uint8_t ois_otp_buf[MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE];
	uint8_t dac_data = 0;
	OISDBG("Enter");
	rc = msm_get_otp_data(ois_otp_buf,
			MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE, OTP_REAR_CAMERA_OIS);
	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}
	RamAccFixMod(ON);
	/* OIS adjusted parameter */
	RamWriteA(DAXHLO, (ois_otp_buf[2]<<8|ois_otp_buf[1]));
	RamWriteA(DAYHLO, (ois_otp_buf[4]<<8|ois_otp_buf[3]));
	RamWriteA(DAXHLB, (ois_otp_buf[6]<<8|ois_otp_buf[5]));
	RamWriteA(DAYHLB, (ois_otp_buf[8]<<8|ois_otp_buf[7]));
	RamWriteA(OFF0Z, (ois_otp_buf[10]<<8|ois_otp_buf[9]));
	RamWriteA(OFF1Z, (ois_otp_buf[12]<<8|ois_otp_buf[11]));
	RamWriteA(sxg, (ois_otp_buf[14]<<8|ois_otp_buf[13]));
	RamWriteA(syg, (ois_otp_buf[16]<<8|ois_otp_buf[15]));
	RamAccFixMod(OFF);

	/* AF adjusted parameter */
	RegWriteA(IZAL, ois_otp_buf[21]);
	RegWriteA(IZAH, ois_otp_buf[22]);
	RegWriteA(IZBL, ois_otp_buf[23]);
	RegWriteA(IZBH, ois_otp_buf[24]);

	/* Ram Access */
	RamWrite32A(gxzoom,
		(ois_otp_buf[29]<<24 | ois_otp_buf[28]<<16
			|ois_otp_buf[27]<<8 | ois_otp_buf[26]));
	RamWrite32A(gyzoom,
		(ois_otp_buf[33]<<24
			| ois_otp_buf[32]<<16 | ois_otp_buf[31]<<8
			| ois_otp_buf[30]));
	RegWriteA(OSCSET, ois_otp_buf[25]);

	dac_data = ois_otp_buf[35];
	SetDOFSTDAF(dac_data);

	OISDBG("Exit\n");
	return rc;
}

static int Ois_Write_OTP_sharp_imx214(void)
{
	int rc = 0;
	uint8_t ois_otp_buf[MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE];
	OISDBG("Enter");
	rc = msm_get_otp_data(ois_otp_buf,
			MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE, OTP_REAR_CAMERA_OIS);
	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}
	RamAccFixMod(ON);
	/* OIS adjusted parameter */
	RamWriteA(DAXHLO, (ois_otp_buf[2]<<8|ois_otp_buf[1]));
	RamWriteA(DAYHLO, (ois_otp_buf[4]<<8|ois_otp_buf[3]));
	RamWriteA(DAXHLB, (ois_otp_buf[6]<<8|ois_otp_buf[5]));
	RamWriteA(DAYHLB, (ois_otp_buf[8]<<8|ois_otp_buf[7]));

	RamWriteA(OFF0Z, (ois_otp_buf[10]<<8|ois_otp_buf[9]));
	RamWriteA(OFF1Z, (ois_otp_buf[12]<<8|ois_otp_buf[11]));
	RamWriteA(sxg, (ois_otp_buf[14]<<8|ois_otp_buf[13]));
	RamWriteA(syg, (ois_otp_buf[16]<<8|ois_otp_buf[15]));
	RamAccFixMod(OFF);

	/* AF adjusted parameter */
	RegWriteA(IZAL, ois_otp_buf[21]);
	RegWriteA(IZAH, ois_otp_buf[22]);
	RegWriteA(IZBL, ois_otp_buf[23]);
	RegWriteA(IZBH, ois_otp_buf[24]);

	/* Ram Access */
	RamWrite32A(gxzoom,
		(ois_otp_buf[29]<<24 | ois_otp_buf[28]<<16
			|ois_otp_buf[27]<<8 | ois_otp_buf[26]));
	RamWrite32A(gyzoom,
		(ois_otp_buf[33]<<24
			| ois_otp_buf[32]<<16 | ois_otp_buf[31]<<8
			| ois_otp_buf[30]));
	RegWriteA(OSCSET, ois_otp_buf[25]);
	OISDBG("Otp Check Exit\n");
	return rc;
}

/* ADD-S: add new ois for ofilm module, compatible with sharp module */
static int Ois_Write_OTP_Ofilm(void)
{
	int rc = 0;
	uint8_t ois_otp_buf[MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE];
	OISDBG("Enter");
	rc = msm_get_otp_data(ois_otp_buf,
			MSM_OTP_REAR_CAMERA_OIS_BUFF_SIZE, OTP_REAR_CAMERA_OIS);
	if (rc < 0) {
		pr_err("failed: get rear camera ois : get otp data err %d", rc);
		return rc;
	}
	RamAccFixMod_OFILM(ON_OFILM);
	/* OIS adjusted parameter */
	RamWriteA_Ofilm(DAXHLO_OFILM, (ois_otp_buf[2]<<8|ois_otp_buf[1]));
	RamWriteA_Ofilm(DAYHLO_OFILM, (ois_otp_buf[4]<<8|ois_otp_buf[3]));
	RamWriteA_Ofilm(DAXHLB_OFILM, (ois_otp_buf[6]<<8|ois_otp_buf[5]));
	RamWriteA_Ofilm(DAYHLB_OFILM, (ois_otp_buf[8]<<8|ois_otp_buf[7]));
	//RamWriteA_Ofilm(0x1450, 0x199A);/*Hall AD offset X set fix to 0.2f */

	RamWriteA(OFF0Z_OFILM, (ois_otp_buf[10]<<8|ois_otp_buf[9]));
	RamWriteA_Ofilm(OFF1Z_OFILM, (ois_otp_buf[12]<<8|ois_otp_buf[11]));
	RamWriteA_Ofilm(sxg_OFILM, (ois_otp_buf[14]<<8|ois_otp_buf[13]));
	RamWriteA_Ofilm(syg_OFILM, (ois_otp_buf[16]<<8|ois_otp_buf[15]));
	RamAccFixMod_OFILM(OFF_OFILM);

	/* AF adjusted parameter */
	RegWriteA_Ofilm(IZAL_OFILM, ois_otp_buf[21]);
	RegWriteA_Ofilm(IZAH_OFILM, ois_otp_buf[22]);
	RegWriteA_Ofilm(IZBL_OFILM, ois_otp_buf[23]);
	RegWriteA_Ofilm(IZBH_OFILM, ois_otp_buf[24]);

	/* Ram Access */
	RamWrite32A_Ofilm(gxzoom_OFILM,
		(ois_otp_buf[29]<<24 | ois_otp_buf[28]<<16
			|ois_otp_buf[27]<<8 | ois_otp_buf[26]));
	RamWrite32A_Ofilm(gyzoom_OFILM,
		(ois_otp_buf[33]<<24
			| ois_otp_buf[32]<<16 | ois_otp_buf[31]<<8
			| ois_otp_buf[30]));
	RegWriteA_Ofilm(OSCSET_OFILM, ois_otp_buf[25]);
	OISDBG("Otp Check Exit\n");
	return rc;
}
/* ADD-E: add new ois for ofilm module, compatible with sharp module */

static int  Ois_SetI2cClient(struct msm_camera_i2c_client *client)
{
	if (!client) {
		pr_err("%s: i2c client is NULL !!!\n", __func__);
		return INIT_FAILED;
	} else {
		g_i2c_clinet = client;
		g_i2c_clinet->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	}
	return INIT_SUCCESS;
}


static int lc898122_check_i2c(void)
{
	unsigned char check_data;

	RegReadA(LC898122_CHECK_RIGISTER, &check_data);
	if (LC898122_CHECK_DATA == check_data) {
		return INIT_SUCCESS;
	} else {
		pr_err("ois lc898122_check_i2c error : 0x%x\n", check_data);
		return INIT_FAILED;
	}
}

static int lc898122_max1_init(int32_t type)
{
	int32_t rc = 0;
	rc = lc898122_check_i2c();
	if (rc < 0)
		return INIT_SUCCESS;
	SelectModule(MODULE_20M);
	OISDBG("  max1 type:%d\n", type);
	switch (type) {
	case OIS_INIT_S:
		OISDBG(" Init set");
		IniSetAf();
		IniSet();
		rc = Ois_Write_OTP_sharp_imx230();
		RemOff(ON);
		SetTregAf(0x0400);
		RtnCen(0x00);
		msleep(150);
		RemOff(OFF);
		OISDBG(" return center");
		break;
	case OIS_CENTERING_ON_S:
		/*RemOff(ON);
		SetTregAf(0x0100);
		RtnCen(0x00);
		msleep(150);
		RemOff(OFF);
		OISDBG(" return center");
		*/
		break;
	case OIS_CENTERING_OFF_S:
		break;
	case OIS_PANTILT_ON_S:
		/*SetPanTilode(ON);
		OISDBG(" SetPanTilode over\n"); */
		break;
	case OIS_ENABLE_S:
		OisEna();
		OISDBG(" OisEna over\n");
		break;
	case OIS_DISABLE_S:
		/* OisDisable(); */
		break;
	case OIS_STILL_MODE_S:
		SetH1cMod(0);
		OISDBG(" SetSTILLMode over\n");
		break;
	case OIS_MOVIE_MODE_S:
		SetH1cMod(MOVMODE);
		OISDBG(" SetmoveMODE over\n");
		break;
	case OIS_CALIBRATION_S:
		break;
	default:
		pr_err("Not have thise case type :%d\n", type);
		break;
	}

	return rc;

}

/* ADD-S: add new ois for ofilm module, compatible with sharp module */
#define SHARP_MODULE 0x01
#define OFILM_MODULE 0x02
#define VALID_MODULE 0X03
static int Ois_get_mode(void)
{
	int32_t rc = 0;
	uint8_t module_id = 0;
	uint8_t moduleid_otp_buf[MSM_OTP_REAR_CAMERA_MODULE_ID_BUFF_SIZE];
	rc = msm_get_otp_data(moduleid_otp_buf,
		MSM_OTP_REAR_CAMERA_MODULE_ID_BUFF_SIZE,
		OTP_REAR_CAMERA_MODULE_ID);
	if (rc < 0) {
		return rc;
	} else {
		module_id = moduleid_otp_buf[0];
		if (module_id == 0x01) {
			SelectModule(MODULE_13M);
			OISDBG(" sharp module module_id :%x", module_id);
			return SHARP_MODULE;
		} else if (module_id == 0x10) {
			OISDBG(" ofilm module module_id:%x\n", module_id);
			return OFILM_MODULE;
		} else {
			OISDBG(" valiled module module_id:%x\n", module_id);
			return VALID_MODULE;
		}
	}
}
static int lc898122_x1_ofilm_init(int32_t type)
{
	int32_t rc = 0;
	rc = lc898122_check_i2c();
	if (rc < 0)
		return INIT_SUCCESS;
	OISDBG(" X1 type :%d\n", type);
	switch (type) {
	case OIS_INIT_S:
		OISDBG(" Init set");
		IniSet_OFILM();
		IniSetAf_OFILM();
//		RamWrite32A(gxlmt3HS0_OFILM, 0x3EB33333);  /*0.35f*/
//		RamWrite32A(gxlmt3HS1_OFILM, 0x3EB33333);  /*0.35f*/
		Ois_Write_OTP_Ofilm();
		/* RemOff_OFILM_3M2(ON); */
		/* RegWriteA_Ofilm(0x0302, 0x00); */
		/* RegWriteA_Ofilm(0x0303, 0xDF); */
		RegWriteA_Ofilm(0x0304, 100);
		/* SetTregAf_ofilm(0x0100); */
		RtnCen_OFILM(0x00);
		msleep(150);
		/* RemOff_ofilm(OFF); */
		OISDBG(" return center");
		/* ois_sin_test(); */
		OISDBG("============after return\n");
		break;
	case OIS_CENTERING_ON_S:
	/*	RemOff(ON);
		RtnCen(0x00);
		SetTregAf(0x0100);
		msleep(150);
		RemOff(OFF);
		OISDBG(" return center");
		*/
		break;
	case OIS_CENTERING_OFF_S:
		break;
	case OIS_PANTILT_ON_S:
		SetPanTiltMode_OFILM(ON_OFILM);
		OISDBG(" SetPanTilode over\n");
		break;
	case OIS_ENABLE_S:
		OisEna_OFILM();
		OISDBG(" OisEna over\n");
		break;
	case OIS_DISABLE_S:
		/* OisDisable(); */
		break;
	case OIS_STILL_MODE_S:
		SetH1cMod_OFILM(0);
		break;
	case OIS_MOVIE_MODE_S:
		SetH1cMod_OFILM(MOVMODE);
		break;
	case OIS_CALIBRATION_S:
		break;
	default:
		pr_err("Not have thise case type :%d\n", type);
		break;
	}
	return rc;
}
/* ADD-E: add new ois for ofilm module, compatible with sharp module */

static int lc898122_x1_sharp_init(int32_t type)
{
	int32_t rc = 0;
	rc = lc898122_check_i2c();
	if (rc < 0)
		return INIT_SUCCESS;
	OISDBG(" X1 type :%d\n", type);
	switch (type) {
	case OIS_INIT_S:
		OISDBG(" Init set");
		/*IniSetAf();*/
		IniSet();
		Ois_Write_OTP_sharp_imx214();
		RemOff(ON);
		RegWriteA(0x0302, 0x00);
		RegWriteA(0x0303, 0xDF);
		RegWriteA(0x0304, 0x04);
		SetTregAf(0x0100);
		RtnCen(0x00);
		msleep(150);
		RemOff(OFF);
		OISDBG(" return center");
		/* ois_sin_test(); */
		OISDBG("============after return\n");
		break;
	case OIS_CENTERING_ON_S:
	/*	RemOff(ON);
		RtnCen(0x00);
		SetTregAf(0x0100);
		msleep(150);
		RemOff(OFF);
		OISDBG(" return center");
		*/
		break;
	case OIS_CENTERING_OFF_S:
		break;
	case OIS_PANTILT_ON_S:
		/*SetPanTilode(ON);
		OISDBG(" SetPanTilode over\n");*/
		break;
	case OIS_ENABLE_S:
		OisEna();
		OISDBG(" OisEna over\n");
		break;
	case OIS_DISABLE_S:
		/* OisDisable(); */
		break;
	case OIS_STILL_MODE_S:
		SetH1cMod(0);
		break;
	case OIS_MOVIE_MODE_S:
		SetH1cMod(MOVMODE);
		break;
	case OIS_CALIBRATION_S:
		break;
	default:
		pr_err("Not have thise case type :%d\n", type);
		break;
	}
	return rc;
}

int oiscontrol_interface(struct msm_camera_i2c_client *client,
		const char *module_name, int32_t type)
{
	int32_t rc = 0;
	Ois_SetI2cClient(client);
	if (strcmp(module_name, "max1") == 0)
		rc = lc898122_max1_init(type);
	/* ADD-S: add new ois for ofilm module, compatible with sharp module */
	if (strcmp(module_name, "x1") == 0) {
		rc = Ois_get_mode();
		if (SHARP_MODULE == rc)
			rc = lc898122_x1_sharp_init(type);
		else if (OFILM_MODULE == rc)
			rc = lc898122_x1_ofilm_init(type);
	}
	/* ADD-E: add new ois for ofilm module, compatible with sharp module */
	return rc;
}
