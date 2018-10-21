/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */
/*********************************************************
History:

   Description: add mipi  azet hx8379a id:0.3V j213



*********************************************************/

#ifdef BUILD_LK
#include "platform/mt_gpio.h"
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mt-plat/mt_gpio.h>
#endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//RGK add
// ---------------------------------------------------------------------------
#include <cust_adc.h>    	// zhoulidong  add for lcm detect
#define MIN_VOLTAGE (1000)     // zhoulidong  add for lcm detect
#define MAX_VOLTAGE (2200)     // zhoulidong  add for lcm detect

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  			(480)
#define FRAME_HEIGHT 			(854)

#define REGFLAG_DELAY             	0XFE
#define REGFLAG_END_OF_TABLE   		0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_HX8379			0x8379  //D3

#define LCM_DSI_CMD_MODE		0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif
static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    				(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 					(lcm_util.udelay(n))
#define MDELAY(n) 					(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg						lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

// zhoulidong  add for lcm detect ,read adc voltage
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = {
{0xB9,3,{0xFF,0x83,0x79}},  

{0xB1,20,{0x44,0x17,0x17,0x2F,0x2F,0x50,0xd0,0xEE,0x54,0x80,0x38,0x38,0xF8,0x22,0x22,0x22,0x00,0x80,0x30,0x00}},

{0xB2,9,{0x82,0xFE,0x0d,0x0a,0x00,0x50,0x11,0x42,0x1D}},
  
{0xB4,10,{0x02,0x55,0x00,0x66,0x00,0x66,0x22,0x77,0x23,0x77}}, 

{0xcc,1,{0x02}},     
//{0xd2,1,{0x33}}, 
 
   
{0xd3,35,{0x00,0x07,0x00,0x3c,0x01,0x08,0x08,0x32,0x10,0x04,0x00,0x04,0x03,0x70,0x03,0x70,0x00,0x08,0x00,0x08,0x37,0x33,0x06,0x06,0x37,0x06,0x06,0x37,0x0b,0x00,0x00,0x00,0x0a,0x00,0x11}},

{0xD5,34,{0x19,0x19,0x18,0x18,0x1a,0x1a,0x1b,0x1b,0x02,0x03,0x00,0x01,0x06,0x07,0x04,0x05,0x20,0x21,0x22,0x23,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x00}},

{0xD6,32,{0x18,0x18,0x19,0x19,0x1b,0x1b,0x1a,0x1a,0x03,0x02,0x05,0x04,0x07,0x06,0x01,0x00,0x23,0x22,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},


{0xE0,42,{0x00,0x04,0x07,0x0b,0x0c,0x23,0x1d,0x2f,0x08,0x0b,0x0c,0x17,0x0f,0x14,0x17,0x15,0x16,0x09,0x14,0x15,0x08,0x00,0x04,0x07,0x0b,0x0c,0x23,0x1d,0x2f,0x08,0x0b,0x0c,0x17,0x0f,0x14,0x17,0x15,0x16,0x09,0x14,0x15,0x18}},
 
{0xC7,4,{0x00,0x00,0x00,0xC0}}, 

{0xB6,2,{0x51,0x51}}, 

{0x11,1,{0x00}},
{REGFLAG_DELAY, 120, {}},

{0x29,1,{0x00}}, //Display On
{REGFLAG_DELAY, 20, {}},
};
//static unsigned int  vcomadj=0x93;
static void init_lcm_registers(void)
{

unsigned int data_array[16];

#if 1
// HX8379-C+HSD4.5 after 20150604 by himax
data_array[0] = 0x00043902;
data_array[1] = 0x7983FFB9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00113902;
data_array[1] = 0x161644B1;//47
data_array[2] = 0xD0503131;
data_array[3] = 0x388054EE;
data_array[4] = 0x2222F838;
data_array[5] = 0x00000022;
dsi_set_cmdq(data_array, 6, 1);
MDELAY(1);

data_array[0] = 0x000A3902;
data_array[1] = 0x0dFE82B2;
data_array[2] = 0x1150200a;//sap
data_array[3] = 0x00001542;
dsi_set_cmdq(data_array, 4, 1);
MDELAY(1);

data_array[0] = 0x000B3902;
data_array[1] = 0x027C02B4;
data_array[2] = 0x227C027C;
data_array[3] = 0x00862386;
dsi_set_cmdq(data_array, 4, 1);
MDELAY(1);

data_array[0] = 0x00053902;
data_array[1] = 0x000000C7;
data_array[2] = 0x000000C0;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);


data_array[0] = 0x00023902;
data_array[1] = 0x000002CC;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000011D2;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x001E3902;
data_array[1] = 0x000700D3;
data_array[2] = 0x08081C3C;
data_array[3] = 0x00021032;
data_array[4] = 0x03700302;
data_array[5] = 0x00080070;
data_array[6] = 0x06333708;
data_array[7] = 0x06063706;
data_array[8] = 0x00000b37;
dsi_set_cmdq(data_array, 9, 1);
MDELAY(1);

data_array[0] = 0x00233902;
data_array[1] = 0x181919D5;
data_array[2] = 0x1b1a1a18;
data_array[3] = 0x0003021b;
data_array[4] = 0x04070601;
data_array[5] = 0x22212005;
data_array[6] = 0x18181823;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x00000018;
dsi_set_cmdq(data_array, 10, 1);
MDELAY(1);

data_array[0] = 0x00213902;
data_array[1] = 0x191818D6;
data_array[2] = 0x1b1a1a19;
data_array[3] = 0x0502031b;
data_array[4] = 0x01060704;
data_array[5] = 0x21222300;
data_array[6] = 0x18181820;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x00000018;
dsi_set_cmdq(data_array, 10, 1);
MDELAY(1);

data_array[0] = 0x002B3902;
data_array[1] = 0x040000E0;
data_array[2] = 0x1A3F0D0b;
data_array[3] = 0x0E0C082D;
data_array[4] = 0x17141018;
data_array[5] = 0x13091615;
data_array[6] = 0x00001814;
data_array[7] = 0x3F0D0b04;
data_array[8] = 0x0C072E1A;
data_array[9] = 0x130f190F;
data_array[10] = 0x07141416;
data_array[11] = 0x00171312;
dsi_set_cmdq(data_array, 12, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x004848B6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);



data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(120);
data_array[0] = 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(20);

#else
//HX8379-C+HSD4.5 before 20150604
data_array[0] = 0x00043902;
data_array[1] = 0x7983FFB9;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00113902;
data_array[1] = 0x161647B1;
data_array[2] = 0xD0503131;
data_array[3] = 0x388054EE;
data_array[4] = 0x2222F838;
data_array[5] = 0x00000022;
dsi_set_cmdq(data_array, 6, 1);
MDELAY(1);

data_array[0] = 0x000A3902;
data_array[1] = 0x0dFE82B2;
data_array[2] = 0x1150000a;
data_array[3] = 0x00001D42;
dsi_set_cmdq(data_array, 4, 1);
MDELAY(1);

data_array[0] = 0x000B3902;
data_array[1] = 0x005502B4;
data_array[2] = 0x22660066;
data_array[3] = 0x00772377;
dsi_set_cmdq(data_array, 4, 1);
MDELAY(1);
data_array[0] = 0x00053902;
data_array[1] = 0x000000C7;
data_array[2] = 0x000000C0;
dsi_set_cmdq(data_array, 3, 1);
MDELAY(1);


data_array[0] = 0x00023902;
data_array[1] = 0x000002CC;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x00023902;
data_array[1] = 0x000011D2;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);

data_array[0] = 0x001E3902;
data_array[1] = 0x000700D3;
data_array[2] = 0x0808143C;
data_array[3] = 0x00051032;
data_array[4] = 0x03700305;
data_array[5] = 0x00080070;
data_array[6] = 0x07222708;
data_array[7] = 0x07072707;
data_array[8] = 0x00000b27;
dsi_set_cmdq(data_array, 9, 1);
MDELAY(1);

data_array[0] = 0x00233902;
data_array[1] = 0x181919D5;
data_array[2] = 0x1b1a1a18;
data_array[3] = 0x0003021b;
data_array[4] = 0x04070601;
data_array[5] = 0x22212005;
data_array[6] = 0x18181823;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x00000018;
dsi_set_cmdq(data_array, 10, 1);
MDELAY(1);

data_array[0] = 0x00213902;
data_array[1] = 0x191818D6;
data_array[2] = 0x1b1a1a19;
data_array[3] = 0x0502031b;
data_array[4] = 0x01060704;
data_array[5] = 0x21222300;
data_array[6] = 0x18181820;
data_array[7] = 0x18181818;
data_array[8] = 0x18181818;
data_array[9] = 0x00000018;
dsi_set_cmdq(data_array, 10, 1);
MDELAY(1);

data_array[0] = 0x002B3902;
data_array[1] = 0x040000E0;
data_array[2] = 0x1A3F0D0b;
data_array[3] = 0x0E0C082D;
data_array[4] = 0x17141018;
data_array[5] = 0x13091615;
data_array[6] = 0x00001814;
data_array[7] = 0x3F0D0b04;
data_array[8] = 0x0C072E1A;
data_array[9] = 0x130f190F;
data_array[10] = 0x07141416;
data_array[11] = 0x00171312;
dsi_set_cmdq(data_array, 12, 1);
MDELAY(1);

data_array[0] = 0x00033902;
data_array[1] = 0x004848B6;
dsi_set_cmdq(data_array, 2, 1);
MDELAY(1);



data_array[0] = 0x00110500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(120);
data_array[0] = 0x00290500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(20);

#endif

}



/*

static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 200, {}},

	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

*/
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	//{0x28, 1, {0x00}},
	//{REGFLAG_DELAY, 50, {}},

	// Sleep Mode On
	{0x39, 1, {0x28}},
	{REGFLAG_DELAY, 120, {}},
	{0x39, 1, {0x10}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
/*

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	
	for(i = 0; i < count; i++)
	{
		unsigned cmd;
		cmd = table[i].cmd;
		
		switch (cmd)
		{
		case REGFLAG_DELAY :
			MDELAY(table[i].count);
			break;
		case REGFLAG_END_OF_TABLE :
			break;
		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

   memset(params, 0, sizeof(LCM_PARAMS));
    
    params->type   = LCM_TYPE_DSI;
    
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

	params->physical_width = 56;
	params->physical_height = 99;	
    
    // enable tearing-free
    params->dbi.te_mode				= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
    //params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE;

    params->dsi.ssc_disable = 1;         //关闭展屏  
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;
    
    // Video mode setting		
    params->dsi.intermediat_buffer_num = 2;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    
    params->dsi.word_count=480*3;	//DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line=854;

    params->dsi.vertical_sync_active				=5;
    params->dsi.vertical_backporch					= 8;
    params->dsi.vertical_frontporch					= 12;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
/*
    params->dsi.vertical_sync_active				= 4;
    params->dsi.vertical_backporch					= 8;
    params->dsi.vertical_frontporch					= 8;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;

*/

    params->dsi.horizontal_sync_active				= 54;///////////////20 20 4  20  14  6
     params->dsi.horizontal_backporch				= 54;
    params->dsi.horizontal_frontporch				= 54;
   // params->dsi.horizontal_blanking_pixel				= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    /*
    params->dsi.horizontal_sync_active				= 6;
    params->dsi.horizontal_backporch				= 37;
    params->dsi.horizontal_frontporch				= 37;
    params->dsi.horizontal_blanking_pixel				= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
    */
    params->dsi.PLL_CLOCK = 200;//156;
    // Bit rate calculation

   // params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4
   // params->dsi.pll_div2=1;		// div2=0,1,2,3;div2_real=1,2,4,4
   // params->dsi.fbk_sel=1;		 // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
   // params->dsi.fbk_div =27;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)		

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a; //\u5bc4\u5b58\u5668
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c; //\u8bfb\u53d6\u7684\u503c 

	params->dsi.lcm_esd_check_table[1].cmd = 0x0d; //\u5bc4\u5b58\u5668
	params->dsi.lcm_esd_check_table[1].count = 1;
	params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00; //\u8bfb\u53d6\u7684\u503c 

	//params->dsi.lcm_esd_check_table[2].cmd = 0x0e; //\u5bc4\u5b58\u5668
	//params->dsi.lcm_esd_check_table[2].count = 1;
	//params->dsi.lcm_esd_check_table[2].para_list[0] = 0x00; //\u8bfb\u53d6\u7684\u503c 

	//params->dsi.lcm_esd_check_table[3].cmd = 0x0f; //\u5bc4\u5b58\u5668
	//params->dsi.lcm_esd_check_table[3].count = 1;
	//params->dsi.lcm_esd_check_table[3].para_list[0] = 0xc0; //\u8bfb\u53d6\u7684\u503c 


}


static void lcm_init(void)
{
	//printk("lcm_init by test");	
	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(20);
	#if defined(BUILD_LK) 
 printf("MYCAT hx8379c, lcm_init\n");
 #else
 printk("MYCAT hx8379c, lcm_init\n");
#endif

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	init_lcm_registers();
}


static void lcm_suspend(void)
{	
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
unsigned int data_array[16];

data_array[0] = 0x00100500;
dsi_set_cmdq(data_array, 1, 1);
MDELAY(120);
	
	SET_RESET_PIN(1);     
        MDELAY(1);
        SET_RESET_PIN(0);
        MDELAY(5);
        SET_RESET_PIN(1);
        MDELAY(120);	
	
}


static void lcm_resume(void)
{
	lcm_init();
	
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

/*
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int array[16];

	array[0]= 0x00053902;
	array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	array[2]= (x1_LSB);
	array[3]= 0x00053902;
	array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	array[5]= (y1_LSB);
	array[6]= 0x002c3909;

	dsi_set_cmdq(array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255) 
			level = 255;

	if(level >0) 
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

*/

// zhoulidong  add for lcm detect (start)

static unsigned int lcm_compare_id(void)
{
  int   data_array[4];
  char  buffer[5];
  unsigned int id=0;
        SET_RESET_PIN(1);
  MDELAY(5);
  SET_RESET_PIN(0);
  MDELAY(10);
  SET_RESET_PIN(1);
  MDELAY(120);
 
    data_array[0]= 0x00043902;  
    data_array[1]= 0x7983FFB9; 
    dsi_set_cmdq(data_array, 2, 1); 
    MDELAY(10);

 data_array[0] = 0x00053700;// read id return two byte,version and id
 dsi_set_cmdq(data_array, 1, 1);
 read_reg_v2(0x04,buffer,5);
 
 id=(buffer[0]<<8)+buffer[1];
#if defined(BUILD_LK) 
 printf("MYCAT hx8379c, id = 0x%08x\n",  id);
 #else
 printk("MYCAT hx8379c, id = 0x%08x\n",  id);
#endif
  return (LCM_ID_HX8379 == id)?1:0;


}


static int rgk_lcm_compare_id(void)
{
	return 1;
	int data[4] = {0,0,0,0};
	int res = 0;
	int rawdata = 0;
	int lcm_vol = 0;

//#ifdef AUXADC_LCM_VOLTAGE_CHANNEL
	res = IMM_GetOneChannelValue(1,data,&rawdata);
	if(res < 0)
	{ 
	#ifdef BUILD_LK
		printf("[adc_uboot]: get data error\n");
	#endif
		return 0;
	}
//#endif
	lcm_vol = data[0]*1000+data[1]*10;

	#ifdef BUILD_LK
	printf("[adc_uboot]: lcm_vol= %d\n",lcm_vol);
	printf("rawdata %d\n",data);
	#endif
/*	
	if (lcm_vol>=MIN_VOLTAGE &&lcm_vol <= MAX_VOLTAGE &&lcm_compare_id())
	{
		return 1;
	}
*/
	if (rawdata>=MIN_VOLTAGE&&lcm_compare_id())
	{
		return 1;
	}
	return 0;
	
}
// zhoulidong add for eds(start)
static unsigned int lcm_esd_check(void)
{
	#ifdef BUILD_LK
		//printf("lcm_esd_check()\n");
	#else
		//printk("lcm_esd_check()\n");
	#endif 
#ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x0a, buffer, 1);
	if(buffer[0]==0x9c)
	{
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif
		return FALSE;
	}
	else
	{	
		//#ifdef BUILD_LK
		//printf("%s %d\n FALSE", __func__, __LINE__);
		//#else
		//printk("%s %d\n FALSE", __func__, __LINE__);
		//#endif		 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	
	#ifdef BUILD_LK
		printf("lcm_esd_recover()\n");
	#else
		printk("lcm_esd_recover()\n");
	#endif	
	
	lcm_init();	

	return TRUE;
}
// zhoulidong add for eds(end)

LCM_DRIVER zaw700_hx8379c_fwvga_dsi_vdo_txd_TXDT450SKP_lcm_drv = 
{
    	.name			= "zaw700_hx8379c_fwvga_dsi_vdo_txd_TXDT450SKP",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id	=  lcm_compare_id,
	//.esd_check     = lcm_esd_check,
	//.esd_recover   = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
	//.update         = lcm_update,
#endif
};

