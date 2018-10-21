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

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/


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
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)

#define MIN_VOLTAGE (200)     // dongteng add for lcm detect
#define MAX_VOLTAGE (1200)     // dongteng add for lcm detect
#define COMPARE_BY_ADC   0

#define REGFLAG_DELAY          	0XFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_ID_OTM8019 (0x8019)

#define LCM_DSI_CMD_MODE									0

#ifndef TRUE
    #define   TRUE     1
#endif
 
#ifndef FALSE
    #define   FALSE    0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static unsigned int lcm_compare_id(void);

static struct LCM_setting_table lcm_initialization_setting[] = {

	{0x00, 1, {0x00}},
	{0xff, 3, {0x80,0x19,0x01}},

	{0x00, 1, {0x80}},
	{0xff, 2, {0x80,0x19}},

	{0x00, 1, {0x90}},
	{0xb3, 1, {0x02}},

	{0x00, 1, {0x92}},
	{0xb3, 1, {0x45}},

	{0x00, 1, {0x80}},
	{0xc0, 9, {0x00,0x58,0x00,0x15,0x15,0x00,0x58,0x15,0x15}},

	{0x00, 1, {0x90}},
	{0xc0, 6, {0x00,0x15,0x00,0x00,0x00,0x03}},

	{0x00, 1, {0xa2}},
	{0xc0, 3, {0x02,0x1b,0x02}},

	{0x00, 1, {0xb4}},
	{0xc0, 1, {0x70}},

	{0x00, 1, {0x89}},
	{0xc4, 1, {0x00}},

	{0x00, 1, {0x81}},
	{0xc4, 1, {0x83}},

	{0x00, 1, {0x81}},
	{0xc5, 1, {0x66}},

	{0x00, 1, {0x82}},
	{0xc5, 1, {0xb0}},

	{0x00, 1, {0x90}},
	{0xc5, 6, {0x4e,0x87,0x01,0x33,0x44,0x44}},

	{0x00, 1, {0xb1}},
	{0xc5, 1, {0xa9}},

	{0x00, 1, {0x00}},
	{0xd8, 2, {0x62,0x62}},

	//{0x00, 1, {0x00}},
	//{0xd9, 1, {0x49}},

	{0x00, 1, {0x80}},
	{0xce, 12, {0x8b,0x03,0x00,0x8a,0x03,0x00,0x89,0x03,0x00,0x88,0x03,0x00}},

	{0x00, 1, {0xa0}},
	{0xce, 14, {0x38,0x07,0x03,0x54,0x00,0x00,0x00,0x38,0x06,0x03,0x55,0x00,0x00,0x00}},

	{0x00, 1, {0xb0}},
	{0xce, 14, {0x38,0x05,0x03,0x56,0x00,0x00,0x00,0x38,0x04,0x03,0x57,0x00,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xce, 14, {0x38,0x03,0x03,0x58,0x00,0x00,0x00,0x38,0x02,0x03,0x59,0x00,0x00,0x00}},

	{0x00, 1, {0xd0}},
	{0xce, 14, {0x38,0x01,0x03,0x5a,0x00,0x00,0x00,0x38,0x00,0x03,0x5b,0x00,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xcf, 10, {0x02,0x02,0x00,0x00,0x00,0x00,0x01,0x80,0x02,0x00}},

	{0x00, 1, {0x80}},
	{0xcb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0x90}},
	{0xcb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xa0}},
	{0xcb, 1, {0x00}},

	{0x00, 1, {0xa5}},
	{0xcb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xb0}},
	{0xcb, 6, {0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xc0}},
	{0xcb, 15, {0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xd0}},
	{0xcb, 1, {0x00}},

	{0x00, 1, {0xd5}},
	{0xcb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01}},

	{0x00, 1, {0xe0}},
	{0xcb, 6, {0x01,0x01,0x01,0x01,0x01,0x01}},

	{0x00, 1, {0x80}},
	{0xcc, 10, {0x26,0x25,0x21,0x22,0x0c,0x0a,0x10,0x0e,0x02,0x04}},

	{0x00, 1, {0x90}},
	{0xcc, 6, {0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0x9a}},
	{0xcc, 5, {0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xa0}},
	{0xcc, 11, {0x00,0x03,0x01,0x0d,0x0f,0x09,0x0b,0x22,0x21,0x25,0x26}},

	{0x00, 1, {0xb0}},
	{0xcc, 10, {0x25,0x26,0x21,0x22,0x10,0x0a,0x0c,0x0e,0x04,0x02}},

	{0x00, 1, {0xc0}},
	{0xcc, 6, {0x00,0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xca}},
	{0xcc, 5, {0x00,0x00,0x00,0x00,0x00}},

	{0x00, 1, {0xd0}},
	{0xcc, 11, {0x00,0x01,0x03,0x0d,0x0b,0x09,0x0f,0x22,0x21,0x26,0x25}},

	{0x00, 1, {0x00}},
	{0xe1, 20, {0x00,0x01,0x05,0x0E,0x1D,0x31,0x39,0x6d,0x5F,0x77,0x8E,0x7C,0x94,0x7E,0x83,0x7C,0x73,0x6A,0x5F,0x30}},
  
	{0x00, 1, {0x00}},
	{0xe2, 20, {0x00,0x01,0x05,0x0E,0x1D,0x31,0x39,0x6d,0x5F,0x77,0x8E,0x7C,0x93,0x7E,0x83,0x7B,0x73,0x6A,0x5F,0x30}},

	{0x00, 1, {0x80}},
	{0xc4, 1, {0x30}},

	{0x00, 1, {0x98}},
	{0xc0, 1, {0x00}},

	{0x00, 1, {0xa9}},
	{0xc0, 1, {0x0a}},

	{0x00, 1, {0xb0}},
	{0xc1, 3, {0x20,0x00,0x00}},

	{0x00, 1, {0xe1}},
	{0xc0, 2, {0x40,0x30}},

	{0x00, 1, {0x80}},
	{0xc1, 2, {0x03,0x33}},

	{0x00, 1, {0xa0}},
	{0xc1, 1, {0xe8}},

	{0x00, 1, {0x90}},
	{0xb6, 1, {0xb4}},


	{0x00, 1, {0x00}},
	{0xfb, 1, {0x01}},

	{0x00, 1, {0x00}},
	{0xff, 3, {0xff,0xff,0xff}},



{0x11,1,{ 0x00 }},
{REGFLAG_DELAY, 120, {}},

{0x29,1,{ 0x00 }},
{REGFLAG_DELAY, 10, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_init_registers(void)
{
	unsigned int data_array[16];
	
	data_array[0] = 0x00002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00042902;
	data_array[1] = 0x018712FF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x80002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x008712ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x92002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x000220ff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x80002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000a2902;
	data_array[1] = 0x006400c0;
	data_array[2] = 0x64001010;
	data_array[3] = 0x00001010;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x90002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00072902;
	data_array[1] = 0x004b00c0;
	data_array[2] = 0x00040001;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0xb3002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x005500c0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x81002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x000055c1;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0xa0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x041005c4;
	data_array[2] = 0x11150502;
	data_array[3] = 0x02071005;
	data_array[4] = 0x00111505;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xb0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x000000c4;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x91002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x00d2a6c5;
	dsi_set_cmdq(data_array, 2, 1);

	
	data_array[0] = 0x00002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x00B7B7d8;
	dsi_set_cmdq(data_array, 2, 1);


	data_array[0] = 0xb3002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x000084c5;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0xbb002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x00008ac5;
	dsi_set_cmdq(data_array, 2, 1);

	

	data_array[0] = 0x82002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x00000ac4;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0xc6002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x000003b0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x000040d0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x000000d1;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0xb2002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x000000f5;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0xb6002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x000000f5;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x94002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x000000f5;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0xd2002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00032902;
	data_array[1] = 0x001506f5;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0xb4002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x0000ccc5;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x90002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00052902;
	data_array[1] = 0x021102f5;	
	data_array[2] = 0x00000015;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x90002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x000050c5;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x94002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00022902;
	data_array[1] = 0x000066c5;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x80002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000c2902;
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x90002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xA0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xB0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xC0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x050505cb;
	data_array[2] = 0x05050505;
	data_array[3] = 0x05000505;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xD0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x000000cb;
	data_array[2] = 0x00000500;
	data_array[3] = 0x05050505;
	data_array[4] = 0x05050505;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xE0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x050005cb;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000005;
	dsi_set_cmdq(data_array, 5, 1);


	data_array[0] = 0xf0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000c2902;
	data_array[1] = 0xffffffcb;
	data_array[2] = 0xffffffff;
	data_array[3] = 0xffffffff;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0x80002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x0a2a29cc;
	data_array[2] = 0x12100e0c;
	data_array[3] = 0x08000614;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0x90002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x000000cc;
	data_array[2] = 0x00000200;
	data_array[3] = 0x0b092a29;
	data_array[4] = 0x13110f0d;
	dsi_set_cmdq(data_array, 5, 1);

	

	data_array[0] = 0xa0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x070005cc;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000001;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xb0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x132a29cc;
	data_array[2] = 0x0b0d0f11;
	data_array[3] = 0x07000109;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xc0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00102902;
	data_array[1] = 0x000000cc;
	data_array[2] = 0x00000500;
	data_array[3] = 0x12142a29;
	data_array[4] = 0x0a0c0e10;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xd0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x080002cc;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000006;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0x80002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000D2902;
	data_array[1] = 0x100589ce;
	data_array[2] = 0x00000588;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0x90002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x10fc54ce;
	data_array[2] = 0x5500fd54;
	data_array[3] = 0x01550000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xA0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x040758ce;
	data_array[2] = 0x001000fc;
	data_array[3] = 0xfd040658;
	data_array[4] = 0x00001000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xB0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x040558ce;
	data_array[2] = 0x000000fe;
	data_array[3] = 0xff040458;
	data_array[4] = 0x00001000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xc0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x050358ce;
	data_array[2] = 0x00100000;
	data_array[3] = 0x01050258;
	data_array[4] = 0x00001000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xd0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x050158ce;
	data_array[2] = 0x00100002;
	data_array[3] = 0x03050058;
	data_array[4] = 0x00001000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0x80002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x050050cf;
	data_array[2] = 0x00100004;
	data_array[3] = 0x05050150;
	data_array[4] = 0x00001000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0x90002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x050250cf;
	data_array[2] = 0x00100006;
	data_array[3] = 0x07050350;
	data_array[4] = 0x00001000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xa0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xb0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000f2902;
	data_array[1] = 0x000000cf;
	data_array[2] = 0x00000000;
	data_array[3] = 0x00000000;
	data_array[4] = 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0] = 0xc0002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x000c2902;
	data_array[1] = 0x203939cf;
	data_array[2] = 0x01000020;
	data_array[3] = 0x00002001;
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0] = 0xb5002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00072902;
	data_array[1] = 0xff950bc5;
	data_array[2] = 0x00ff950b;
	dsi_set_cmdq(data_array, 3, 1);

//gamma 2.2+
	data_array[0] = 0x00002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00152902;
	data_array[1] = 0x302511e1;
	data_array[2] = 0x59584b3c;
	data_array[3] = 0x7b887081;
	data_array[4] = 0x51547968;
	data_array[5] = 0x1f263543;
	data_array[6] = 0x00000008;
	dsi_set_cmdq(data_array, 7, 1);

//gamma2.2-
	data_array[0] = 0x00002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00152902;
	data_array[1] = 0x302511e2;
	data_array[2] = 0x58584b3c;
	data_array[3] = 0x7b887081;
	data_array[4] = 0x51547967;
	data_array[5] = 0x1f263543;
	data_array[6] = 0x00000008;
	dsi_set_cmdq(data_array, 7, 1);

	//data_array[0] = 0xa0002300;
	//dsi_set_cmdq(data_array, 1, 1);
	//data_array[0] = 0x00022902;
	//data_array[1] = 0x0000c2c1;
	//dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00352300;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00002300;
	dsi_set_cmdq(data_array, 1, 1);
	data_array[0] = 0x00042902;
	data_array[1] = 0xffffffff;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(80);
	
}

/*
static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
*/


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 100, {}},

    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


//int vcom=0xa7;
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
			//case 0xd9 :
			//	table[i].para_list[0]=vcom;
			//	dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
			//	vcom-=2;
            //    break;
				
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
	 
	 params->type	= LCM_TYPE_DSI;
	 
	 params->width	= FRAME_WIDTH;
	 params->height = FRAME_HEIGHT;
		// enable tearing-free
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;//LCM_DBI_TE_MODE_VSYNC_ONLY;
	//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
	 params->dsi.mode	=  SYNC_PULSE_VDO_MODE;  
	 // DSI
	 /* Command mode setting */
	 params->dsi.LANE_NUM				 = LCM_TWO_LANE;
	 
	 //The following defined the fomat for data coming from LCD engine.
	 params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	 params->dsi.data_format.trans_seq	 = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	 params->dsi.data_format.padding	 = LCM_DSI_PADDING_ON_LSB;
	 params->dsi.data_format.format    = LCM_DSI_FORMAT_RGB888;
	 
	 // Video mode setting		 
	 params->dsi.intermediat_buffer_num = 2;
	 
	 params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	 
	 params->dsi.word_count=480*3;	 //DSI CMD mode need set these two bellow params, different to 6577
	 params->dsi.vertical_active_line=854;
	
	 params->dsi.vertical_sync_active				 = 8;//4
	 params->dsi.vertical_backporch 				 = 16;//16
	 params->dsi.vertical_frontporch				 = 20;//20
	 params->dsi.vertical_active_line				 = FRAME_HEIGHT; 
	 
	 params->dsi.horizontal_sync_active 					 = 10;//10
	 params->dsi.horizontal_backporch				 = 10;//70;50
	 params->dsi.horizontal_frontporch				 = 65;//80;60
	 params->dsi.horizontal_blanking_pixel				 = 65;//100;80
	 params->dsi.horizontal_active_pixel					 = FRAME_WIDTH;  
	 
	 // Bit rate calculation
	 /*params->dsi.pll_div1=1;	 // div1=0,1,2,3;div1_real=1,2,4,4
	 params->dsi.pll_div2=1;	 // div2=0,1,2,3;div2_real=1,2,4,4
	 params->dsi.fbk_sel=1; 	  // fbk_sel=0,1,2,3;fbk_sel_real=1,2,4,4
	 params->dsi.fbk_div =32;//28	 30  //ox1c=28	 // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	 //18--28*/
		//	params->dsi.HS_PRPR=3;
		//params->dsi.CLK_HS_POST=22;
		//params->dsi.DA_HS_EXIT=20;
	
			params->dsi.PLL_CLOCK=200;
				//params->dsi.ssc_range=8;
				params->dsi.ssc_disable=1;
//PLL_CLOCK*2/()/()/2/24
				//params->dsi.esd_check_enable = 1;
				//params->dsi.customization_esd_check_enable = 1;
				//params->dsi.lcm_esd_check_table[0].cmd = 0x0a;  
				//params->dsi.lcm_esd_check_table[0].count = 1;
				//params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;  	

}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(20); 
	SET_RESET_PIN(0);
	MDELAY(20); 
	
	SET_RESET_PIN(1);
	MDELAY(200);

	//lcm_init_registers();
	#if defined(BUILD_LK) 
	 	printf("otm8019, lcm_init\n");
	 #else
	 	printk("otm8019, lcm_init\n");
	#endif 
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}



static void lcm_suspend(void)
{
/*
       unsigned int data_array[16];

       data_array[0]=0x00280500; // Display Off^M
       dsi_set_cmdq(data_array, 1, 1);
               
       data_array[0] = 0x00100500; // Sleep In^M
       dsi_set_cmdq(data_array, 1, 1);

       SET_RESET_PIN(1);
       SET_RESET_PIN(0);
       MDELAY(1);
       SET_RESET_PIN(1);
*/
/*	SET_RESET_PIN(1);
	MDELAY(20); 
	SET_RESET_PIN(0);
	MDELAY(20); 
	
	SET_RESET_PIN(1);
	MDELAY(200);
*/
push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
/*
   //1 do lcm init again to solve some display issue^M
       SET_RESET_PIN(1);
       SET_RESET_PIN(0);
       MDELAY(1);

       SET_RESET_PIN(1);
       MDELAY(20);

       init_lcm_registers();
*/
	lcm_init();

//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id0,id1,id=0;
        unsigned char buffer[5];
        unsigned int array[16];

	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

        array[0] = 0x00053700;          // read id return two byte,version and id
        dsi_set_cmdq(array, 1, 1);

        read_reg_v2(0xA1, buffer, 5);   //018B8019
        id0 = buffer[2];
        id1 = buffer[3];
        id = (id0 << 8) | id1;

    #ifdef BUILD_LK
                printf("%s,LK OTM8019 id = 0x%08x\n", __func__, id);
    #else
                printk(KERN_ERR "%s,LK OTM8019 id = 0x%08x\n", __func__, id);  
    #endif

    if(id == LCM_ID_OTM8019)
    {
	return 1;
    }
    else
    {
        return 0;
    }

}



LCM_DRIVER zaw700_otm8019_fwvga_dsi_vdo_boyi_lcm_drv = 
{
        .name			= "zaw700_otm8019_fwvga_dsi_vdo_boyi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,	
#if (LCM_DSI_CMD_MODE)
	//.set_backlight	= lcm_setbacklight,
     //  .update         = lcm_update,
#endif
    };
