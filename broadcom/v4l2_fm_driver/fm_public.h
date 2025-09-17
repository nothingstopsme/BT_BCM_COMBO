/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.


 *  Copyright (C) 2009-2014 Broadcom Corporation
 */


/************************************************************************************
 *
 *  Filename:      fm_public.h
 *
 *  Description:   FM Driver public header file.
 *
 ***********************************************************************************/

#ifndef _FM_PUBLIC_H
#define _FM_PUBLIC_H

/*******************************************************************************
**  Constants & Macros
*******************************************************************************/

/* Band types */
#define FM_BAND_EUROPE      0
#define FM_BAND_JAPAN       1
#define FM_BAND_NA          2
#define FM_BAND_RUSSIAN     3
#define FM_BAND_CHINA       4
#define FM_BAND_ITALY       5
#define FM_BAND_MAX         FM_BAND_ITALY


#define    FM_RDS_BIT       0x01
#define    FM_RBDS_BIT      0x02
#define    FM_AF_BIT        0x04

/* FM audio output mode */
enum
{
    FM_AUTO_MODE = 1,   /* auto blend by default */
    FM_STEREO_MODE,     /* manual stereo switch */
    FM_MONO_MODE,       /* manual mono switch */
    FM_SWITCH_MODE      /* auto stereo, and switch activated */
};

/* FM audio routing configuration */
#define FM_AUDIO_NONE       0x00  /* No FM audio output */
#define FM_AUDIO_DAC        0x01  /* routing FM over analog output */
#define FM_AUDIO_I2S        0x02  /* routing FM over digital (I2S) output */
#define FM_AUDIO_BT_MONO    0x04  /* routing FM over SCO */
#define FM_AUDIO_BT_STEREO  0x08  /* routing FM over BT Stereo */

#define     FM_DEEMPHA_50U      0       /* 6th bit in FM_AUDIO_CTRL0 set to 0, Europe default */
#define     FM_DEEMPHA_75U      (1<<6)  /* 6th bit in FM_AUDIO_CTRL0 set to 1, US  default */

#define  FM_STEP_50KHZ      0x00
#define  FM_STEP_100KHZ     0x01
#define  FM_STEP_200KHZ     0x02

#define  CHL_SPACE_ONE      0x01
#define  CHL_SPACE_TWO      0x02
#define  CHL_SPACE_FOUR     0x04



#define FM_GET_FREQ(x) ((unsigned short) ((x * 10) - 64000))
#define FM_SET_FREQ(x) ((unsigned int) ((x + 64000)/10))
enum
{
    FM_RDS,
    FM_RBDS
};

#ifndef FALSE
#define FALSE  0
#endif

#ifndef TRUE
#define TRUE  1
#endif

/* RDS data query type;
 * the values also correspond to bit positions in the event mask
 * for indicate the availability of the associated rds data
 */
enum RdsQueryType {
    GET_EVENT_MASK = 0,
    GET_PI_CODE = 1,
    GET_TP_CODE,
    GET_PTY_CODE,
    GET_TA_CODE,
    GET_MS_CODE,
    GET_PS_CODE,
    GET_RT_MSG,
    GET_CT_DATA,
    GET_TMC_CHANNEL,
};

#define RDS_QUERY_TYPE_TO_EVENT_MASK(type) (((__u32)1) << (type-1))

#define RDS_EVENT_PI_CODE RDS_QUERY_TYPE_TO_EVENT_MASK(GET_PI_CODE)
#define RDS_EVENT_TP RDS_QUERY_TYPE_TO_EVENT_MASK(GET_TP_CODE)
#define RDS_EVENT_PTY RDS_QUERY_TYPE_TO_EVENT_MASK(GET_PTY_CODE)
#define RDS_EVENT_TA RDS_QUERY_TYPE_TO_EVENT_MASK(GET_TA_CODE)
#define RDS_EVENT_MS RDS_QUERY_TYPE_TO_EVENT_MASK(GET_MS_CODE)
#define RDS_EVENT_PS RDS_QUERY_TYPE_TO_EVENT_MASK(GET_PS_CODE)
#define RDS_EVENT_RT RDS_QUERY_TYPE_TO_EVENT_MASK(GET_RT_MSG)
#define RDS_EVENT_CT RDS_QUERY_TYPE_TO_EVENT_MASK(GET_CT_DATA)

#endif
