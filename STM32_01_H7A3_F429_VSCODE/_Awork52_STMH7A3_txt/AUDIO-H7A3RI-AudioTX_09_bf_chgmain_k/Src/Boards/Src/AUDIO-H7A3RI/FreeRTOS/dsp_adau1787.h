/**
 * @file      dsp_adau1787.h
 *
 *
 * @brief     
 *
 * @author    Decawave Applications
 *
 * @attention Copyright (c) 2021 - 2022, Qorvo US, Inc.
 * All rights reserved
 * Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions, and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 * 3. You may only use this software, with or without any modification, with an
 *  integrated circuit developed by Qorvo US, Inc. or any of its affiliates
 *  (collectively, "Qorvo"), or any module that contains such integrated circuit.
 * 4. You may not reverse engineer, disassemble, decompile, decode, adapt, or
 *  otherwise attempt to derive or gain access to the source code to any software
 *  distributed under this license in binary or object code form, in whole or in
 *  part.
 * 5. You may not use any Qorvo name, trademarks, service marks, trade dress,
 *  logos, trade names, or other symbols or insignia identifying the source of
 *  Qorvo's products or services, or the names of any of Qorvo's developers to
 *  endorse or promote products derived from this software without specific prior
 *  written permission from Qorvo US, Inc. You must not call products derived from
 *  this software "Qorvo", you must not have "Qorvo" appear in their name, without
 *  the prior permission from Qorvo US, Inc.
 * 6. Qorvo may publish revised or new version of this license from time to time.
 *  No one other than Qorvo US, Inc. has the right to modify the terms applicable
 *  to the software provided under this license.
 * THIS SOFTWARE IS PROVIDED BY QORVO US, INC. "AS IS" AND ANY EXPRESS OR IMPLIED
 *  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. NEITHER
 *  QORVO, NOR ANY PERSON ASSOCIATED WITH QORVO MAKES ANY WARRANTY OR
 *  REPRESENTATION WITH RESPECT TO THE COMPLETENESS, SECURITY, RELIABILITY, OR
 *  ACCURACY OF THE SOFTWARE, THAT IT IS ERROR FREE OR THAT ANY DEFECTS WILL BE
 *  CORRECTED, OR THAT THE SOFTWARE WILL OTHERWISE MEET YOUR NEEDS OR EXPECTATIONS.
 * IN NO EVENT SHALL QORVO OR ANYBODY ASSOCIATED WITH QORVO BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 */

#ifndef __DSP_AUAU1787_H
#define __DSP_AUAU1787_H

/*dsp*/
#define USER_DSP_ADDR				(0x28)	
#define DSP_PWR_CTRL				PIN_PA27   //Dout

/*dsp configure*/
#define USER_DSP_VOLUME_STEPS_LENGTGH		17
#define USER_DSP_VOLUME_0_DB_STEP			(0x0A)    //step 10
#define USER_DSP_VOLUME_4_DB_STEP			(0x0C)    //step 12
#define USDER_DSP_VOLUME_DEFAULT_STEP		USER_DSP_VOLUME_0_DB_STEP
#define DSP_VOLUME_MUTE						(0xFF)
#define ADAU1787_DAC_VOL0_REG				(0xC03C)		//L
#define ADAU1787_DAC_VOL1_REG				(0xC03D)		//R
#define USER_DSP_DAC_L_CHANNEL_VOL_REG      ADAU1787_DAC_VOL0_REG
#define USER_DSP_DAC_R_CHANNEL_VOL_REG      ADAU1787_DAC_VOL1_REG
#define ADAU1787_DMIC_VOL0_REG				(0xC032)
#define ADAU1787_DMIC_VOL1_REG				(0xC033)
#define ADAU1787_ADC2_VOL_REG				(0xC01F)         //bt to dsp 
#define ADAU1787_ADC3_VOL_REG				(0xC020)		 //bt to dsp 
#define USER_BT_A_L_OUT_REG					ADAU1787_ADC2_VOL_REG
#define USER_BT_A_R_OUT_REG					ADAU1787_ADC3_VOL_REG
#define USER_BT_A_OUT_GAIN_DATA				(0x40)			 //adau adc2/adc3 gain data
#define USER_DSP_MIC_IN_GAIN_DATA			(0x20)			 //adau mic1/mic2 gain data
#define USER_DSP_MIC_MODE_MIC_IN_GAIN_DATA	(0x20)			 //adau mic1/mic2 gain data
#define USER_DSP_MULT_MODE_MIC_IN_GAIN_DATA	(0x1A)			 //adau mic1/mic2 gain data

#define USER_DSP_GAIN_DATA_LENGTH_1			(4)
#define USER_DSP_GAIN_DATA_LENGTH_2			(8)
#define USER_DSP_MIC_GAIN_STEP_NEGATIVE_5DB	(4)
#define USER_DSP_MIC_GAIN_STEP_DEFAULT		(USER_DSP_MIC_GAIN_STEP_NEGATIVE_5DB)
#define USER_DSP_MIC_VOLUME_DATA_OFFSET		(0x28)			//9dB

#define USER_DSP_PWR_REG					(0xC008)		//DSP POWER
#define USER_DSP_PWR_ON_DATA				(0x01)			//DSP POWER ON
#define USER_DSP_PWR_OFF_DATA				(0x00)			//DSP POWER OFF

#define USER_DSP_ALL_EQ_NUM			(14)


#define USER_DSP_EQ_STEP_LENGTH		(13)
#define USER_DSP_EQ_DATA_LENGTH		(20)
#define USER_DSP_EQ_DEFAULT_STEP	(USER_DSP_EQ_STEP_LENGTH/2)
#define USER_DSP_EQ_L_CHANNEL		(0)
#define USER_DSP_EQ_R_CHANNEL		(1)


#define USER_DSP_LIMITER_STEP_LENGTH		(9)
#define USER_DSP_LIMITER_STEP_OFF			(0)
#define USER_DSP_LIMITER_L_CHANNEL			(0)
#define USER_DSP_LIMITER_R_CHANNEL			(1)
#define USER_DSP_LIMETER_LINK_STATUS		(0)
#define USER_DSP_LIMITER_STEP_DEFAULT		USER_DSP_LIMITER_STEP_OFF
#define USER_DSP_LIMITER_DATA_LENGTH		(136)


#define USER_DSP_DB_CHECK_DATA_LENGTH		(4)
#define USER_DSP_DB_CHECK_REG_NUM			(4)
//#define USER_DSP_DB_CHECK_REPORT_INTERVAL	(1000)//ms

#define USER_DSP_MODE_BT_MODE				1
#define USER_DSP_MODE_MIC_MODE				2
#define USER_DSP_MODE_MULT_MODE				3				//BT&MIC
#define USER_DSP_MODE_RESUME_MODE			4				//BT&MIC
#define USER_DSP_MODE_DEFAULT_MODE			0

//these regs address is not const , changed with new dsp version
#define USER_DSP_VERSION	(519)

#define USER_DSP_L_EQ_1_60HZ_REG	(0x22F8)
#define USER_DSP_L_EQ_2_140HZ_REG	(0x2320)
#define USER_DSP_L_EQ_3_330HZ_REG	(0x2364)
#define USER_DSP_L_EQ_4_770HZ_REG	(0x2394)
#define USER_DSP_L_EQ_5_1K8HZ_REG	(0x23BC)
#define USER_DSP_L_EQ_6_4K3HZ_REG	(0x23D0)
#define USER_DSP_L_EQ_7_10KHZ_REG	(0x23F8)

#define USER_DSP_R_EQ_1_60HZ_REG	(0x230C)
#define USER_DSP_R_EQ_2_140HZ_REG	(0x2334)
#define USER_DSP_R_EQ_3_330HZ_REG	(0x2350)
#define USER_DSP_R_EQ_4_770HZ_REG	(0x2380)
#define USER_DSP_R_EQ_5_1K8HZ_REG	(0x23A8)
#define USER_DSP_R_EQ_6_4K3HZ_REG	(0x23E4)
#define USER_DSP_R_EQ_7_10KHZ_REG	(0x240C)

#define USER_DSP_LIMITER_L_CHANNEL_REG	(0x2148)
#define USER_DSP_LIMITER_R_CHANNEL_REG	(0x21FC)

#define USER_DSP_DB_CHECK_IP_L_REG	(0x2378)
#define USER_DSP_DB_CHECK_IP_R_REG	(0x237C)
#define USER_DSP_DB_CHECK_OP_L_REG	(0x2600)
#define USER_DSP_DB_CHECK_OP_R_REG	(0x2604)

#define USER_DSP_MIC_GAIN_OP_L_REG_1	(0x20E8)
#define USER_DSP_MIC_GAIN_OP_L_REG_2	(0x97E8)
#define USER_DSP_MIC_GAIN_OP_R_REG_1	(0x20DC)
#define USER_DSP_MIC_GAIN_OP_R_REG_2	(0x97E0)

#define USER_DSP_BT_GAIN_OP_L_REG	(0x20D4)
#define USER_DSP_BT_GAIN_OP_R_REG	(0x20D8)

#define USER_DSP_DAC_GAIN_OP_L_REG_1	(0x2608)
#define USER_DSP_DAC_GAIN_OP_L_REG_2	(0x97F0)
#define USER_DSP_DAC_GAIN_OP_R_REG_1	(0x2614)
#define USER_DSP_DAC_GAIN_OP_R_REG_2	(0x97F8)





#define DSP_DATA_COPY_HERE
#ifdef DSP_DATA_COPY_HERE
//↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓

/*dsp array*/
//Please add this define value to conf_board.h
#define DSP_REG_DATA_LENGTH		226
#define DSP_PROGRAM_DATA_5000_LENGTH	1735
#define DSP_PARAMETER_DATA_2000_LENGTH	52
#define DSP_REG_DATA_2_LENGTH		6

#endif //dsp data copy 

#endif // CONF_BOARD_H
