/*
 *   Copyright (c) Texas Instruments Incorporated 2012-2015
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file ov10640_utils.h
 *
 */

#ifndef OV10640_UTIL_H_
#define OV10640_UTIL_H_

#include <linux/kernel.h>
#include "ov10640.h"

typedef  unsigned int UInt32;
typedef  int Bool;
typedef unsigned char UInt8;
typedef unsigned short UInt16;
typedef unsigned long long UInt64;
typedef int Int32;

#define TRUE (1)
#define FALSE (0)
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */



/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define USE_GROUP_HOLD_FOR_WB

#define TARGET_BLC_L_S_VS (128)

#define  POS_R  (3)
#define  POS_GR (1)
#define  POS_GB (2)
#define  POS_B  (0)

/*
 * OV10640 Register Offsets.
 */

/** < Register to set the operating mode of sensor : Streaming or Still
 *    capture*/
#define OV10640_SOFTWARE_RESET_REG                  (0x301AU)

#define OV10640_SOFTWARE_RESET_REG_VAL              (0x10D8U)

#define OV10640_OPERATING_MODE_REG_VAL              (0x10D8U)

#define OV10640_CHIP_ID_UPPER_REG                   (0x300AU)
/** < Address of chip ID MSB register */
#define OV10640_CHIP_ID_LOWER_REG                   (0x300BU)
/** <  Address of chip ID LSB register */
#define OV10640_CHIP_ID_REV_ID                      (0x300DU)
/** <  Address of chip ID LSB register */
#define OV10640_CHIP_ID_R1E                         (0xB4U)

#define OV10640_GROUP_LENGTH                        (0x3028U)
#define OV10640_GROUP_CONTROL                       (0x302CU)
#define OV10640_GROUP_OPER_CONTROL                  (0x302FU)

/** < hold function is bit 15 of address */
#define OV10640_LONG_EXP_REG_HOLD                   (0xB0E6U)
#define OV10640_SHORT_EXP_REG_HOLD                  (0xB0E8U)
#define OV10640_VSHORT_EXP_REG_HOLD                 (0xB0EAU)
#define OV10640_ANALOG_GAIN_REG_HOLD                (0xB0EBU)
#define OV10640_DGAIN_L_REG_HOLD                    (0xB0ECU)
#define OV10640_DGAIN_L_FRACT_REG_HOLD              (0xB0EDU)
#define OV10640_DGAIN_S_REG_HOLD                    (0xB0EEU)
#define OV10640_DGAIN_VS_REG_HOLD                   (0xB0F0U)

#define OV10640_NORMALIZATION_REG_HOLD              (0xB1BFU)

#define OV10640_VERT_TOTAL_SIZE_REG_HOLD            (0xB082U)

#define OV10640_WB_GAIN_R_L_REG_HOLD                (0xB1C3U)
#define OV10640_WB_GAIN_GR_L_REG_HOLD               (0xB1C5U)
#define OV10640_WB_GAIN_GB_L_REG_HOLD               (0xB1C7U)
#define OV10640_WB_GAIN_B_L_REG_HOLD                (0xB1C9U)

#define OV10640_WB_GAIN_R_S_REG_HOLD                (0xB1CBU)
#define OV10640_WB_GAIN_GR_S_REG_HOLD               (0xB1CDU)
#define OV10640_WB_GAIN_GB_S_REG_HOLD               (0xB1CFU)
#define OV10640_WB_GAIN_B_S_REG_HOLD                (0xB1D1U)

#define OV10640_WB_GAIN_R_VS_REG_HOLD               (0xB1D3U)
#define OV10640_WB_GAIN_GR_VS_REG_HOLD              (0xB1D5U)
#define OV10640_WB_GAIN_GB_VS_REG_HOLD              (0xB1D7U)
#define OV10640_WB_GAIN_B_VS_REG_HOLD               (0xB1D9U)

#define OV10640_WB_OFFSET_R_L_REG_HOLD              (0xB1DBU)
#define OV10640_WB_OFFSET_GR_L_REG_HOLD             (0xB1DEU)
#define OV10640_WB_OFFSET_GB_L_REG_HOLD             (0xB1E1U)
#define OV10640_WB_OFFSET_B_L_REG_HOLD              (0xB1E4U)

#define OV10640_WB_OFFSET_R_S_REG_HOLD              (0xB1E7U)
#define OV10640_WB_OFFSET_GR_S_REG_HOLD             (0xB1EAU)
#define OV10640_WB_OFFSET_GB_S_REG_HOLD             (0xB1EDU)
#define OV10640_WB_OFFSET_B_S_REG_HOLD              (0xB1F0U)

#define OV10640_WB_OFFSET_R_VS_REG_HOLD             (0xB1F3U)
#define OV10640_WB_OFFSET_GR_VS_REG_HOLD            (0xB1F6U)
#define OV10640_WB_OFFSET_GB_VS_REG_HOLD            (0xB1F9U)
#define OV10640_WB_OFFSET_B_VS_REG_HOLD             (0xB1FCU)


#define OV10640_WB_GAIN_R_L_REG                     (0x31C3U)
#define OV10640_WB_GAIN_GR_L_REG                    (0x31C5U)
#define OV10640_WB_GAIN_GB_L_REG                    (0x31C7U)
#define OV10640_WB_GAIN_B_L_REG                     (0x31C9U)

#define OV10640_WB_GAIN_R_S_REG                     (0x31CBU)
#define OV10640_WB_GAIN_GR_S_REG                    (0x31CDU)
#define OV10640_WB_GAIN_GB_S_REG                    (0x31CFU)
#define OV10640_WB_GAIN_B_S_REG                     (0x31D1U)

#define OV10640_WB_GAIN_R_VS_REG                    (0x31D3U)
#define OV10640_WB_GAIN_GR_VS_REG                   (0x31D5U)
#define OV10640_WB_GAIN_GB_VS_REG                   (0x31D7U)
#define OV10640_WB_GAIN_B_VS_REG                    (0x31D9U)

#define OV10640_WB_OFFSET_R_L_REG                   (0x31DBU)
#define OV10640_WB_OFFSET_GR_L_REG                  (0x31DEU)
#define OV10640_WB_OFFSET_GB_L_REG                  (0x31E1U)
#define OV10640_WB_OFFSET_B_L_REG                   (0x31E4U)

#define OV10640_WB_OFFSET_R_S_REG                   (0x31E7U)
#define OV10640_WB_OFFSET_GR_S_REG                  (0x31EAU)
#define OV10640_WB_OFFSET_GB_S_REG                  (0x31EDU)
#define OV10640_WB_OFFSET_B_S_REG                   (0x31F0U)

#define OV10640_WB_OFFSET_R_VS_REG                  (0x31F3U)
#define OV10640_WB_OFFSET_GR_VS_REG                 (0x31F6U)
#define OV10640_WB_OFFSET_GB_VS_REG                 (0x31F9U)
#define OV10640_WB_OFFSET_B_VS_REG                  (0x31FCU)

/** < hold function is bit 15 of address */
#define OV10640_LONG_EXP_REG                        (0x30E6U)
#define OV10640_SHORT_EXP_REG                       (0x30E8U)
#define OV10640_VSHORT_EXP_REG                      (0x30EAU)
#define OV10640_ANALOG_GAIN_REG                     (0x30EBU)
#define OV10640_DGAIN_L_REG                         (0x30ECU)
#define OV10640_DGAIN_L_FRACT_REG                   (0x30EDU)
#define OV10640_DGAIN_S_REG                         (0x30EEU)
#define OV10640_DGAIN_VS_REG                        (0x30F0U)

#define OV10640_NORMALIZATION_REG                   (0x31BFU)

#define OV10640_VERT_TOTAL_SIZE_REG                 (0x3082U)
#define OV10640_VERT_ACTIVE_SIZE_REG                (0x307EU)

#define OV10640_DLY_IN_MS                           (10U)

#define OV10640_STATUS_LINE_MARKER                  (0xDA)

#define OV10640_STATUS_LINE_FIRST_REG               (0x3000U) /*(0x302cU)*/

#define OV10640_STATUS_LINE_REAL_GAIN               (0x3105U)

#if (OV10640_STATUS_LINE_FIRST_REG > OV10640_STATUS_LINE_REAL_GAIN)
#error "Gain register address shuld falls into status line registers coverage"
#endif

#define OV10640_STATUS_LINE_EXP                     (0x3096U)

#if (OV10640_STATUS_LINE_FIRST_REG > OV10640_STATUS_LINE_EXP)
#error "Exp register address shuld falls into status line registers coverage"
#endif

#define OV10640_STATUS_LINE_DGAIN                   (0x30DCU)

#if (OV10640_STATUS_LINE_FIRST_REG > OV10640_STATUS_LINE_DGAIN)
#error "DGain register address shuld falls into status line registers coverage"
#endif

#define OV10640_STATUS_LINE_REAL_GAIN_SHORT         (0x3107U)
#define OV10640_STATUS_LINE_EXP_SHORT               (0x3098U)

#define OV10640_STATUS_LINE_REAL_GAIN_VERY_SHORT    (0x3109U)
#define OV10640_STATUS_LINE_EXP_VERY_SHORT          (0x309AU)

#define OV10640_FRAME_COUNT                         (0x309CU)

#define OV10640_STATUS_LINE_BOTTOM_FIRST_REG        (0x4000U)
#define OV10640_STATUS_LINE_BOTTOM_HIST_REG         (0x4014U)
#define OV10640_STATUS_LINE_BOTTOM_ACCU_REG         (0x4009U)
#define OV10640_STATUS_LINE_BOTTOM_PIX_COUNT_REG    (0x4011U)

#if (OV10640_STATUS_LINE_BOTTOM_FIRST_REG > OV10640_STATUS_LINE_BOTTOM_HIST_REG)
#error "Histogram register address shuld falls into status line registers coverage"
#endif

#define OV10640_OUTPUT_DATA_FMT_MASK                (0x07U)
#define OV10640_OUTPUT_DATA_FMT_LONG_CH             (0x05U)
#define OV10640_OUTPUT_DATA_FMT_12BIT_COMP          (0x04U)
#define OV10640_OUTPUT_DATA_FMT_16BIT_COMP          (0x03U)
#define OV10640_OUTPUT_DATA_FMT_2x12BIT             (0x01U)
#define OV10640_OUTPUT_DATA_FMT_3x12BIT             (0x00U)

#define OV10640_DCC_CAMERA_ID                       (10640U)

#define OV10640_MIRROR_OFFSET_ENABLE_MASK           (0x2U)
#define OV10640_FLIP_OFFSET_ENABLE_MASK             (0x4U)

#define OV10640_READ_MODE_MIRROR_ENABLE_MASK        (0x4U)
#define OV10640_READ_MODE_FLIP_ENABLE_MASK          (0x8U)

#define OV10640_ISP_CTRL2_MIRROR_ENABLE_MASK        (0x1U)
#define OV10640_ISP_CTRL2_FLIP_ENABLE_MASK          (0x2U)
#ifndef MMS_USECASE
#define EXP_RATIO                                   (32U)
#else
#define EXP_2x12_RATIO                              (16U)
#define EXP_3x12_RATIO                              (256U)
#endif

typedef struct {
    UInt32 nExposureTime;
    UInt32 nAnalogGain;
    UInt32 nDigitalGain;
    Bool   bConvGain;
} ov10640ExpoGainParams;

typedef enum
{
    SEN_WDR_MODE_NONE = 0,
    SEN_WDR_MODE_3x12_12BITS_COMB,
    SEN_WDR_MODE_2x12BITS,
    SEN_WDR_MODE_3x12BITS,
    SEN_WDR_MODE_16BITS,
} ov10640SenWdrMode;

/**
 *  \brief OV10640 driver ov10640ExpoGain object.
 */
typedef struct {
    ov10640ExpoGainParams longExposure;
    ov10640ExpoGainParams shortExposure;
    ov10640ExpoGainParams veryShortExposure;
    UInt32  ExposureTime;
    /**< Sensor Exposure Time */
    UInt32 shortExposureTime;
    /**< Sensor Short Exposure Time used in WDR case */
    UInt32 VeryShortExposureTime;
    /**< Sensor Very Short Exposure Time used in WDR case */
    UInt32 realGain;
    /**< Sensor Gain */
    UInt32 realShortGain;
    /**< Sensor Short Gain */
    UInt32 realVeryShortGain;
    /**< Sensor Very Short Gain */
    UInt32 ispGain;
    /**< ISP Digital Gain */
    UInt32 lsExpRatio;
    /**< Long to Short exposure ratio*/
    UInt32 lvsExpRatio;
    /**< Long to Very Short exposure ratio*/
} ov10640ExpoGain;

/*
typedef Int32 (*ov10640ReadExpGain)(void *frmData,
                                    UInt8 nDshift,
                                    IssSensor_AewbInfo *pAewbInfo);
typedef Int32 (*ov10640ReadWbGain)(void *frmData,
                                    UInt8 nDshift,
                                    UInt32 * pRgbPos,
                                    IssSensor_AewbInfo *pAewbInfo);

typedef struct
{
    ov10640ExpoGain           senExpoGain;
    ov10640ExpoGain           senLastExpGain;
    UInt32                    senWbGain[4];
    UInt32                    rgbPos[4];
    UInt32                    senLastWbGain[4];
    Bool                      isWbGainInSensor;
    UInt8                     regCgAgain;
    UInt8                     nDshift;
    ov10640SenWdrMode         senWdrMode;
    ov10640ReadExpGain        readExpoGain;
    ov10640ReadWbGain         readWbGain;
} ov10640SensorObj;
*/

#define Q16(F) ((UInt32)(0x10000 * (F)))

#define OV10640_SENSOR_ACT
/*#define SMOOTH_L2VS_RATIO_CHANGE */
/*#define MMS_AE_RATIO_DEBUG */
#define PREC_MULT (1000)
#define MULT256 (256)
#define ROWS_MULT32 (32)
#define CG_GAIN_0FF (0)
#define CG_GAIN_0N (1)
#define AG_ONE (1)
#define AG_TWO (2)
#define AG_FOUR (4)
#define AG_EIGHT (8)
#define MIN_LVS_DGAIN (UInt32)(1.35*PREC_MULT) /* fix for dark sun effect */
#define MIN_S_DGAIN (UInt32)(1.5*PREC_MULT) /* fix for dark sun effect */
#define VS_MIN_ROWS32 (UInt32)(0.55*ROWS_MULT32 + 0.5) /*min 0.55 rows*32 */
#define VS_MAX_ROWS32 (UInt32)(3.95*ROWS_MULT32) /*max 3.95 rows * 32 */
#define VS_MIN_TOTAL_EXP_ROW32 (VS_MIN_ROWS32 * MIN_LVS_DGAIN)
#define S_MIN_ROWS32 (UInt32)(4*ROWS_MULT32) /*min 4 rows*32 */
#define L2S_MAX_RATIO (630) /*16/6.5 = 2.461 max 2.461 * 256 = 630 */
#define L2S_DEF_RATIO (UInt32)(6.5*MULT256) /* default long to short pixels ratio */
#define OV10640_CONV_GAIN (659) /*(2.5742x, 256=1x) */
#define SHORT_MAX_GAIN (12000)
#define FILT_RATIO (26) /* 10% --> 256 = 100% */
#define ADD_TO_GAMM_LIM (26) /* 10% --> 256 = 100% */

struct ae_calculation_output {
    UInt32 shutter;                         /* shutter speed, [us] */
    UInt32 again;                           /* analog gain, [EV units] */
    /* UInt32 dgain;                           /\* digital gain, [U16Q8] *\/ */

    /* OV10640 parameters */
    UInt32 exposureTime, analogGain, sensorDGain, convSensorGainFlag, digitalGain;
    UInt32 exposureTimeS, analogGainS, sensorDGainS, convSensorGainFlagS;
    UInt32 exposureTimeVS, analogGainVS, sensorDGainVS,convSensorGainFlagVS;
    /*Long to Short Ratio ratio 265 = 1x */
    UInt32 long2Short_ratio;
    /*Long to Very Short Ratio ratio 265 = 1x */
    UInt32 long2Vshort_ratio;
};

int Set_HDR_ExpGain_Params(struct ae_calculation_output* AE_PrmsOut);
u8 ov10640_again_to_exponent(struct ov10640 *self, u32 ag_gain);
u16 ov10640_digital_gain_to_register(u32 digital_gain);


#endif /* #ifndef OV10640_UTILS_H_  */

