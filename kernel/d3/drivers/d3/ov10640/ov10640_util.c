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
#include "ov10640_util.h"



u16 ov10640_digital_gain_to_register(u32 digital_gain)
{
	UInt32 dGainInt, dGainFract;
	UInt16 nValue;

	dGainInt = (digital_gain / 1000);
	dGainFract = (digital_gain % 1000);
	dGainFract = (dGainFract * 256) / 1000;
	nValue = ((dGainInt & 0x3F) << 8) | (dGainFract & 0xFF);
	return nValue;
}


u8 ov10640_again_to_exponent(struct ov10640 *self, u32 ag_gain)
{
	if (ag_gain > AG_EIGHT) {
		dev_warn(self->dev,
			 "invalid analog gain value! (1,2,4, or 8) but saw %u"
			 " setting to 8"
			 ,
			 ag_gain);
		ag_gain = AG_EIGHT;
	}
	switch(ag_gain) {
	case AG_ONE:
		return 0;
		break;
	case AG_TWO:
		return 1;
		break;
	case AG_FOUR:
		return 2;
	case AG_EIGHT:
		return 3;
		break;
	}
	return 0;
}


int Set_HDR_ExpGain_Params(struct ae_calculation_output* AE_PrmsOut)
{
    UInt32 exp_rowL, exp_rowS, exp_rowVS;
    UInt32 total_gainS, total_gainVS;
    UInt64 total_exp_rowL,total_exp_rowS,total_exp_rowVS;
    UInt32 row_time_ns;
    UInt32 max_gamm_limit_thr;
    UInt32 l2vs_adapt_ratio;
    UInt64 total_exp_long;
    /*UInt32 l2s_ratio; */
    UInt32 add_gain;
    UInt32 max_gamma_limit = 0;

    /* /\*Set dgain in the ISP *\/ */
    /* AE_PrmsOut->digitalGain  =  AE_PrmsOut->dgain; */

    /*Convert rows from float us to UInt32 ns */
    /* row_time_ns = (UInt32)(row_time * PREC_MULT); */
    row_time_ns = (UInt32)(OV10640_MIN_EXPO * PREC_MULT);


/* #ifdef FLICKER_CORRECTION_ON */
/*     total_exp_rowL = AE_PrmsOut->again * AE_PrmsOut->shutter; */
/*     AE_PrmsOut->shutter = 10000; */
/*     AE_PrmsOut->again = total_exp_rowL / AE_PrmsOut->shutter; */
/*     if(AE_PrmsOut->again < MIN_LVS_DGAIN) */
/*     { */
/*        AE_PrmsOut->again = MIN_LVS_DGAIN; */
/*     } */
/* #endif */

    /* Convert exposure time in ns to rows. */
    exp_rowL = (AE_PrmsOut->shutter * PREC_MULT) / row_time_ns;

    add_gain = (UInt32)(((UInt64)AE_PrmsOut->shutter * PREC_MULT * MULT256) / (exp_rowL * row_time_ns));
    AE_PrmsOut->again = (AE_PrmsOut->again * add_gain) / MULT256;

    /*Calculate total exposure  ((exp time in row) * (gain x*1000)) */
    total_exp_rowL = exp_rowL*AE_PrmsOut->again;

/* #ifdef SMOOTH_L2VS_RATIO_CHANGE */
/*     UInt32 temp_L2VS_ratio; */
/*     total_exp_rowVS = (UInt32)((total_exp_rowL * ROWS_MULT32 + MULT256) / MULT256); /\* 1x = 256 *\/ */

/*     temp_L2VS_ratio = 0x10000; */
/*     if (total_exp_rowVS < VS_MIN_TOTAL_EXP_ROW32) */
/*     { */
/*         temp_L2VS_ratio = (UInt32)(((total_exp_rowL * MULT256) * ROWS_MULT32) / VS_MIN_TOTAL_EXP_ROW32); */
/*     } */

/*     temp_L2VS_ratio = ((*last_set_L2VS_ratio * (MULT256-FILT_RATIO)) + (temp_L2VS_ratio * FILT_RATIO)) / MULT256; */
/*     *last_set_L2VS_ratio = temp_L2VS_ratio; */

/*     if (total_exp_rowVS < VS_MIN_TOTAL_EXP_ROW32) */
/*     { */
/*         total_exp_rowL = (temp_L2VS_ratio * VS_MIN_TOTAL_EXP_ROW32) / (MULT256 * ROWS_MULT32); */
/*         exp_rowL = total_exp_rowL/AE_PrmsOut->again; */
/*     } */
/*     Vps_printf("<<--AEOUT:R Temp ExpVS =  %f %f %lu %lu\n", (float)*last_set_L2VS_ratio/256.0, (float)temp_L2VS_ratio/256.0, total_exp_rowVS, (UInt32)total_exp_rowL); */
/* #endif */

    /*Set Long exposure time in rows*32 */
    AE_PrmsOut->exposureTime = exp_rowL * ROWS_MULT32;

    /*Deal with sensor gain Here!!! */
    /* sensor gain < 1.35x >> Compensate with exposure time in rows */
    if (AE_PrmsOut->again <= MIN_LVS_DGAIN)
    {
        AE_PrmsOut->convSensorGainFlag = CG_GAIN_0FF;
        AE_PrmsOut->analogGain = AG_ONE;
        AE_PrmsOut->sensorDGain = MIN_LVS_DGAIN;
        AE_PrmsOut->exposureTime = (UInt32)((((UInt64)exp_rowL * AE_PrmsOut->again * ROWS_MULT32) / MIN_LVS_DGAIN));
        add_gain = AE_PrmsOut->exposureTime*MULT256 / ((AE_PrmsOut->exposureTime / ROWS_MULT32)*ROWS_MULT32);
        AE_PrmsOut->exposureTime = (AE_PrmsOut->exposureTime / ROWS_MULT32)*ROWS_MULT32;
        AE_PrmsOut->sensorDGain = (AE_PrmsOut->sensorDGain * add_gain) / MULT256;
    }

    /* 1.35x < sensor gain < 2.7x (1.35 *2) */
    if ((AE_PrmsOut->again > MIN_LVS_DGAIN) && (AE_PrmsOut->again < (MIN_LVS_DGAIN * AG_TWO)))
    {
        AE_PrmsOut->convSensorGainFlag = CG_GAIN_0FF;
        AE_PrmsOut->analogGain = AG_ONE;
        AE_PrmsOut->sensorDGain = AE_PrmsOut->again;
    }

    /* 2.7x (1.35 *2) < sensor gain < 3.47x (1.35*2.57) */
    if ((AE_PrmsOut->again >= (MIN_LVS_DGAIN * AG_TWO)) &&
        (AE_PrmsOut->again < ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN) / MULT256))))
    {
        AE_PrmsOut->convSensorGainFlag = CG_GAIN_0FF;
        AE_PrmsOut->analogGain = AG_TWO;
        AE_PrmsOut->sensorDGain = AE_PrmsOut->again / AG_TWO;
    }

    /* 3.47x (1.35*2.57) < sensor gain < 6.94x (1.35*2.57*2) */
    if ((AE_PrmsOut->again >= ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN) / MULT256))) &&
        (AE_PrmsOut->again < ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_TWO) / MULT256))))
    {
         AE_PrmsOut->convSensorGainFlag = CG_GAIN_0N;
         AE_PrmsOut->analogGain = AG_ONE;
         AE_PrmsOut->sensorDGain = (AE_PrmsOut->again * MULT256) / OV10640_CONV_GAIN;
    }

    /* 6.94x (1.35*2.57*2) < sensor gain < 13.88x (1.35*2.57*4) */
    if ((AE_PrmsOut->again >= ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_TWO) / MULT256))) &&
        (AE_PrmsOut->again < ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_FOUR) / MULT256))))
    {
        AE_PrmsOut->convSensorGainFlag = CG_GAIN_0N;
        AE_PrmsOut->analogGain = AG_TWO;
        AE_PrmsOut->sensorDGain = (AE_PrmsOut->again * MULT256) / (OV10640_CONV_GAIN * AG_TWO);
    }

    /* 13.88x(1.35*2.57*4) < sensor gain < 27.75x (1.35*2.57*8) */
    if ((AE_PrmsOut->again >= ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_FOUR) / MULT256))) &&
        (AE_PrmsOut->again < ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_EIGHT) / MULT256))))
    {
        AE_PrmsOut->convSensorGainFlag = CG_GAIN_0N;
        AE_PrmsOut->analogGain = AG_FOUR;
        AE_PrmsOut->sensorDGain = (AE_PrmsOut->again * MULT256) / (OV10640_CONV_GAIN * AG_FOUR);
    }

    /* sensor gain > 27.75x (1.35*2.57*4) >> increase sensor dgain to max; */
    if (AE_PrmsOut->again >= ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_EIGHT) / MULT256)))
    {
        AE_PrmsOut->convSensorGainFlag = CG_GAIN_0N;
        AE_PrmsOut->analogGain = AG_EIGHT;
        AE_PrmsOut->sensorDGain = (AE_PrmsOut->again * MULT256) / (OV10640_CONV_GAIN * AG_EIGHT);
    }

    /*Clip Long sensor dgain to min */
    /* There is no need to clip Short and Very Short gain too, */
    /* because there are round to near upper values */
    if (AE_PrmsOut->sensorDGain < MIN_LVS_DGAIN)
    {
        AE_PrmsOut->sensorDGain = MIN_LVS_DGAIN;
    }

    /*////////////////////////////////////////// Set Short exposure parameters ///////////////////////////////////////////////// */
    if (!max_gamma_limit)
    {
        max_gamm_limit_thr = L2S_MAX_RATIO;
    }
    else
    {
        max_gamm_limit_thr = max_gamma_limit + (max_gamma_limit * ADD_TO_GAMM_LIM) / MULT256;
        max_gamm_limit_thr = (max_gamm_limit_thr * 16) / L2S_DEF_RATIO;

        if (max_gamm_limit_thr < MULT256)
        {
            max_gamm_limit_thr = MULT256;
        }

        if (max_gamm_limit_thr > L2S_MAX_RATIO)
        {
            max_gamm_limit_thr = L2S_MAX_RATIO;
        }
    }

    /*Set maximum ratio for test */
    total_exp_rowS = (total_exp_rowL * MULT256 + max_gamm_limit_thr) / max_gamm_limit_thr; /* 16/6.5 = 2.461 max 2.461 * 256 = 630!!! */

    /*Deal with sensor gain Here!!! */
    AE_PrmsOut->convSensorGainFlagS = CG_GAIN_0FF; /*conversion gain is not used for Short frame */

    /*Initial short exposure with long exposure */
    exp_rowS = exp_rowL;

    /*Calc total gain for short */
    total_gainS = (UInt32)((total_exp_rowS + exp_rowS) / exp_rowS);

    /*Set exposure time in rows*32 for Short */
    AE_PrmsOut->exposureTimeS = exp_rowS * ROWS_MULT32;

    /* sensor gain < 3x (1.5 *2) */
    if (total_gainS <= (MIN_S_DGAIN * AG_TWO))
    {
        AE_PrmsOut->analogGainS = AG_TWO;
        AE_PrmsOut->sensorDGainS = MIN_S_DGAIN;
        AE_PrmsOut->exposureTimeS = (UInt32)((((UInt64)exp_rowS * total_gainS) * ROWS_MULT32 + (AG_TWO * MIN_S_DGAIN)) / (AG_TWO * MIN_S_DGAIN));
        if (AE_PrmsOut->exposureTimeS < S_MIN_ROWS32)
        {
            AE_PrmsOut->exposureTimeS = S_MIN_ROWS32;
        }
        else
        {
            add_gain = AE_PrmsOut->exposureTimeS*MULT256 / ((AE_PrmsOut->exposureTimeS / ROWS_MULT32)*ROWS_MULT32);
            AE_PrmsOut->exposureTimeS = (AE_PrmsOut->exposureTimeS / ROWS_MULT32) * ROWS_MULT32;
            AE_PrmsOut->sensorDGainS = (AE_PrmsOut->sensorDGainS * add_gain) / MULT256;
        }
    }

    /* 3x (1.5 *2) < sensor gain < 6x (1.5 *4) */
    if ((total_gainS > (MIN_S_DGAIN * AG_TWO)) && (total_gainS < (MIN_S_DGAIN * AG_FOUR)))
    {
        AE_PrmsOut->analogGainS = AG_TWO;
        AE_PrmsOut->sensorDGainS = (total_gainS + AG_TWO) / AG_TWO;
    }

    /* 6x (1.5 *4) < sensor gain < 12x (1.5 *8) */
    if ((total_gainS >= (MIN_S_DGAIN * AG_FOUR)) && (total_gainS < (MIN_S_DGAIN * AG_EIGHT)))
    {
        AE_PrmsOut->analogGainS = AG_FOUR;
        AE_PrmsOut->sensorDGainS = (total_gainS + AG_FOUR) / AG_FOUR;
    }

    /* 12x (1.5 *4) < sensor gain >> set sensor dgain as needed for keep the ratio in limits */
    if (total_gainS >(MIN_S_DGAIN * AG_EIGHT))
    {
        AE_PrmsOut->analogGainS = AG_EIGHT;
        AE_PrmsOut->sensorDGainS = (total_gainS + AG_EIGHT) / AG_EIGHT;
    }

    /*//////////////////////////////////// Set Very Short exposure parameters ////////////////////////////////////////// */

    /*AE provides long2Vshort_ratio [1x : 256x]  256 = 1x */
    /*total_exp_rowVS = (UInt32)(((total_exp_rowL * MULT256)  * ROWS_MULT32 + AE_PrmsOut->long2Vshort_ratio) / AE_PrmsOut->long2Vshort_ratio); */

    /*Set maximum ratio for test (256) and calculate total exposure VS in rows*32 */
    /*total_exp_rowVS = (UInt32)(((total_exp_rowL * MULT256)  * ROWS_MULT32 + MULT256*MULT256) / (MULT256*MULT256));  1x = 256  */
    l2vs_adapt_ratio = ((L2S_DEF_RATIO*max_gamm_limit_thr + 16) / 16);
    total_exp_rowVS = (UInt32)(((total_exp_rowL * MULT256)  * ROWS_MULT32 + l2vs_adapt_ratio) / l2vs_adapt_ratio); /* 1x = 256 */

    /*Initial short exposure with max possible */
    exp_rowVS = VS_MAX_ROWS32; /*max 3.95 rows * 32 = 126 */

    /*Calc total gain for Short */
    total_gainVS = (total_exp_rowVS + exp_rowVS ) / exp_rowVS;

    /* Set Very Short exposure */
    AE_PrmsOut->exposureTimeVS =  exp_rowVS;

    /*Deal with sensor gain Here!!! */
    /* sensor gain < 1.35x */
    if (total_gainVS <= MIN_LVS_DGAIN)
    {
        AE_PrmsOut->convSensorGainFlagVS = CG_GAIN_0FF;
        AE_PrmsOut->analogGainVS = AG_ONE;
        AE_PrmsOut->sensorDGainVS = MIN_LVS_DGAIN;
        AE_PrmsOut->exposureTimeVS = (AE_PrmsOut->exposureTimeVS * total_gainVS + MIN_LVS_DGAIN) / MIN_LVS_DGAIN;
        if (AE_PrmsOut->exposureTimeVS < VS_MIN_ROWS32) /*min 0.55 rows*32 = 18  value */
        {
            AE_PrmsOut->exposureTimeVS = VS_MIN_ROWS32; /* Clip to min limit will decrease the ratio */
                                         /* ratio always will be < 256 */

            total_exp_long = (UInt64)AE_PrmsOut->exposureTime * AE_PrmsOut->sensorDGain * AE_PrmsOut->analogGain;
            if(AE_PrmsOut->convSensorGainFlag)
            {
                total_exp_long *= OV10640_CONV_GAIN;
            }
            else
            {
                total_exp_long *= MULT256;
            }

            /*AE_PrmsOut->long2Vshort_ratio = (UInt32)(total_exp_long/(AE_PrmsOut->exposureTimeVS * AE_PrmsOut->sensorDGainVS)); */
            /*AE_PrmsOut->long2Short_ratio = (UInt32)(sqrt((double)AE_PrmsOut->long2Vshort_ratio/MULT256)*MULT256); */
        }
    }

    /* 1.35x < sensor gain < 2.7x (1.35*2) */
    if ((total_gainVS > MIN_LVS_DGAIN) &&
        (total_gainVS < (MIN_LVS_DGAIN * AG_TWO)))
    {
        AE_PrmsOut->convSensorGainFlagVS = CG_GAIN_0FF;
        AE_PrmsOut->analogGainVS = AG_ONE;
        AE_PrmsOut->sensorDGainVS = total_gainVS;
    }

    /* 2.7x < sensor gain < 3.47x */
    if ((total_gainVS >= (MIN_LVS_DGAIN * AG_TWO)) && (total_gainVS < ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN) / MULT256))))
    {
        AE_PrmsOut->convSensorGainFlagVS = CG_GAIN_0FF;
        AE_PrmsOut->analogGainVS = AG_TWO;
        AE_PrmsOut->sensorDGainVS = (total_gainVS + AG_TWO) / AG_TWO;
    }

    /* 3.47x < sensor gain < 6.94x */
    if ((total_gainVS >= ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN) / MULT256))) &&
        (total_gainVS < ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_TWO) / MULT256))))
    {
        AE_PrmsOut->convSensorGainFlagVS = CG_GAIN_0N;
        AE_PrmsOut->analogGainVS = AG_ONE;
        AE_PrmsOut->sensorDGainVS = (total_gainVS * MULT256 + OV10640_CONV_GAIN) / OV10640_CONV_GAIN;
    }

    /* 6.94x < sensor gain < 13.88x */
    if ((total_gainVS >= ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_TWO) / MULT256))) &&
        (total_gainVS < ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_FOUR) / MULT256))))
    {
        AE_PrmsOut->convSensorGainFlagVS = CG_GAIN_0N;
        AE_PrmsOut->analogGainVS = AG_TWO;
        AE_PrmsOut->sensorDGainVS = (total_gainVS * MULT256 + (OV10640_CONV_GAIN * AG_TWO)) / (OV10640_CONV_GAIN *AG_TWO);
    }

    /* 13.88x < sensor gain < 27.75x */
    if ((total_gainVS >= ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_FOUR) / MULT256))) &&
        (total_gainVS < ((UInt32)((MIN_LVS_DGAIN * OV10640_CONV_GAIN * AG_EIGHT) / MULT256))))
    {
        AE_PrmsOut->convSensorGainFlagVS = CG_GAIN_0N;
        AE_PrmsOut->analogGainVS = AG_FOUR;
        AE_PrmsOut->sensorDGainVS = (total_gainVS * MULT256 + (OV10640_CONV_GAIN * AG_FOUR)) / (OV10640_CONV_GAIN * AG_FOUR);
    }

    /* sensor gain > 27.75x */
    if (total_gainVS >= ((UInt32)((MIN_LVS_DGAIN*OV10640_CONV_GAIN*AG_EIGHT) / MULT256)))
    {
        AE_PrmsOut->convSensorGainFlagVS = CG_GAIN_0N;
        AE_PrmsOut->analogGainVS = AG_EIGHT;
        AE_PrmsOut->sensorDGainVS = (total_gainVS * MULT256 + (OV10640_CONV_GAIN * AG_EIGHT)) / (OV10640_CONV_GAIN * AG_EIGHT);
    }

    total_exp_rowL = (UInt64)AE_PrmsOut->exposureTime * AE_PrmsOut->sensorDGain * AE_PrmsOut->analogGain;
    if(AE_PrmsOut->convSensorGainFlag)
    {
        total_exp_rowL *= OV10640_CONV_GAIN;
    }
    else
    {
        total_exp_rowL *= MULT256;
    }

    AE_PrmsOut->long2Short_ratio  = (UInt32)(total_exp_rowL / ((UInt64)AE_PrmsOut->exposureTimeS * AE_PrmsOut->sensorDGainS * AE_PrmsOut->analogGainS));

    total_exp_rowVS = (UInt64)AE_PrmsOut->exposureTimeVS * AE_PrmsOut->sensorDGainVS * AE_PrmsOut->analogGainVS;
    if(AE_PrmsOut->convSensorGainFlagVS)
    {
        total_exp_rowVS = (total_exp_rowVS * OV10640_CONV_GAIN) / MULT256;
    }
    AE_PrmsOut->long2Vshort_ratio = (UInt32)(total_exp_rowL / total_exp_rowVS);

    return 0;
}
