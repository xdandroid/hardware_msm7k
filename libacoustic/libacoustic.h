/*
 * Author: Jbruneaux
 *
 * Description : provide interface between userland and kernel for the 
 * acoustic management
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef ANDROID_HARDWARE_LIB_HTC_ACOUSTIC_H
#define ANDROID_HARDWARE_LIB_HTC_ACOUSTIC_H

#include <utils/threads.h>
#include <stdint.h>

//#define MSM_HTC_ACOUSTIC_WINCE "/dev/htc-acoustic_wince"
#define MSM_HTC_ACOUSTIC_WINCE "/dev/htc-acoustic"

struct msm_audio_path {
    bool bEnableMic;
    bool bEnableDualMic;
    bool bEnableSpeaker;
    bool bEnableHeadset;
};

struct adie_table {
    int table_num;
    char* pcArray;
};

struct htc_voc_cal_table {
    uint16_t* pArray;
    int       size;
};

/* IOCTLs */
#define ACOUSTIC_IOCTL_MAGIC 'p'
#define ACOUSTIC_ARM11_DONE	                    _IOW(ACOUSTIC_IOCTL_MAGIC, 22, unsigned int)

#define ACOUSTIC_UPDATE_ADIE_TABLE              _IOW(ACOUSTIC_IOCTL_MAGIC,  1, struct adie_table* )
#define ACOUSTIC_UPDATE_VOLUME_TABLE            _IOW(ACOUSTIC_IOCTL_MAGIC,  2, uint16_t* )
#define ACOUSTIC_UPDATE_CE_TABLE                _IOW(ACOUSTIC_IOCTL_MAGIC,  3, uint16_t* )
#define ACOUSTIC_UPDATE_AUDIO_PATH_TABLE        _IOW(ACOUSTIC_IOCTL_MAGIC,  4, uint16_t* )
#define ACOUSTIC_GET_HTC_VOC_CAL_FIELD_SIZE     _IOW(ACOUSTIC_IOCTL_MAGIC,  5, uint16_t* )
#define ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE _IOW(ACOUSTIC_IOCTL_MAGIC,  6, struct htc_voc_cal_table* )
#define ACOUSTIC_PCOM_UPDATE_AUDIO              _IOW(ACOUSTIC_IOCTL_MAGIC,  7, uint16_t* )

#define ACOUSTIC_SET_HW_AUDIO_PATH          _IOW(ACOUSTIC_IOCTL_MAGIC,  10, struct msm_audio_path* )

#define TRUE 1
#define FALSE 0

#define SAMP_RATE_INDX_8000	0
#define SAMP_RATE_INDX_11025	1
#define SAMP_RATE_INDX_12000	2
#define SAMP_RATE_INDX_16000	3
#define SAMP_RATE_INDX_22050	4
#define SAMP_RATE_INDX_24000	5
#define SAMP_RATE_INDX_32000	6
#define SAMP_RATE_INDX_44100	7
#define SAMP_RATE_INDX_48000	8

#define AUDIO_HW_IN_SAMPLERATE  8000                 // Default audio input sample rate

/* enable_mask bits */
#define ADRC_ENABLE  0x0001
#define ADRC_DISABLE 0x0000
#define EQ_ENABLE    0x0002
#define EQ_DISABLE   0x0000
#define RX_IIR_ENABLE   0x0004
#define RX_IIR_DISABLE  0x0000

#define AGC_ENABLE     0x0001
#define NS_ENABLE      0x0002
#define TX_IIR_ENABLE  0x0004

#if __cplusplus
extern "C" {
#endif

#include <linux/msm_audio.h>

struct eq_filter_type {
    int16_t gain;
    uint16_t freq;
    uint16_t type;
    uint16_t qf;
};

struct eqalizer {
    uint16_t bands;
    uint16_t params[132];
};

struct rx_iir_filter {
    uint16_t num_bands;
    uint16_t iir_params[48];
};

struct tx_iir {
/*
    uint16_t  cmd_id;
    uint16_t  active_flag;
*/
    uint16_t num_bands;
    uint16_t iir_params[48];
    uint16_t active_flag;
};

struct adrc_filter {
	uint16_t adrc_params[8];
};

struct ns {
/*
        uint16_t  cmd_id;
        uint16_t  ec_mode_new;
*/
        uint16_t  dens_gamma_n;
        uint16_t  dens_nfe_block_size;
        uint16_t  dens_limit_ns;
        uint16_t  dens_limit_ns_d;
        uint16_t  wb_gamma_e;
        uint16_t  wb_gamma_n;
};

struct tx_agc {
/*
        uint16_t  cmd_id;
        uint16_t  tx_agc_param_mask;
*/
        uint16_t  tx_agc_enable_flag;
        uint16_t  static_gain;
        int16_t   adaptive_gain_flag;
        uint16_t  agc_params[17];
};

struct adrc_config {
    uint16_t adrc_band_params[10];
};

struct adrc_ext_buf {
    int16_t buff[196];
};

struct mbadrc_filter {
    uint16_t num_bands;
    uint16_t down_samp_level;
    uint16_t adrc_delay;
    uint16_t ext_buf_size;
    uint16_t ext_partition;
    uint16_t ext_buf_msw;
    uint16_t ext_buf_lsw;
    struct adrc_config adrc_band[5];
    struct adrc_ext_buf  ext_buf;
};

enum CE_audio_devices {
    HEADSET = 0,
    HANDSFREE,
    EARCUPLE,
    BTHEADSET,
    CARKIT,
    TTY_FULL,   
    TTY_VCO,
    TTY_HCO,
    REC_INC_MIC,
    REC_EXT_MIC,
    PLAYBACK_HEADSET,
    PLAYBACK_HANDSFREE,
    SYS
};

/***********************************************************************************
 *
 *  Interfaces
 *
 ***********************************************************************************/
int htc_acoustic_init(void);
int htc_acoustic_deinit(void);

int msm72xx_enable_audpp(uint16_t enable_mask, uint32_t device);
int msm72xx_set_audpre_params(int audpre_index, int tx_iir_index);
int msm72xx_enable_audpre(int acoustic_flags, int audpre_index, int tx_iir_index);
int snd_get_num_endpoints(void);
int snd_get_endpoint(int, struct msm_snd_endpoint *);

int msm72xx_set_acoustic_table(int device, int volume);
int msm72xx_set_audio_path(bool bEnableMic, bool bEnableDualMic,
                           int device_out, bool bEnableOut);

#if __cplusplus
} // extern "C"
#endif

#endif
