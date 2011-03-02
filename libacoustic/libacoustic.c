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

#define LOG_NDEBUG 0
#define LOG_TAG "Libacoustic-wince"
#include <cutils/log.h>

#include <dlfcn.h>
#include <fcntl.h>
#include <time.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <assert.h>
#include <stdlib.h>
#include <ctype.h>
#include <signal.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <stdbool.h>

#include "libacoustic.h"

//#define AUDIO_PARA_DEFAULT_FILENAME "/system/etc/AudioPara.csv"
#define AUDIO_PARA_DEFAULT_FILENAME   "/sdcard/AudioPara3.csv"
//#define AUDIO_FILTER_DEFAULT_FILENAME "/system/etc/AudioFilter.csv"
#define AUDIO_FILTER_DEFAULT_FILENAME "/sdcard/AudioFilterTable.csv"
//#define AUDIO_PREPROCESS_DEFAULT_FILENAME "/system/etc/AudioPreProcessTable.csv"
#define AUDIO_PREPROCESS_DEFAULT_FILENAME "/sdcard/AudioPreProcessTable.csv"


#define PCM_OUT_DEVICE      "/dev/msm_pcm_out"
#define PCM_IN_DEVICE       "/dev/msm_pcm_in"
#define PCM_CTL_DEVICE      "/dev/msm_pcm_ctl"
#define PREPROC_CTL_DEVICE  "/dev/msm_audpre"

#define MAX_MODE_NAME_LENGTH 32

struct header_s {
    uint8_t Header;
    char Mode[MAX_MODE_NAME_LENGTH];
};

struct register_table_s {
    uint8_t Address;
    uint8_t Reg;
};

struct au_table_st {    
    struct register_table_s register_table[26]; /* Add 1 .. Add 26 */   
    uint8_t Total_number;
    uint8_t Delay_number;
    uint8_t Delay_time;
    uint8_t extra[12];
};

struct au_table_s {
    union {
        struct au_table_st table;
        char   array[0x80];
    };
};

struct be_table_st {
    uint16_t Operator_ID;
    uint16_t Vocpath;
    uint16_t volume_level;
    uint16_t codecTxGain;
    uint16_t codecRxGain;
    uint16_t codecSTGain;
    uint16_t txVolume;
    uint16_t rxVolume;
    uint16_t pcmFormatCtrl;
    uint16_t ecSwitch;
    uint16_t ecMode;
    uint16_t ecStartupMuteHangoverThres;
    uint16_t ecFarendHangoverThres;
    uint16_t esecDoubletalkHangoverThres;
    uint16_t hecDoubletalkHangoverThres;
    uint16_t aecDoubletalkHangoverThres;
    uint16_t ecStartupMuteMode;
    uint16_t ecMuteOverride;
    uint16_t ecStartupErleThres;
    uint16_t ecForceHalfDuplex;
    uint16_t esecResetThres;
    uint16_t hecResetThres;
    uint16_t aecResetThres;
    uint16_t ecInputSampOffset;
    uint16_t rxAgcEnableFlag;
    uint16_t compFlinkStaticGain;
    uint16_t compFlinkAIGFlag;
    uint16_t expFlinkThreshold;
    uint16_t expFlinkSlope;
    uint16_t compFlinkThreshold;
    uint16_t compFlinkSlope;
    uint16_t rxAvcEnableFlag;
    uint16_t avcRlinkSensitivityOffset;
    uint16_t avcFlinkHeadroom;
    uint16_t txAgcEnableFlag;
    uint16_t compRlinkStaticGain;
    uint16_t compRlinkAIGFlag;
    uint16_t expRlinkThreshold;
    uint16_t expRlinkSlope;
    uint16_t compRlinkThreshold;
    uint16_t compRlinkSlope;
    uint16_t nsSwitch;
    uint16_t nsMinGain;
    uint16_t nsSlope;
    uint16_t nsSNRThreshold;
    uint16_t rxPcmFiltCoeff[6];
    uint16_t txPcmFiltCoeff[6];
    uint16_t ec_reset_flag;
    uint16_t Reserved[5];
    uint16_t MCC_Val;
    uint16_t MNC_Val; 
};

struct be_table_s {
    union {
        struct be_table_st table;
        uint16_t array[0x80];
    };
};

struct d_table_st {
    uint16_t routing_mode_config;
    uint16_t internal_codec_config;
    uint16_t external_codec_config;
    uint16_t pcm_ctrl;
    uint16_t codec_intf_ctrl;
    uint16_t dma_path_ctrl;
    uint16_t eight_khz_int_mode;
    uint16_t rx_codec_stereo_config;
    uint16_t tx_codec_stereo_config;
    uint16_t ECNS;
    uint16_t unk0;
};

struct d_table_s {
    union {
        struct d_table_st table;
        uint16_t array[0xB];
    };
};

struct fg_table_st {
    uint16_t volume_level;
    uint16_t codecTxGain;
    uint16_t codecRxGain;
    uint16_t codecSTGain;
    uint16_t txVolume;
    uint16_t rxVolume;
    uint16_t rxAgcEnableFlag;
    uint16_t compFlinkStaticGain;
    uint16_t compFlinkAIGFlag;
    uint16_t expFlinkThreshold;
    uint16_t expFlinkSlope;
    uint16_t compFlinkThreshold;
    uint16_t compFlinkSlope;
    uint16_t comFlinkRmsTav;
    uint16_t compFlinkReleaseK;
    uint16_t compFlinkAIGMin;
    uint16_t compFlinkAIGMax;
    uint16_t rxAvcEnableFlag;
    uint16_t avcRlinkSensitivityOffset;
    uint16_t avcFlinkHeadroom;
    uint16_t txAgcEnableFlag;
    uint16_t compRlinkStaticGain;
    uint16_t compRlinkAIGFlag;
    uint16_t expRlinkThreshold;
    uint16_t expRlinkSlope;
    uint16_t compRlinkThreshold;
    uint16_t compRlinkSlope;
    uint16_t comRlinkRmsTav;
    uint16_t compRlinkReleaseK;
    uint16_t compRlinkAIGMin;
    uint16_t compRlinkAIGMax;
    uint16_t NLPP_limit;
    uint16_t NLPP_gain;
    uint16_t AF_limit;
    uint16_t HS_mode;
    uint16_t Tuning_mode;
    uint16_t echo_path_delay;
    uint16_t OutputGain;
    uint16_t InputGain;
    uint16_t AF_twoalpha;
    uint16_t AF_erl;
    uint16_t AF_taps;
    uint16_t AF_present_coefs;
    uint16_t AF_offset;
    uint16_t AF_erl_bg;
    uint16_t AF_taps_bg;
    uint16_t PCD_threshold;
    uint16_t minimum_erl;
    uint16_t erl_step;
    uint16_t max_noise_floor;
    uint16_t Det_threshold;
    uint16_t SPDET_Far;
    uint16_t SPDET_mic;
    uint16_t SPDET_xclip;
    uint16_t DENS_tail_alpha;
    uint16_t DENS_tail_portion;
    uint16_t DENS_gamma_e_alpha;
    uint16_t DENS_gamma_e_dt;
    uint16_t DENS_gamma_e_low;
    uint16_t DENS_gamma_e_rescue;
    uint16_t DENS_gamma_e_high;
    uint16_t DENS_spdet_near;
    uint16_t DENS_spdet_act;
    uint16_t DENS_gamma_n;
    uint16_t DENS_NFE_blocksize;
    uint16_t DENS_limit_NS;
    uint16_t DENS_NL_atten;
    uint16_t DENS_CNI_Level;
    uint16_t WB_echo_ratio;
    uint16_t rxPcmFiltEnableFlag;
    uint16_t rxPcmFiltCoeff[7];
    uint16_t txPcmFiltEnableFlag;
    uint16_t txPcmFiltCoeff[7];
    uint16_t rxiirFiltNumCoeff[18];
    uint16_t rxiirFiltDenCoeff[12];
    uint16_t rxiirFiltNumShiftFactor[4];
    uint16_t txiirFiltNumCoeff[18];
    uint16_t txiirFiltDenCoeff[12];
    uint16_t txiirFiltNumShiftFactor[4];
    uint16_t ecparameterupdated;
};

struct fg_table_s {
    union {
        struct fg_table_st table;
        uint16_t array[0xA0];
    };
};

struct c_table_s {
    union {
        struct fg_table_st table;
        uint16_t array[0x50];  
    };
};

/***********************************************************************************
 *
 *  Global variables
 *
 ***********************************************************************************/
static struct au_table_s*    Audio_Path_Table = NULL;              /* a_table ('A') */
static struct au_table_s*    Audio_Path_Uplink_Table = NULL;       /* u_table ('U') */
static struct fg_table_s*    Phone_Acoustic_Table = NULL;          /* f_table ('F' + 'B') */
static struct fg_table_s*    BT_Phone_Acoustic_Table = NULL;       /* g_table ('E' + 'G') */
static struct d_table_s*     HTC_VOC_CAL_CODEC_TABLE_Table = NULL; /* d_table ('D') */
static struct c_table_s*     CE_Acoustic_Table = NULL;             /* c_table ('C') */

static int acousticfd = 0;
static int m7xsnddriverfd;
static int mNumSndEndpoints;
static struct msm_snd_endpoint *mSndEndpoints;

static bool mInit = false;

static struct rx_iir_filter iir_cfg[1];
static struct adrc_filter adrc_cfg[1];
static struct eqalizer eqalizer[1];
static uint16_t adrc_flag[1];
static uint16_t eq_flag[1];
static uint16_t rx_iir_flag[1];
static bool audpp_filter_inited = false;
static bool audpre_filter_inited = false;
static bool adrc_filter_exists[1];

static struct tx_iir tx_iir_cfg[18];    // Normal + Full DUplex
static struct ns ns_cfg[9];
static struct tx_agc tx_agc_cfg[9];

static int mSampleRate = AUDIO_HW_IN_SAMPLERATE;

static int SND_DEVICE_CURRENT;
static int SND_DEVICE_HANDSET;
static int SND_DEVICE_SPEAKER;
static int SND_DEVICE_HEADSET;
static int SND_DEVICE_BT;
static int SND_DEVICE_CARKIT;
static int SND_DEVICE_TTY_FULL;
static int SND_DEVICE_TTY_VCO;
static int SND_DEVICE_TTY_HCO;
static int SND_DEVICE_NO_MIC_HEADSET;
static int SND_DEVICE_FM_HEADSET;
static int SND_DEVICE_HEADSET_AND_SPEAKER;
static int SND_DEVICE_FM_SPEAKER;
static int SND_DEVICE_BT_EC_OFF;
static int SND_DEVICE_IDLE;

/* Specific pass-through device for special ops */
static int SND_DEVICE_PLAYBACK_HANDSFREE = 253;
static int SND_DEVICE_PLAYBACK_HEADSET = 254;

static int mCurrentSndDevice = -1;
static int mCurrent_Adie_PGA_Gain = 1;

/***********************************************************************************
 *
 *  Privates functions
 *
 ***********************************************************************************/
static int openacousticfd(void) {
    acousticfd = open(MSM_HTC_ACOUSTIC_WINCE, O_RDWR);
    if ( acousticfd < 0 ) {
        LOGE("Error opening dev %s (fd = %d). Error %s (%d)", MSM_HTC_ACOUSTIC_WINCE, acousticfd,
                    strerror(errno), errno);
        return -1;
    } 
    return 0;
}

static int get_pga_gain_for_fm_profile(int profile, int current_pga_gain)
{
    // FM HEADSET index
    if ( (profile == 18) | (profile == 19) ) {
        if ( current_pga_gain < 0xB ) {
            return ( 0x34 - (current_pga_gain * 4) );
        }
        return 0xFF; 
    }
    // FM SPEAKER index
    else if ( (profile == 22) | (profile == 23) ) {
        if ( current_pga_gain < 0xB ) {
            return ( 0x28 - (current_pga_gain * 4) );
        }
        return 0xFF; 
    }
    return 0;
}

static int UpdateAudioAdieTable(bool bAudioUplinkReq, int paramR1, bool bEnableHSSD, bool bAUXBypassReq)
{
    struct adie_table table;
    int table_num = 0;
    int tab_byte_idx;

    char temp_table[0x80];


    LOGV("UpdateAudioAdieTable(bAudioUplinkReq %d,bAUXBypassReq %d, bEnableHSSD=%d)\n",
            bAudioUplinkReq, bEnableHSSD, bAUXBypassReq);

    do 
    {
        memset(temp_table, 0, 0x80);
        tab_byte_idx = 0;
        
        do
        {
            temp_table[tab_byte_idx] = Audio_Path_Table[table_num].array[tab_byte_idx];
            temp_table[tab_byte_idx + 1] = (temp_table[tab_byte_idx + 1] | Audio_Path_Table[table_num].array[tab_byte_idx + 1]);

            if ( (!(table_num & 1)) && ( bAudioUplinkReq != 0 ) ) {
                temp_table[tab_byte_idx + 1] = (Audio_Path_Uplink_Table[table_num].array[tab_byte_idx + 1] 
                                                     | Audio_Path_Table[table_num].array[tab_byte_idx + 1]);
            }

            if ( paramR1 == 0 ) {
                if ( bEnableHSSD != 0 ) {
                    if ( Audio_Path_Table[table_num].array[tab_byte_idx] == 0x37 ) {
                        temp_table[tab_byte_idx + 1] |= 0x80;
                    }
                    if ( Audio_Path_Table[table_num].array[tab_byte_idx] == 0x48 ) {
                        temp_table[tab_byte_idx + 1] |= 0xC0;
                    }
                }
            } else {
                if ( Audio_Path_Table[table_num].array[tab_byte_idx] != 0x3E ) {
                    temp_table[tab_byte_idx + 1] = (temp_table[tab_byte_idx + 1] & 0xE7) | 0x10;           
                }
            }

            if ( (bAUXBypassReq != 0) && (Audio_Path_Table[table_num].array[tab_byte_idx] == 0x42) ) {
                temp_table[tab_byte_idx + 1] = get_pga_gain_for_fm_profile(table_num, mCurrent_Adie_PGA_Gain);
            }

            tab_byte_idx += 2;
        }
        while( tab_byte_idx < 0x80);
        
        /* Send table to kernel for update */
        table.table_num = table_num;
        table.pcArray = temp_table;
        if ( ioctl(acousticfd, ACOUSTIC_UPDATE_ADIE_TABLE, &table) < 0) {
            LOGE("ACOUSTIC_UPDATE_ADIE_TABLE error.");
            return -EIO;
        }    

        table_num += 1;
    }
    while ( table_num < 32 );

    return 0;
}


static int ParseAudioParaLine(char* line, int len)
{
    char *token, *ps;
    int table_num;
    int field_count = 0;

    /* Parse the first field */
    token = strtok(line, ",");
    switch ( token[0] ) {
        case 'A':
            table_num = strtol(token + 1, &ps, 10);
            //LOGV("Audio Path Table: %d\n", table_num);
            /* Skip the mode name string field */
            strtok(NULL, ",");
            while ( (token = strtok(NULL, ",")) ) {
                Audio_Path_Table[table_num].array[field_count++] = strtol(token, &ps, 16);
            };
            //LOGV("field count : %d (0x%x)\n", field_count, field_count);
            break;

        case 'B':
        case 'F':
            table_num = strtol(token + 1, &ps, 10);
            //LOGV("Phone Acoustic Table: %d\n", table_num);
            /* Skip the mode name string field */
            strtok(NULL, ",");
            while ( (token = strtok(NULL, ",")) ) {
                Phone_Acoustic_Table[table_num].array[field_count++] = strtol(token, &ps, 16);
            };
            //LOGV("field count : %d, size %d (0x%x)\n", field_count, field_count *2, field_count *2);
            break;

        case 'C':
            table_num = strtol(token + 1, &ps, 10);
            //LOGV("CE Acoustic Table: %d\n", table_num);
            /* Skip the mode name string field */
            strtok(NULL, ",");
            while ( (token = strtok(NULL, ",")) ) {
                CE_Acoustic_Table[table_num].array[field_count++] = strtol(token, &ps, 16);
            };
            //LOGV("field count : %d, size %d (0x%x)\n", field_count, field_count *2, field_count *2);
            break;

        case 'D':
            table_num = strtol(token + 1, &ps, 10);
            //LOGV("HTC_VOC_CAL_CODEC_TABLE Table: %d\n", table_num);
            /* Skip the mode name string field */
            strtok(NULL, ",");
            while ( (token = strtok(NULL, ",")) ) {
                HTC_VOC_CAL_CODEC_TABLE_Table[table_num].array[field_count++] = strtol(token, &ps, 16);
            };
            //LOGV("field count : %d, size %d (0x%x)\n", field_count, field_count *2, field_count *2);
            break;

        case 'E':
        case 'G':
            table_num = strtol(token + 1, &ps, 10);
            //LOGV("BT Phone Acoustic Table: %d\n", table_num);
            /* Skip the mode name string field */
            strtok(NULL, ",");
            while ( (token = strtok(NULL, ",")) ) {
                BT_Phone_Acoustic_Table[table_num].array[field_count++] = strtol(token, &ps, 16);
            };
            //LOGV("field count : %d, size %d (0x%x)\n", field_count, field_count *2, field_count *2);
            break;

        case 'H':
            //LOGV("It's just a header line, skip it\n");
            return 0;

        case 'U':
            table_num = strtol(token + 1, &ps, 10);
            //LOGV("Audio Path Table Uplink: %d\n", table_num);
            /* Skip the mode name string field */
            strtok(NULL, ",");
            while ( (token = strtok(NULL, ",")) ) {
                Audio_Path_Uplink_Table[table_num].array[field_count++] = strtol(token, &ps, 16);
            };
            //LOGV("field count : %d, size %d (0x%x)\n", field_count, field_count, field_count);
            break;

        default:
            LOGE("Unknown parameter field %c\n", token[0]);
            return -1;
    }
    
    return 0;

}

static int ReadAudioParaFromFile(void)
{
    struct stat st;
    char *read_buf;
    char *next_str, *current_str;
    int csvfd;

    static const char *const path =
        AUDIO_PARA_DEFAULT_FILENAME;

    csvfd = open(path, O_RDONLY);
    if (csvfd < 0) {
        /* Failed to open parameters file ... */
        LOGE("Failed to open %s. Error %s (%d)\n",
             path, strerror(errno), errno);
        return -1;
    } else {
        LOGE("Successfully opened %s\n", path);
    }

    if (fstat(csvfd, &st) < 0) {
        LOGE("Failed to stat %s: %s (%d)\n",
             path, strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    read_buf = (char *) mmap(0, st.st_size,
                    PROT_READ | PROT_WRITE,
                    MAP_PRIVATE,
                    csvfd, 0);

    if (read_buf == MAP_FAILED) {
        LOGE("Failed to mmap parameters file: %s (%d)\n",
             strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    if ( Audio_Path_Table == NULL ) {
        Audio_Path_Table = (struct au_table_s*) malloc(32 * sizeof(struct au_table_s) );  // 0x1000
        if (Audio_Path_Table == NULL) {
            LOGE("Failed to malloc Audio_Path_Table\n");
            return -1;
        }
    }

    if ( Audio_Path_Uplink_Table == NULL ) {
        Audio_Path_Uplink_Table = (struct au_table_s*) malloc(32 * sizeof(struct au_table_s) ); // 0x1000
        if (Audio_Path_Uplink_Table == NULL) {
            LOGE("Failed to malloc Audio_Path_Uplink_Table\n");
            return -1;
        }
    }

    if ( Phone_Acoustic_Table == NULL ) {
        Phone_Acoustic_Table = (struct fg_table_s*) malloc(100 * sizeof(struct fg_table_s) ); // 0x3C00
        if (Phone_Acoustic_Table == NULL) {
            LOGE("Failed to malloc Phone_Acoustic_Table\n");
            return -1;
        }
    }

    if ( BT_Phone_Acoustic_Table == NULL ) {
        BT_Phone_Acoustic_Table = (struct fg_table_s*) malloc(100 * sizeof(struct fg_table_s) );  // 0x7800
        if (BT_Phone_Acoustic_Table == NULL) {
            LOGE("Failed to malloc BT_Phone_Acoustic_Table\n");
            return -1;
        }
    }

    if ( HTC_VOC_CAL_CODEC_TABLE_Table == NULL ) {
        HTC_VOC_CAL_CODEC_TABLE_Table = (struct d_table_s*) malloc(32 * sizeof(struct d_table_s));
        if (HTC_VOC_CAL_CODEC_TABLE_Table == NULL) {
            LOGE("Failed to malloc HTC_VOC_CAL_CODEC_TABLE_Table\n");
            return -1;
        }
    }

    if ( CE_Acoustic_Table == NULL ) {
        CE_Acoustic_Table = (struct c_table_s*) malloc(15 * sizeof(struct c_table_s) );   // 0x960
        if (CE_Acoustic_Table == NULL) {
            LOGE("Failed to malloc CE_Acoustic_Table\n");
            return -1;
        }
    }

    current_str = read_buf;

    while (1) {
        int len;
        next_str = strchr(current_str, '\n');
        if (!next_str)
           break;
        len = next_str - current_str;
        *next_str++ = '\0';
        if ( ParseAudioParaLine(current_str, len) < 0 ) {
            break;
        }

        current_str = next_str;
    }

    munmap(read_buf, st.st_size);
    close(csvfd);

    // initialise audio table with uplink off
    UpdateAudioAdieTable(0, 0, 0, 0);

    if (ioctl(acousticfd, ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE,
                         &(HTC_VOC_CAL_CODEC_TABLE_Table->array) ) < 0) {
        LOGE("ACOUSTIC_UPDATE_HTC_VOC_CAL_CODEC_TABLE error.");
        return -EIO;
    }

/*
    if ( BT_Phone_Acoustic_Table[0] == 0 ) {
        memcpy(&BT_Phone_Acoustic_Table[0x40], &f_table[0x1680], 0x140);
    }
*/

    return 0;
}



/* Imports from codeaurora (check_and_set_audpp_parameters), adaptated/splitted for wince devices */
static int check_and_set_audpre_parameters(char *buf, int size)
{
    char *p, *ps;
    static const char *const seps = ",";
    int table_num;
    int i, j;
    int device_id = 0;
    int samp_index = 0;
    int fd;

    if (buf[0] == 'A')  {
        /* TX_IIR filter */
        if (!(p = strtok(buf, ","))){
            goto token_err;}

        /* Table header */
        samp_index = strtol(p + 1, &ps, 10);
        LOGV("Found TX_IIR filter %d", samp_index); 
        /* Index range = 0..17 */
        if ( table_num > 17 ) {
            return -EINVAL;
        }

        if (!(p = strtok(NULL, seps))){
            goto token_err;}
        /* Table description */
        if (!(p = strtok(NULL, seps))){
            goto token_err;}

        for (i = 0; i < 48; i++) {
            j = (i >= 40)? i : ((i % 2)? (i - 1) : (i + 1));
            tx_iir_cfg[samp_index].iir_params[j] = (uint16_t)strtol(p, &ps, 16);
            if (!(p = strtok(NULL, seps))){
                goto token_err;}
        }

        tx_iir_cfg[samp_index].active_flag = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps))){
            goto token_err;}

        tx_iir_cfg[samp_index].num_bands = (uint16_t)strtol(p, &ps, 16);
/*
        tx_iir_cfg[samp_index].cmd_id = 0;
*/
    } else if(buf[0] == 'B')  {
        /* AGC filter */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        samp_index = strtol(p + 1, &ps, 10);
        LOGV("Found AGC filter %d", samp_index); 
        /* Index range = 0..8 */
        if ( table_num > 8 ) {
            return -EINVAL;
        }

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        tx_agc_cfg[samp_index].tx_agc_enable_flag = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        tx_agc_cfg[samp_index].static_gain = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        tx_agc_cfg[samp_index].adaptive_gain_flag = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        for (i = 0; i < 17; i++) {
            tx_agc_cfg[samp_index].agc_params[i] = (uint16_t)strtol(p, &ps, 16);
            if (!(p = strtok(NULL, seps)))
                goto token_err;
            }
/*
        for (i = 0; i < 19; i++) {
            tx_agc_cfg[samp_index].agc_params[i] = (uint16_t)strtol(p, &ps, 16);
            if (!(p = strtok(NULL, seps)))
                goto token_err;
            }

        tx_agc_cfg[samp_index].cmd_id = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        tx_agc_cfg[samp_index].tx_agc_param_mask = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
*/
    } else if ((buf[0] == 'C')) {       
        /* This is the NS record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        samp_index = strtol(p + 1, &ps, 10);
        LOGV("Found NS record %d", samp_index); 
        /* Index range = 0..8 */
        if ( table_num > 8 ) {
            return -EINVAL;
        }

        if (!(p = strtok(NULL, seps)))
            goto token_err;

        /* Table description */
/*
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        ns_cfg[samp_index].cmd_id = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        ns_cfg[samp_index].ec_mode_new = (uint16_t)strtol(p, &ps, 16);
*/
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        ns_cfg[samp_index].dens_gamma_n = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        ns_cfg[samp_index].dens_nfe_block_size = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        ns_cfg[samp_index].dens_limit_ns = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        ns_cfg[samp_index].dens_limit_ns_d = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        ns_cfg[samp_index].wb_gamma_e = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        ns_cfg[samp_index].wb_gamma_n = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
    }
//    }
    return 0;

token_err:
    LOGE("malformatted pcm control buffer");
    return -EINVAL;
}

static int get_audpre_table(void)
{
    struct stat st;
    char *read_buf;
    char *next_str, *current_str;
    int csvfd;

    LOGI("get_audpre_table");
    static const char *const path =
            AUDIO_PREPROCESS_DEFAULT_FILENAME;
    csvfd = open(path, O_RDONLY);
    if (csvfd < 0) {
        /* failed to open normal acoustic file ... */
        LOGE("failed to open AUDIO_NORMAL_PREPROCESS %s: %s (%d).",
             path, strerror(errno), errno);
        return -1;
    } else LOGI("open %s success.", path);

    if (fstat(csvfd, &st) < 0) {
        LOGE("failed to stat %s: %s (%d).",
             path, strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    read_buf = (char *) mmap(0, st.st_size,
                    PROT_READ | PROT_WRITE,
                    MAP_PRIVATE,
                    csvfd, 0);

    if (read_buf == MAP_FAILED) {
        LOGE("failed to mmap parameters file: %s (%d)",
             strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    current_str = read_buf;

    while (1) {
        int len;
        next_str = strchr(current_str, '\n');
        if (!next_str)
           break;
        len = next_str - current_str;
        *next_str++ = '\0';
        if (check_and_set_audpre_parameters(current_str, len)) {
            LOGI("failed to set audpre parameters, exiting.");
            munmap(read_buf, st.st_size);
            close(csvfd);
            return -1;
        }
        current_str = next_str;
    }

    munmap(read_buf, st.st_size);
    close(csvfd);
    return 0;
}

static int check_and_set_audpp_parameters(char *buf, int size)
{
    char *p, *ps;
    static const char *const seps = ",";
    int table_num;
    int i, j;
    int samp_index = 0;
    struct eq_filter_type eq[12];
    int fd;
    void *audioeq;
    void *(*eq_cal)(int32_t, int32_t, int32_t, uint16_t, int32_t, int32_t *, int32_t *, uint16_t *);
    uint16_t numerator[6];
    uint16_t denominator[4];
    uint16_t shift[2];

    if ( (buf[0] == 'A') && (buf[1] == '1') ) {
        LOGV("Found IIR filter");
        /* IIR filter */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        for (i = 0; i < 48; i++) {
            j = (i >= 40)? i : ((i % 2)? (i - 1) : (i + 1));
            iir_cfg[0].iir_params[j] = (uint16_t)strtol(p, &ps, 16);
            if (!(p = strtok(NULL, seps)))
                goto token_err;
        }
        rx_iir_flag[0] = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        iir_cfg[0].num_bands = (uint16_t)strtol(p, &ps, 16);

    } else if ( (buf[0] == 'B') && (buf[1] == '1') ) {
        LOGV("Found ADRC record");
        /* This is the ADRC record we are looking for.  Tokenize it */
        adrc_filter_exists[0] = true;
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_flag[0] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_cfg[0].adrc_params[0] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_cfg[0].adrc_params[1] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_cfg[0].adrc_params[2] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_cfg[0].adrc_params[3] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_cfg[0].adrc_params[4] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_cfg[0].adrc_params[5] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_cfg[0].adrc_params[6] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;
        adrc_cfg[0].adrc_params[7] = (uint16_t)strtol(p, &ps, 16);

        if (!(p = strtok(NULL, seps)))
            goto token_err;

    } else if ( (buf[0] == 'C') && (buf[1] == '1') ) {
        LOGV("Found EQ record");
        /* This is the EQ record we are looking for.  Tokenize it */
        if (!(p = strtok(buf, ",")))
            goto token_err;

        /* Table header */
        table_num = strtol(p + 1, &ps, 10);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        /* Table description */
        if (!(p = strtok(NULL, seps)))
            goto token_err;

        eq_flag[0] = (uint16_t)strtol(p, &ps, 16);
        if (!(p = strtok(NULL, seps)))
            goto token_err;
        LOGI("EQ flag = %02x.", eq_flag[0]);

        audioeq = dlopen("/system/lib/libaudioeq.so", RTLD_NOW);
        if (audioeq == NULL) {
            LOGE("audioeq library open failure");
            return -1;
        }
        eq_cal = (void *(*) (int32_t, int32_t, int32_t, uint16_t, int32_t, int32_t *, int32_t *, uint16_t *)) dlsym(audioeq, "audioeq_calccoefs");
        memset(&eqalizer[0], 0, sizeof(eqalizer));

        /* Temp add the bands here */
        eqalizer[0].bands = 8;
        for (i = 0; i < eqalizer[0].bands; i++) {

            eq[i].gain = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].freq = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].type = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;
            eq[i].qf = (uint16_t)strtol(p, &ps, 16);

            if (!(p = strtok(NULL, seps)))
                goto token_err;

            eq_cal(eq[i].gain, eq[i].freq, 48000, eq[i].type, eq[i].qf, (int32_t*)numerator, (int32_t *)denominator, shift);
            for (j = 0; j < 6; j++) {
                eqalizer[0].params[ ( i * 6) + j] = numerator[j];
            }
            for (j = 0; j < 4; j++) {
                eqalizer[0].params[(eqalizer[0].bands * 6) + (i * 4) + j] = denominator[j];
            }
            eqalizer[0].params[(eqalizer[0].bands * 10) + i] = shift[0];
        }
        dlclose(audioeq);

    }
    return 0;

token_err:
    LOGE("malformatted pcm control buffer");
    return -EINVAL;
}

static int get_audpp_filter(void)
{
    struct stat st;
    char *read_buf;
    char *next_str, *current_str;
    int csvfd;

    LOGI("get_audpp_filter");
    static const char *const path =
            AUDIO_FILTER_DEFAULT_FILENAME;
    csvfd = open(path, O_RDONLY);
    if (csvfd < 0) {
        /* failed to open normal acoustic file ... */
        LOGE("failed to open AUDIO_NORMAL_FILTER %s: %s (%d).",
             path, strerror(errno), errno);
        return -1;
    } else LOGI("open %s success.", path);

    if (fstat(csvfd, &st) < 0) {
        LOGE("failed to stat %s: %s (%d).",
             path, strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    read_buf = (char *) mmap(0, st.st_size,
                    PROT_READ | PROT_WRITE,
                    MAP_PRIVATE,
                    csvfd, 0);

    if (read_buf == MAP_FAILED) {
        LOGE("failed to mmap parameters file: %s (%d)",
             strerror(errno), errno);
        close(csvfd);
        return -1;
    }

    current_str = read_buf;

    while (1) {
        int len;
        next_str = strchr(current_str, '\n');
        if (!next_str)
           break;
        len = next_str - current_str;
        *next_str++ = '\0';
        if (check_and_set_audpp_parameters(current_str, len)) {
            LOGI("failed to set audpp parameters, exiting.");
            munmap(read_buf, st.st_size);
            close(csvfd);
            return -1;
        }
        current_str = next_str;
    }

    munmap(read_buf, st.st_size);
    close(csvfd);
    return 0;
}

static unsigned calculate_audpre_table_index(unsigned index)
{
    switch (index) {
        case 48000:    return SAMP_RATE_INDX_48000;
        case 44100:    return SAMP_RATE_INDX_44100;
        case 32000:    return SAMP_RATE_INDX_32000;
        case 24000:    return SAMP_RATE_INDX_24000;
        case 22050:    return SAMP_RATE_INDX_22050;
        case 16000:    return SAMP_RATE_INDX_16000;
        case 12000:    return SAMP_RATE_INDX_12000;
        case 11025:    return SAMP_RATE_INDX_11025;
        case 8000:    return SAMP_RATE_INDX_8000;
        default:     return -1;
    }
}


/***********************************************************************************
 *
 *  Interfaces
 *
 ***********************************************************************************/
int htc_acoustic_init(void)
{
    int rc = 0;
    int cnt;
    struct msm_snd_endpoint *ept;

    LOGV("Running custom built libhtc_acoustic.so");

    /* Open the acoustic driver */
    rc = openacousticfd();
    if ( rc < 0 ) {
        return rc;
    }

    /* Read parameters from csv file */
    rc = ReadAudioParaFromFile();
    if ( rc < 0 ) {
        return rc;
    }

    /* Read filter tables */
    rc = get_audpp_filter();
    if ( rc == 0 ) {
        audpp_filter_inited = true;
    }
    rc = get_audpre_table();
    if ( rc == 0 ) {
        audpre_filter_inited = true;
    }

    m7xsnddriverfd = open("/dev/msm_snd", O_RDWR);
    if (m7xsnddriverfd >= 0) {
        int rc = ioctl(m7xsnddriverfd, SND_GET_NUM_ENDPOINTS, &mNumSndEndpoints);
        if (rc >= 0) {
            mSndEndpoints = malloc(mNumSndEndpoints * sizeof(struct msm_snd_endpoint));
            mInit = true;
            LOGV("constructed (%d SND endpoints)", mNumSndEndpoints);
            struct msm_snd_endpoint *ept = mSndEndpoints;
            for (cnt = 0; cnt < mNumSndEndpoints; cnt++, ept++) {
                ept->id = cnt;
                ioctl(m7xsnddriverfd, SND_GET_ENDPOINT, ept);
                LOGV("cnt = %d ept->name = %s ept->id = %d\n", cnt, ept->name, ept->id);
#define CHECK_FOR(desc) if (!strcmp(ept->name, #desc)) SND_DEVICE_##desc = ept->id;
                CHECK_FOR(CURRENT)
                CHECK_FOR(HANDSET)
                CHECK_FOR(SPEAKER)
                CHECK_FOR(BT)
                CHECK_FOR(BT_EC_OFF)
                CHECK_FOR(HEADSET)
                CHECK_FOR(CARKIT)
                CHECK_FOR(TTY_FULL)
                CHECK_FOR(TTY_VCO)
                CHECK_FOR(TTY_HCO)
                CHECK_FOR(NO_MIC_HEADSET)
                CHECK_FOR(FM_HEADSET)
                CHECK_FOR(FM_SPEAKER)
                CHECK_FOR(HEADSET_AND_SPEAKER)
                CHECK_FOR(IDLE)
#undef CHECK_FOR
            }
        }
        else LOGE("Could not retrieve number of MSM SND endpoints.");

/* TODO : Check and enable ??
        int AUTO_VOLUME_ENABLED = 1; // setting enabled as default

        static const char *const path = "/system/etc/AutoVolumeControl.txt";
        int txtfd;
        struct stat st;
        char *read_buf;

        txtfd = open(path, O_RDONLY);
        if (txtfd < 0) {
            LOGE("failed to open AUTO_VOLUME_CONTROL %s: %s (%d)",
                  path, strerror(errno), errno);
        }
        else {
            if (fstat(txtfd, &st) < 0) {
                LOGE("failed to stat %s: %s (%d)",
                      path, strerror(errno), errno);
                close(txtfd);
            }

            read_buf = (char *) mmap(0, st.st_size,
                        PROT_READ | PROT_WRITE,
                        MAP_PRIVATE,
                        txtfd, 0);

            if (read_buf == MAP_FAILED) {
                LOGE("failed to mmap parameters file: %s (%d)",
                      strerror(errno), errno);
                close(txtfd);
            }

            if(read_buf[0] =='0')
               AUTO_VOLUME_ENABLED = 0;

            munmap(read_buf, st.st_size);
            close(txtfd);
        }

        ioctl(m7xsnddriverfd, SND_AVC_CTL, &AUTO_VOLUME_ENABLED);
        ioctl(m7xsnddriverfd, SND_AGC_CTL, &AUTO_VOLUME_ENABLED);
*/
    }
	else LOGE("Could not open MSM SND driver.");

    return rc;
}

int snd_get_num_endpoints(void)
{
    return mNumSndEndpoints;
}

int snd_get_endpoint(int cnt, struct msm_snd_endpoint *p_ept)
{
    int icnt;
    struct msm_snd_endpoint *ept = mSndEndpoints;

    *p_ept = ept[cnt];

    return 0;
}

int htc_acoustic_deinit(void)
{
    int rc = 0;

    /* Close the acoustic driver */
    close(acousticfd);

    /* Free the memory */
    if ( Audio_Path_Table != NULL )
        free(Audio_Path_Table);
    if ( Audio_Path_Uplink_Table != NULL )
        free(Audio_Path_Uplink_Table);
    if ( Phone_Acoustic_Table != NULL )
        free(Phone_Acoustic_Table);
    if ( BT_Phone_Acoustic_Table != NULL )
        free(BT_Phone_Acoustic_Table);
    if ( HTC_VOC_CAL_CODEC_TABLE_Table != NULL )
        free(HTC_VOC_CAL_CODEC_TABLE_Table);
    if ( mSndEndpoints != NULL )
        free(mSndEndpoints);

    return rc;
}

// TODO : See how EQ and filters are derived from the initial settings
int msm72xx_enable_audpp(uint16_t enable_mask, uint32_t device)
{
    int fd;
    int device_id=0;

    if (!audpp_filter_inited) return -EINVAL;

    LOGI("SET DEVICE - %d",device);
    if(device == (uint32_t)SND_DEVICE_SPEAKER)
    {
        device_id = 0;
        LOGI("SET DEVICE TO SND_DEVICE_SPEAKER device_id=0 .");
    }
    if(device == (uint32_t)SND_DEVICE_HANDSET)
    {
//        device_id = 1;
        device_id = 0;
        LOGI("SET DEVICE - SND_DEVICE_HANDSET device_id=1 .");
    }
    if(device == (uint32_t)SND_DEVICE_HEADSET)
    {
//        device_id = 2;
        device_id = 0;
        LOGI("SET DEVICE - SND_DEVICE_HEADSET device_id=2 .");
    }

    fd = open(PCM_CTL_DEVICE, O_RDWR);
    if (fd < 0) {
        LOGE("Cannot open PCM Ctl device");
        return -EPERM;
    }

    if (adrc_filter_exists[device_id])
    {
        if (adrc_flag[device_id] == 0 && (enable_mask & ADRC_ENABLE))
            enable_mask &= ~ADRC_ENABLE;
        else if(enable_mask & ADRC_ENABLE)
        {
            LOGI("ADRC Filter ADRC FLAG = %02x.", adrc_flag[device_id]);
            LOGI("ADRC Filter COMP THRESHOLD = %02x.", adrc_cfg[device_id].adrc_params[0]);
            LOGI("ADRC Filter COMP SLOPE = %02x.", adrc_cfg[device_id].adrc_params[1]);
            LOGI("ADRC Filter COMP RMS TIME = %02x.", adrc_cfg[device_id].adrc_params[2]);
            LOGI("ADRC Filter COMP ATTACK[0] = %02x.", adrc_cfg[device_id].adrc_params[3]);
            LOGI("ADRC Filter COMP ATTACK[1] = %02x.", adrc_cfg[device_id].adrc_params[4]);
            LOGI("ADRC Filter COMP RELEASE[0] = %02x.", adrc_cfg[device_id].adrc_params[5]);
            LOGI("ADRC Filter COMP RELEASE[1] = %02x.", adrc_cfg[device_id].adrc_params[6]);
            LOGI("ADRC Filter COMP DELAY = %02x.", adrc_cfg[device_id].adrc_params[7]);
            if (ioctl(fd, AUDIO_SET_ADRC, &adrc_cfg[device_id]) < 0)
            {
                LOGE("set adrc filter error.");
            }
        }
    }

    if (eq_flag[device_id] == 0 && (enable_mask & EQ_ENABLE))
        enable_mask &= ~EQ_ENABLE;
    else if (enable_mask & EQ_ENABLE)
    {
	    LOGI("Setting EQ Filter");
        if (ioctl(fd, AUDIO_SET_EQ, &eqalizer[device_id]) < 0) {
            LOGE("set Equalizer error.");
        }
    }

    if (rx_iir_flag[device_id] == 0 && (enable_mask & RX_IIR_ENABLE))
        enable_mask &= ~RX_IIR_ENABLE;
    else if (enable_mask & RX_IIR_ENABLE)
    {
        LOGI("IIR Filter FLAG = %02x.", rx_iir_flag[device_id]);
        LOGI("IIR NUMBER OF BANDS = %02x.", iir_cfg[device_id].num_bands);
        LOGI("IIR Filter N1 = %02x.", iir_cfg[device_id].iir_params[0]);
        LOGI("IIR Filter N2 = %02x.",  iir_cfg[device_id].iir_params[1]);
        LOGI("IIR Filter N3 = %02x.",  iir_cfg[device_id].iir_params[2]);
        LOGI("IIR Filter N4 = %02x.",  iir_cfg[device_id].iir_params[3]);
        LOGI("IIR FILTER M1 = %02x.",  iir_cfg[device_id].iir_params[24]);
        LOGI("IIR FILTER M2 = %02x.", iir_cfg[device_id].iir_params[25]);
        LOGI("IIR FILTER M3 = %02x.",  iir_cfg[device_id].iir_params[26]);
        LOGI("IIR FILTER M4 = %02x.",  iir_cfg[device_id].iir_params[27]);
        LOGI("IIR FILTER M16 = %02x.",  iir_cfg[device_id].iir_params[39]);
        LOGI("IIR FILTER SF1 = %02x.",  iir_cfg[device_id].iir_params[40]);
        if (ioctl(fd, AUDIO_SET_RX_IIR, &iir_cfg[device_id]) < 0)
        {
            LOGE("set rx iir filter error.");
        }
    }

    LOGE("msm72xx_enable_audpp: 0x%04x", enable_mask);
    if (ioctl(fd, AUDIO_ENABLE_AUDPP, &enable_mask) < 0) {
        LOGE("enable audpp error");
        close(fd);
        return -EPERM;
    }

    close(fd);
    return 0;
}

// TODO : Check with lbhtc-acoustic.so
int msm72xx_set_audpre_params(int audpre_index, int tx_iir_index)
{
    if (audpre_filter_inited)
    {
        audpre_index = calculate_audpre_table_index(mSampleRate);
        int fd;

        fd = open(PREPROC_CTL_DEVICE, O_RDWR);
        if (fd < 0) {
             LOGE("Cannot open PreProc Ctl device");
             return -EPERM;
        }

         /* Setting AGC Params */
/*
        LOGI("AGC Filter Param1= %02x.", tx_agc_cfg[audpre_index].cmd_id);
        LOGI("AGC Filter Param2= %02x.", tx_agc_cfg[audpre_index].tx_agc_param_mask);
*/
        LOGI("AGC Filter Param3= %02x.", tx_agc_cfg[audpre_index].tx_agc_enable_flag);
        LOGI("AGC Filter Param4= %02x.", tx_agc_cfg[audpre_index].static_gain);
        LOGI("AGC Filter Param5= %02x.", tx_agc_cfg[audpre_index].adaptive_gain_flag);
        LOGI("AGC Filter Param6= %02x.", tx_agc_cfg[audpre_index].agc_params[0]);
        LOGI("AGC Filter Param7= %02x.", tx_agc_cfg[audpre_index].agc_params[16]);
        if (ioctl(fd, AUDIO_SET_AGC, &tx_agc_cfg[audpre_index]) < 0)
        {
            LOGE("set AGC filter error.");
        }

         /* Setting NS Params */
/*
        LOGI("NS Filter Param1= %02x.", ns_cfg[audpre_index].cmd_id);
        LOGI("NS Filter Param2= %02x.", ns_cfg[audpre_index].ec_mode_new);
*/
        LOGI("NS Filter Param3= %02x.", ns_cfg[audpre_index].dens_gamma_n);
        LOGI("NS Filter Param4= %02x.", ns_cfg[audpre_index].dens_nfe_block_size);
        LOGI("NS Filter Param5= %02x.", ns_cfg[audpre_index].dens_limit_ns);
        LOGI("NS Filter Param6= %02x.", ns_cfg[audpre_index].dens_limit_ns_d);
        LOGI("NS Filter Param7= %02x.", ns_cfg[audpre_index].wb_gamma_e);
        LOGI("NS Filter Param8= %02x.", ns_cfg[audpre_index].wb_gamma_n);
        if (ioctl(fd, AUDIO_SET_NS, &ns_cfg[audpre_index]) < 0)
        {
            LOGE("set NS filter error.");
        }

        /* Setting TX_IIR Params */
/*
        LOGI("TX_IIR Filter Param1= %02x.", tx_iir_cfg[audpre_index].cmd_id);
*/
        LOGI("TX_IIR Filter Param2= %02x.", tx_iir_cfg[audpre_index].active_flag);
        LOGI("TX_IIR Filter Param3= %02x.", tx_iir_cfg[audpre_index].num_bands);
        LOGI("TX_IIR Filter Param4= %02x.", tx_iir_cfg[audpre_index].iir_params[0]);
        LOGI("TX_IIR Filter Param5= %02x.", tx_iir_cfg[audpre_index].iir_params[1]);
        LOGI("TX_IIR Filter Param6 %02x.", tx_iir_cfg[audpre_index].iir_params[47]);
        if (ioctl(fd, AUDIO_SET_TX_IIR, &tx_iir_cfg[audpre_index]) < 0)
        {
           LOGE("set TX IIR filter error.");
        }

	    close(fd);
        return 0;
    }

    return -1;
}

// TODO : Check with lbhtc-acoustic.so
int msm72xx_enable_audpre(int acoustic_flags, int audpre_index, int tx_iir_index)
{
    int fd;

    fd = open(PREPROC_CTL_DEVICE, O_RDWR);
    if (fd < 0) {
         LOGE("Cannot open PreProc Ctl device");
         return -EPERM;
    }
     /*Setting AUDPRE_ENABLE*/
    if (ioctl(fd, AUDIO_ENABLE_AUDPRE, &acoustic_flags) < 0)
    {
       LOGE("set AUDPRE_ENABLE error.");
    }
	close(fd);

    return 0;
}


int msm72xx_set_acoustic_table(int device, int volume)
{
    struct fg_table_s* table = NULL;
    struct c_table_s*  ce_table = NULL;
    int out_path = device;

    LOGV("msm72xx_set_acoustic_table %d %d", device, volume);

    if ( volume > 5 ) {
        return -EIO;
    }

    if ( device == SND_DEVICE_CURRENT ) {
       out_path = mCurrentSndDevice;
    } else {
        if( device == SND_DEVICE_HANDSET ) {
            LOGV("Acoustic profile : EARCUPLE");
            out_path = EARCUPLE;
        } else if( device == SND_DEVICE_SPEAKER ) {
            LOGV("Acoustic profile : HANDSFREE");
            out_path = HANDSFREE;
        } else if( device == SND_DEVICE_HEADSET ) {
            LOGV("Acoustic profile : HEADSET");
            out_path = HEADSET;
        } else if( (device == SND_DEVICE_BT) || (device == SND_DEVICE_BT_EC_OFF) ) {
            LOGV("Acoustic profile : BTHEADSET");
            out_path = BTHEADSET;
        } else if( device == SND_DEVICE_CARKIT ) {
            LOGV("Acoustic profile : CARKIT");
            out_path = CARKIT;
        } else if( device == SND_DEVICE_TTY_FULL ) {
            LOGV("Acoustic profile : TTY_FULL");
            out_path = TTY_FULL;
        } else if( device == SND_DEVICE_TTY_VCO ) {
            LOGV("Acoustic profile : TTY_VCO");
            out_path = TTY_VCO;
        } else if( device == SND_DEVICE_TTY_HCO ) {
            LOGV("Acoustic profile : TTY_HCO");
            out_path = TTY_HCO;
        } else if( device == SND_DEVICE_PLAYBACK_HEADSET ) {
            LOGV("Acoustic profile : PLAYBACK_HEADSET");
            out_path = PLAYBACK_HEADSET;
        } else if( device == SND_DEVICE_PLAYBACK_HANDSFREE ) {
            LOGV("Acoustic profile : PLAYBACK_HANDSFREE");
            out_path = PLAYBACK_HANDSFREE;
        } else if( device == SND_DEVICE_IDLE ) {
            LOGV("Acoustic profile : EARCUPLE");
            out_path = EARCUPLE;
        }

    }

    // TODO : See UpdateVolumeTable from CE for device = 3
    switch ( out_path )
    {
        case HEADSET:
        case HANDSFREE:
        case EARCUPLE:
            table = &Phone_Acoustic_Table[(out_path*6) + volume];
        break;

        case BTHEADSET:
            table = &Phone_Acoustic_Table[18];
        break;

        case CARKIT:
            table = &Phone_Acoustic_Table[19];
        break;

        case TTY_FULL:
            table = &Phone_Acoustic_Table[20];
        break;

        case TTY_VCO:
            table = &Phone_Acoustic_Table[21];
        break;

        case TTY_HCO:
            table = &Phone_Acoustic_Table[22];
        break;

        case REC_INC_MIC:
            table = &Phone_Acoustic_Table[23];
        break;

        case REC_EXT_MIC:
            table = &Phone_Acoustic_Table[24];
        break;

        case PLAYBACK_HEADSET:
            table = &Phone_Acoustic_Table[25];
        break;

        case PLAYBACK_HANDSFREE:
            table = &Phone_Acoustic_Table[26];
        break;

        default:
            LOGE("Unknown out_path");
        break;
    }

    if ( table ) {
        mCurrentSndDevice = out_path;
        if (ioctl(acousticfd, ACOUSTIC_UPDATE_VOLUME_TABLE, &(table->array) ) < 0) {
            LOGE("ACOUSTIC_UPDATE_VOLUME_TABLE error.");
            return -EIO;
        }

        /* TODO : Look at UpdateCeTable from CE dll
         * TODO : Is it really usefull as the table for all devices is filled with 0's
         */
        if ( mCurrentSndDevice < SYS ) {
            ce_table = &CE_Acoustic_Table[out_path];
            if (ioctl(acousticfd, ACOUSTIC_UPDATE_CE_TABLE, &(ce_table->array) ) < 0) {
                LOGE("ACOUSTIC_UPDATE_CE_TABLE error.");
                return -EIO;
            }
        }

        if (ioctl(acousticfd, ACOUSTIC_ARM11_DONE, NULL ) < 0) {
            LOGE("ACOUSTIC_ARM11_DONE error.");
            return -EIO;
        }
    }

    return 0;
}

int msm72xx_set_audio_path(bool bEnableMic, bool bEnableDualMic,
                           int device_out, bool bEnableOut)
{
    struct msm_audio_path audio_path = {.bEnableMic = bEnableMic, .bEnableDualMic = bEnableDualMic};

    if ( bEnableOut ) {
        if ( (device_out == SND_DEVICE_SPEAKER) || (device_out == SND_DEVICE_PLAYBACK_HANDSFREE) ) {
            audio_path.bEnableSpeaker = true;
        } else {
            audio_path.bEnableSpeaker = false;
        }

        if ( device_out == SND_DEVICE_HEADSET ) {
            audio_path.bEnableHeadset = true;
        } else {
            audio_path.bEnableHeadset = false;
        }
    } else {
        audio_path.bEnableSpeaker = false;
        audio_path.bEnableHeadset = false;
    }

    LOGV("bEnableMic = %d, bEnableDualMic = %d, bEnableSpeaker = %d, bEnableHeadset = %d",
            bEnableMic, bEnableDualMic, audio_path.bEnableSpeaker, audio_path.bEnableHeadset);

    if ( bEnableMic ) {
        UpdateAudioAdieTable(1, 0, 0, 0);
    } else {
        UpdateAudioAdieTable(0, 0, 0, 0);
    }

    if (ioctl(acousticfd, ACOUSTIC_SET_HW_AUDIO_PATH, &audio_path ) < 0) {
        LOGE("ACOUSTIC_SET_HW_AUDIO_PATH error.");
        return -EIO;
    } 

    return 0;
}

