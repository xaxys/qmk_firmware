// Copyright 2023 Persama (@Persama)
// SPDX-License-Identifier: GPL-2.0-or-later
#pragma once

#include "quantum.h"

enum custom_keycodes {
    RF_DFU = QK_KB_0,
    LNK_USB,
    LNK_RF,
    LNK_BLE1,
    LNK_BLE2,
    LNK_BLE3,

    MAC_TASK,
    MAC_SEARCH,
    MAC_VOICE,
    MAC_CONSOLE,
    MAC_DND,
    MAC_PRT,
    MAC_PRTA,

    DEV_RESET,
    SLEEP_MODE,
    BAT_SHOW,

    SIDE_VAI,
    SIDE_VAD,
    SIDE_MOD_A,
    SIDE_MOD_B,
    SIDE_HUI,
    SIDE_SPI,
    SIDE_SPD,

    // Present in the current NuPhy VIA definition and factory layout, but
    // absent from the published Halo65 V2 source implementation. Reserve the
    // numeric slots so new features never hijack the factory keycodes.
    CODE_MODE,
    PAINT_MODE,

    FN_SWITCH,
    FN_LOCK,
    TAP_HOLD_1,
    DBL_TAP_1,
    TAP_HOLD_2,
    DBL_TAP_2,
};

// Little-endian EEPROM bytes spell "HFN3" (Halo Fn features, schema 3).
// A mismatch disables the optional features instead of trusting stale bytes.
#define FN_FEATURES_EEPROM_MAGIC 0x334E4648UL

// VIA's custom-key list is positional. Fail the build if an enum edit would
// silently make the JSON labels select different firmware actions.
_Static_assert(CODE_MODE == 0x7E17, "CODE_MODE must retain its factory VIA keycode");
_Static_assert(PAINT_MODE == 0x7E18, "PAINT_MODE must retain its factory VIA keycode");
_Static_assert(FN_SWITCH == 0x7E19, "FN_SWITCH must match the VIA custom keycode order");
_Static_assert(FN_LOCK == 0x7E1A, "FN_LOCK must match the VIA custom keycode order");
_Static_assert(TAP_HOLD_1 == 0x7E1B, "TAP_HOLD_1 must match the VIA custom keycode order");
_Static_assert(DBL_TAP_1 == 0x7E1C, "DBL_TAP_1 must match the VIA custom keycode order");
_Static_assert(TAP_HOLD_2 == 0x7E1D, "TAP_HOLD_2 must match the VIA custom keycode order");
_Static_assert(DBL_TAP_2 == 0x7E1E, "DBL_TAP_2 must match the VIA custom keycode order");

extern uint8_t m_sleep_led;

typedef enum {
    RX_Idle,        
    RX_Receiving,   
    RX_Done,        
    RX_Fail,        
    RX_OV_ERR,      
    RX_SUM_ERR,     
    RX_CMD_ERR,    
    RX_DATA_ERR,   
    RX_DATA_OV,     
    RX_FORMAT_ERR,  

    TX_OK = 0XE0,  
    TX_DONE,       
    TX_BUSY,      
    TX_TIMEOUT,    
    TX_DATA_ERR,   

} TYPE_RX_STATE;

#define FUNC_VALID_LEN   32
#define UART_HEAD        0x5A

#define RF_IDLE          0    
#define RF_PAIRING       1     
#define RF_LINKING       2    
#define RF_CONNECT       3    
#define RF_DISCONNECT    4    
#define RF_SLEEP         5   
#define RF_SNIF          6    
#define RF_INVALID       0XFE  
#define RF_ERR_STATE     0XFF  

#define RF_LONG_PRESS_DELAY   30
#define DEV_RESET_PRESS_DELAY 30
#define RGB_TEST_PRESS_DELAY  30

#define CMD_POWER_UP     0XF0  
#define CMD_SLEEP        0XF1  
#define CMD_HAND         0XF2 
#define CMD_SNIF         0XF3  
#define CMD_24G_SUSPEND  0XF4 
#define CMD_IDLE_EXIT    0XFE  

#define CMD_RPT_MS       0XE0  
#define CMD_RPT_BYTE_KB  0XE1
#define CMD_RPT_BIT_KB   0XE2
#define CMD_RPT_CONSUME  0XE3  
#define CMD_RPT_SYS      0XE4 

#define CMD_SET_LINK     0XC0  
#define CMD_SET_CONFIG   0XC1 
#define CMD_GET_CONFIG   0XC2  
#define CMD_SET_NAME     0XC3  
#define CMD_GET_NAME     0XC4  
#define CMD_CLR_DEVICE   0XC5  
#define CMD_NEW_ADV      0XC7 
#define CMD_RF_STS_SYSC  0XC9  
#define CMD_SET_24G_NAME 0XCA  
#define CMD_GO_TEST      0XCF 

#define CMD_RF_DFU       0XB1  

#define CMD_WRITE_DATA   0X80  
#define CMD_READ_DATA    0X81  

#define CMD_WBAT_CFG    0X82  
#define CMD_RBAT_CFG    0X83  

#define LINK_RF_24       0 
#define LINK_BT_1        1  
#define LINK_BT_2        2  
#define LINK_BT_3        3 
#define LINK_USB         4  

#define UART_MAX_LEN     64
typedef struct
{
    uint8_t RXDState;              
    uint8_t RXDLen;               
    uint8_t RXDOverTime;           
    uint8_t TXDLenBack;            
    uint8_t TXDOffset;
    uint8_t TXDBuf[UART_MAX_LEN]; 
    uint8_t RXDBuf[UART_MAX_LEN];  
} USART_MGR_STRUCT;

typedef struct
{
    uint8_t link_mode;    
    uint8_t rf_channel;   
    uint8_t ble_channel; 
    uint8_t rf_state;    
    uint8_t rf_charge;    
    uint8_t rf_led;        
    uint8_t rf_baterry;    
    uint8_t sys_sw_state;  
} DEV_INFO_STRUCT;

#define DELAY_2MS         2     
#define DELAY_4MS         4     
#define DELAY_5MS         5   
#define DELAY_6MS         6    
#define DELAY_8MS         8    
#define DELAY_10MS        10
#define DELAY_15MS        15
#define DELAY_20MS        20
#define DELAY_30MS        30
#define DELAY_40MS        40
#define DELAY_50MS        50
#define DELAY_100MS       100
#define DELAY_200MS       200
#define DELAY_300MS       300
#define DELAY_400MS       400
#define DELAY_500MS       500
#define DELAY_800MS       800
#define DELAY_1SEC        1000
#define DELAY_2SEC        2000
#define DELAY_3SEC        3000
#define DELAY_4SEC        4000
#define DELAY_5SEC        5000

#define SYS_SW_WIN        0xa1
#define SYS_SW_MAC        0xa2

#define RF_LINK_SHOW_TIME 300

#define HOST_USB_TYPE     0
#define HOST_BLE_TYPE     1
#define HOST_RF_TYPE      2

#if (1)
#define LINK_TIMEOUT     (uint32_t)(100 * 120)
#define SLEEP_TIME_DELAY (uint32_t)(100 * 360)
#define POWER_DOWN_DELAY (uint16_t)(24)

#else
#define LINK_TIMEOUT     (100 * 10)
#define SLEEP_TIME_DELAY (100 * 30)
#define POWER_DOWN_DELAY (30)

#endif

typedef struct
{
    uint8_t bit0: 1;
    uint8_t bit1: 1;
    uint8_t bit2: 1;
    uint8_t bit3: 1;
    uint8_t bit4: 1;
    uint8_t bit5: 1;
    uint8_t bit6: 1;
    uint8_t bit7: 1;
}m_8bit;

typedef struct
{
    uint8_t default_brightness_flag;
    uint8_t ee_side_mode_a;
    uint8_t ee_side_mode_b;
    uint8_t ee_side_light;
    uint8_t ee_side_speed;
    uint8_t ee_side_rgb;
    uint8_t ee_side_colour;
    m_8bit ee_dev_config;
    uint32_t fn_features_magic;
} user_config_t;

_Static_assert(sizeof(user_config_t) == EECONFIG_USER_DATA_SIZE, "user_config_t must match the configured EEPROM user data size");
_Static_assert(offsetof(user_config_t, fn_features_magic) == 8, "Fn feature magic must start at EEPROM user byte 8");
_Static_assert((offsetof(user_config_t, fn_features_magic) % _Alignof(uint32_t)) == 0, "Fn feature magic must be 32-bit aligned");
_Static_assert(_Alignof(user_config_t) >= _Alignof(uint32_t), "user_config_t must retain 32-bit alignment");

static inline bool fn_features_magic_is_valid(const user_config_t *config) {
    return config->fn_features_magic == FN_FEATURES_EEPROM_MAGIC;
}

static inline void fn_features_magic_set(user_config_t *config) {
    config->fn_features_magic = FN_FEATURES_EEPROM_MAGIC;
}

extern user_config_t user_config;
#define f_dev_sleep_enable  user_config.ee_dev_config.bit0
#define f_fn_switch         user_config.ee_dev_config.bit1
#define f_fn_lock           user_config.ee_dev_config.bit2
