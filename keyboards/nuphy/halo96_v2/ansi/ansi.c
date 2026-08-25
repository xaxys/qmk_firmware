/*
Copyright 2023 @ Nuphy <https://nuphy.com/>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ansi.h"
#include "usb_main.h"
#include "rf_driver.h"
#include "keymap_introspection.h"
#ifdef VIA_ENABLE
#    include "dynamic_keymap.h"
#    include "via.h"
#endif


user_config_t user_config; 
DEV_INFO_STRUCT dev_info =
    {
        .rf_baterry = 100,
        .link_mode  = LINK_USB,
        .rf_state   = RF_IDLE,
};

uint16_t rf_linking_time            = 0;   
uint16_t rf_link_show_time          = 0; 
uint8_t rf_blink_cnt = 0;     
uint16_t no_act_time                = 0;  
host_driver_t *m_host_driver        = 0;
uint16_t dev_reset_press_delay = 0; 
uint16_t rf_sw_press_delay     = 0;  
uint16_t rgb_test_press_delay  = 0;  
uint8_t rf_sw_temp = 0;
uint8_t rgb_light_old = 0;
uint8_t host_mode;

extern uint8_t side_mode_a;  
extern uint8_t side_light;  
extern uint8_t side_speed; 
extern uint8_t side_rgb;   
extern uint8_t side_colour;
extern report_keyboard_t *keyboard_report;
extern report_nkro_t *nkro_report;
extern uint8_t side_mode_b;   
extern uint8_t uart_bit_report_buf[32];
extern uint8_t bitkb_report_buf[32];
extern uint8_t bytekb_report_buf[8]; 

bool f_uart_ack         = 0; 
bool f_bat_show         = 0; 
bool f_bat_hold         = 0;
bool f_chg_show         = 1;
bool f_sys_show         = 0;
bool f_sleep_show       = 0; 
bool f_usb_offline      = 0; 
bool f_rf_read_data_ok  = 0;
bool f_rf_sts_sysc_ok   = 0;
bool f_rf_new_adv_ok    = 0;
bool f_rf_reset         = 0;
bool f_send_channel     = 0;  
bool f_rf_hand_ok       = 0;  
bool f_rf_send_bitkb    = 0; 
bool f_rf_send_byte     = 0; 
bool f_rf_send_consume  = 0; 
bool f_wakeup_prepare   = 0;
bool f_dial_sw_init_ok  = 0;  
bool f_goto_sleep       = 0;  
bool f_rf_sw_press      = 0;  
bool f_dev_reset_press  = 0;  
bool f_rgb_test_press   = 0;  
bool f_win_lock         = 0;  

// Release before this many milliseconds for VIA Macro 0. Reaching the term
// while the key remains held emits VIA Macro 1 immediately.
#define TAP_HOLD_TERM 500
#define FN_ROW_FIRST_COL 1
#define FN_ROW_LAST_COL 12
#define FN_ROW_KEY_COUNT (FN_ROW_LAST_COL - FN_ROW_FIRST_COL + 1)

// Layer and matrix constants. Single source of truth for the Fn Lock
// F-row logic, so layout numbers never need to be hunted for.
#define LAYER_MAC_BASE     0
#define LAYER_MAC_FN       1
#define LAYER_WIN_BASE     2
#define LAYER_WIN_FN       3
#define FN_KEY_ROW         0   // matrix row holding F1-F12

static bool fn_held = false;
static bool fn_row_key_active[FN_ROW_KEY_COUNT];
static uint16_t fn_row_keycode[FN_ROW_KEY_COUNT];
static uint16_t tap_hold_timer;
static bool tap_hold_active;
static bool tap_hold_hold_sent;
static bool fn_features_validate_pending;

// Tap/Hold 2 (second slot): tap -> VIA Macro 3, hold 500 ms -> Macro 4.
static uint16_t tap_hold_timer_2;
static bool     tap_hold_active_2;
static bool     tap_hold_hold_sent_2;

// Dynamic state-indicator cache. Rescanned periodically from the current
// base layer so the lit keys follow VIA remaps (Caps/Tap-Hold/Fn).
#define IND_MAX_POS 4
static uint8_t  ind_caps_pos[IND_MAX_POS];
static uint8_t  ind_caps_count;
static uint8_t  ind_tap_hold_pos[IND_MAX_POS];
static uint8_t  ind_tap_hold_count;
static uint8_t  ind_fn_pos;
static uint32_t ind_rescan_timer;

// RGB Matrix state-indicator colours (Caps Lock green, Tap/Hold blue,
// Fn Lock orange).
#define IND_CAPS_LOCK_R 0x00
#define IND_CAPS_LOCK_G 0xFF
#define IND_CAPS_LOCK_B 0x00
#define IND_TAP_HOLD_R  0x00
#define IND_TAP_HOLD_G  0x80
#define IND_TAP_HOLD_B  0xFF
#define IND_FN_LOCK_R   0xFF
#define IND_FN_LOCK_G   0x40
#define IND_FN_LOCK_B   0x00

void rf_device_init(void);
void rf_uart_init(void);
void m_side_led_show(void);
void dev_sts_sync(void);
void uart_receive_pro(void);
void Sleep_Handle(void);
void uart_send_report_func(void);
uint8_t uart_send_cmd(uint8_t cmd, uint8_t ack_cnt, uint8_t delayms);
void uart_send_report(uint8_t report_type, uint8_t *report_buf, uint8_t report_size);
void device_reset_show(void);
void device_reset_init(void);
void rgb_test_show(void);
void m_deinit_usb_072(void);

extern void light_speed_control(uint8_t fast);
extern void light_level_control(uint8_t brighten);
extern void side_colour_control(uint8_t dir);
extern void side_mode_a_control(uint8_t dir);
extern void side_mode_b_control(uint8_t dir);
extern bool low_bat_flag;


static void fn_features_save_config(void) {
    fn_features_magic_set(&user_config);
    eeconfig_update_user_datablock(&user_config);
}

static void fn_row_release_all(void) {
    for (uint8_t index = 0; index < FN_ROW_KEY_COUNT; index++) {
        if (fn_row_key_active[index]) {
            unregister_code16(fn_row_keycode[index]);
            fn_row_key_active[index] = false;
        }
    }
}

static void fn_features_disable(void) {
    bool changed = f_fn_lock || !fn_features_magic_is_valid(&user_config);

    f_fn_lock = false;
    tap_hold_active = false;
    tap_hold_hold_sent = false;
    tap_hold_active_2 = false;
    tap_hold_hold_sent_2 = false;
    fn_row_release_all();

    if (changed) {
        fn_features_save_config();
    }
}

#ifdef VIA_ENABLE
static bool dynamic_keymap_contains_keycode(uint16_t keycode) {
    for (uint8_t layer = 0; layer < DYNAMIC_KEYMAP_LAYER_COUNT; layer++) {
        for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
            for (uint8_t col = 0; col < MATRIX_COLS; col++) {
                if (dynamic_keymap_get_keycode(layer, row, col) == keycode) {
                    return true;
                }
            }
        }
    }

    return false;
}

static void fn_features_validate_configuration(void) {
    bool changed = false;

    if (f_fn_lock && !dynamic_keymap_contains_keycode(FN_LOCK)) {
        f_fn_lock = false;
        fn_row_release_all();
        changed = true;
    }

    if (changed) {
        fn_features_save_config();
    }
}
#else
static void fn_features_validate_configuration(void) {
    fn_features_disable();
}
#endif

static bool fn_row_keycode_supported(uint16_t keycode) {
    return keycode != KC_NO && keycode != KC_TRNS && (IS_QK_BASIC(keycode) || IS_QK_MODS(keycode));
}

#ifdef VIA_ENABLE
bool via_command_kb(uint8_t *data, uint8_t length) {
    if (data == NULL || length == 0) {
        return false;
    }

    switch (data[0]) {
        case id_dynamic_keymap_reset:
        case id_eeprom_reset:
            fn_features_disable();
            break;

        case id_dynamic_keymap_set_keycode:
        case id_dynamic_keymap_set_buffer:
            fn_features_validate_pending = true;
            break;

        default:
            break;
    }

    return false;
}
#endif


/**
 * @brief  gpio initial.
 */
void m_gpio_init(void)
{
    setPinOutput(DC_BOOST_PIN); writePinHigh(DC_BOOST_PIN);

    // Initializes the RGB Driver SDB pin
    setPinOutput(RGB_DRIVER_SDB1); writePinHigh(RGB_DRIVER_SDB1);
    setPinOutput(RGB_DRIVER_SDB2); writePinHigh(RGB_DRIVER_SDB2);

    // RF wake up pin configuration
    setPinOutput(NRF_WAKEUP_PIN);
    writePinHigh(NRF_WAKEUP_PIN);

    // RFboot Control pin
    setPinInputHigh(NRF_BOOT_PIN);

    // RF reset pin configuration
    setPinOutput(NRF_RESET_PIN); writePinLow(NRF_RESET_PIN);
    wait_ms(50);
    writePinHigh(NRF_RESET_PIN);

    // Switch detection pin
    setPinInputHigh(DEV_MODE_PIN);
    setPinInputHigh(SYS_MODE_PIN);
}

/**
 * @brief  long press key process.
 */
void long_press_key(void)
{
    static uint32_t long_press_timer = 0;

    if (timer_elapsed32(long_press_timer) < 100) return;
    long_press_timer = timer_read32();

    if (f_rf_sw_press) {
        rf_sw_press_delay++;
        if (rf_sw_press_delay >= RF_LONG_PRESS_DELAY) {
            f_rf_sw_press        = 0;
            dev_info.link_mode   = rf_sw_temp;
            dev_info.rf_channel  = rf_sw_temp;
            dev_info.ble_channel = rf_sw_temp;

            uint8_t timeout = 5;
            while (timeout--) {
                uart_send_cmd(CMD_NEW_ADV, 0, 1);  
                wait_ms(20);                   
                uart_receive_pro();
                if (f_rf_new_adv_ok) break;
            }
        }
    } else {
        rf_sw_press_delay = 0;
    }

    if (f_dev_reset_press) {
        dev_reset_press_delay++;
        if (dev_reset_press_delay >= DEV_RESET_PRESS_DELAY)  {
            f_dev_reset_press = 0;

            if (dev_info.link_mode != LINK_USB) {
                if (dev_info.link_mode != LINK_RF_24) {
                    dev_info.link_mode      = LINK_BT_1;
                    dev_info.ble_channel    = LINK_BT_1;
                    dev_info.rf_channel     = LINK_BT_1;
                } 
            } else {
                dev_info.ble_channel = LINK_BT_1;
            }

            uart_send_cmd(CMD_SET_LINK, 10, 10);   
            wait_ms(500);                          
            uart_send_cmd(CMD_CLR_DEVICE, 10, 10);  

            eeconfig_init();     
            device_reset_show(); 
            device_reset_init(); 

            keymap_config.no_gui = 0;
            f_win_lock = 0;

            if (dev_info.sys_sw_state == SYS_SW_MAC) {
                default_layer_set(1 << LAYER_MAC_BASE);  // MAC
                keymap_config.nkro = 0;   
            } else {
                default_layer_set(1 << LAYER_WIN_BASE);  // WIN
                keymap_config.nkro = 1;    
            }
        }
    } else {
        dev_reset_press_delay = 0;
    }

    if (f_rgb_test_press) {
        rgb_test_press_delay++;
        if (rgb_test_press_delay >= RGB_TEST_PRESS_DELAY) {
            f_rgb_test_press = 0;
            rgb_test_show();  
        }
    } else {
        rgb_test_press_delay = 0;
    }
}

/**
 * @brief  Release all keys, clear keyboard report.
 */
void m_break_all_key(void)
{
    uint8_t report_buf[16];
    bool nkro_temp = keymap_config.nkro; 

    // Keep the optional Fn state in sync with NuPhy's forced report clear
    // when changing OS/link mode or resetting the device.
    fn_held = false;
    tap_hold_active = false;
    tap_hold_hold_sent = false;
    tap_hold_active_2 = false;
    tap_hold_hold_sent_2 = false;
    fn_row_release_all();

    clear_weak_mods();
    clear_mods();
    clear_keyboard();

    keymap_config.nkro = 1;
    memset(nkro_report, 0, sizeof(report_nkro_t));
    host_nkro_send(nkro_report);
    wait_ms(10);

    keymap_config.nkro = 0;
    memset(keyboard_report, 0, sizeof(report_keyboard_t));
    host_keyboard_send(keyboard_report);
    wait_ms(10);

    keymap_config.nkro = nkro_temp;

    if (dev_info.link_mode != LINK_USB) {
        memset(report_buf, 0, 16);
        uart_send_report(CMD_RPT_BIT_KB, report_buf, 16);
        wait_ms(10);
        uart_send_report(CMD_RPT_BYTE_KB, report_buf, 8);
        wait_ms(10);
    }

    memset(uart_bit_report_buf, 0, sizeof(uart_bit_report_buf));
    memset(bitkb_report_buf, 0, sizeof(bitkb_report_buf));
    memset(bytekb_report_buf, 0, sizeof(bytekb_report_buf));
}

/**
 * @brief  switch device link mode.
 * @param mode : link mode
 */
static void switch_dev_link(uint8_t mode)
{
    if (mode > LINK_USB) return;
    m_break_all_key();

    dev_info.link_mode = mode;
    dev_info.rf_state = RF_IDLE;
    f_send_channel    = 1;

    if (mode == LINK_USB) {
        host_mode = HOST_USB_TYPE;
        host_set_driver(m_host_driver);
        rf_link_show_time = 0;
    }
    else {
        host_mode = HOST_RF_TYPE;
        host_set_driver(&rf_host_driver);
    }
}


/**
 * @brief  scan dial switch.
 */
void dial_sw_scan(void)
{
    uint8_t dial_scan               = 0;
    static uint8_t dial_save        = 0xf0;
    static uint8_t debounce         = 0;
    static uint32_t dial_scan_timer = 0;
    static bool flag_power_on       = 1;

    if (!flag_power_on) {
        if (timer_elapsed32(dial_scan_timer) < 20) return;
    }
    dial_scan_timer = timer_read32();

    setPinInputHigh(DEV_MODE_PIN);
    setPinInputHigh(SYS_MODE_PIN);

    if (readPin(DEV_MODE_PIN)) dial_scan |= 0X01;
    if (readPin(SYS_MODE_PIN)) dial_scan |= 0X02;

    if (dial_save != dial_scan) {
        m_break_all_key();      
        dial_save         = dial_scan;
        no_act_time         = 0;   
        rf_linking_time     = 0;   
        debounce            = 25; 
        f_dial_sw_init_ok   = 0;    
        return;
    } else if (debounce) {
        debounce--;
        return;
    }

    if (dial_scan & 0x01) {
        if (dev_info.link_mode != LINK_USB) {
            switch_dev_link(LINK_USB);
        }
    } else {
        if (dev_info.link_mode != dev_info.rf_channel) {
            switch_dev_link(dev_info.rf_channel);
        }
    }

    if (dial_scan & 0x02) {
        if (dev_info.sys_sw_state != SYS_SW_WIN) {
            f_sys_show = 1;
            default_layer_set(1 << LAYER_WIN_BASE);
            dev_info.sys_sw_state = SYS_SW_WIN;
            keymap_config.no_gui  = f_win_lock;
            m_break_all_key();  
        }
        keymap_config.nkro = 1; 
    } else {
        if (dev_info.sys_sw_state != SYS_SW_MAC) {
            f_sys_show = 1;
            default_layer_set(1 << LAYER_MAC_BASE);
            dev_info.sys_sw_state = SYS_SW_MAC;
            f_win_lock            = keymap_config.no_gui;
            m_break_all_key();
        }
        keymap_config.nkro   = 0; 
        keymap_config.no_gui = 0;
    }

    if (f_dial_sw_init_ok == 0) {
        f_dial_sw_init_ok = 1; 
        flag_power_on     = 0;

        if (dev_info.link_mode != LINK_USB) {
            host_set_driver(&rf_host_driver);
        }
    }
}

/**
 * @brief  power on scan dial switch.
 */
void m_power_on_dial_sw_scan(void)
{
    uint8_t dial_scan_dev = 0;
    uint8_t dial_scan_sys = 0;
    uint8_t dial_check_dev = 0;
    uint8_t dial_check_sys = 0;
    uint8_t debounce = 0;

    f_win_lock = 0;
    setPinInputHigh(DEV_MODE_PIN);    
    setPinInputHigh(SYS_MODE_PIN);  

    for(debounce=0; debounce<10; debounce++) {
        dial_scan_dev = 0;
        dial_scan_sys = 0;
        if (readPin(DEV_MODE_PIN))  dial_scan_dev = 0x01;
        else                        dial_scan_dev = 0;
        if (readPin(SYS_MODE_PIN))  dial_scan_sys = 0x01;
        else                        dial_scan_sys = 0;
        if((dial_scan_dev != dial_check_dev)||(dial_scan_sys != dial_check_sys))
        {
            dial_check_dev = dial_scan_dev;
            dial_check_sys = dial_scan_sys;
            debounce = 0;
        }
        wait_ms(1);
    }
    if (dial_scan_dev) {
        if (dev_info.link_mode != LINK_USB) {
            switch_dev_link(LINK_USB);
        }
    } else {
        if (dev_info.link_mode != dev_info.rf_channel) {
            switch_dev_link(dev_info.rf_channel);
        }
    }
    // WIN/MAC 
    if (dial_scan_sys) {
        if (dev_info.sys_sw_state != SYS_SW_WIN) {
            default_layer_set(1 << LAYER_WIN_BASE);  // WIN
            dev_info.sys_sw_state = SYS_SW_WIN;
            keymap_config.nkro    = 1; 
            m_break_all_key();  
        }
    } else {
        if (dev_info.sys_sw_state != SYS_SW_MAC) {
            default_layer_set(1 << LAYER_MAC_BASE);  // MAC
            dev_info.sys_sw_state = SYS_SW_MAC;
            keymap_config.nkro    = 0; 
            f_win_lock            = keymap_config.no_gui;
            keymap_config.no_gui  = 0;
            m_break_all_key();  
        }
    }
}



/**
 * @brief  qmk process record
 */
bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if(!process_record_user(keycode, record)){
        return false;
    }
    no_act_time = 0;

    if (fn_features_validate_pending) {
        fn_features_validate_pending = false;
        fn_features_validate_configuration();
    }

    if (record->event.key.row == FN_KEY_ROW && record->event.key.col >= FN_ROW_FIRST_COL && record->event.key.col <= FN_ROW_LAST_COL) {
        uint8_t index = record->event.key.col - FN_ROW_FIRST_COL;

        if (!record->event.pressed && fn_row_key_active[index]) {
            unregister_code16(fn_row_keycode[index]);
            fn_row_key_active[index] = false;
            return false;
        }

        if (record->event.pressed && (dev_info.sys_sw_state == SYS_SW_MAC || dev_info.sys_sw_state == SYS_SW_WIN)) {
            bool remap = false;
            bool is_mac = dev_info.sys_sw_state == SYS_SW_MAC;
            uint8_t fn_layer       = is_mac ? LAYER_MAC_FN       : LAYER_WIN_FN;
            uint8_t base_layer     = is_mac ? LAYER_MAC_BASE     : LAYER_WIN_BASE;
            uint8_t target_layer   = fn_layer;

            if (f_fn_lock && fn_held) {
                // Fn Lock + Fn held: give the real base-layer key so the
                // user can still type the F-key while locked.
                target_layer = base_layer;
                remap        = true;
            } else if (f_fn_lock && !fn_held) {
                // Fn Lock without Fn: the F1-F12 row behaves as if Fn were
                // held.
                target_layer = fn_layer;
                remap        = true;
            }

            if (remap) {
                uint16_t target_keycode = keycode_at_keymap_location(
                    target_layer,
                    record->event.key.row,
                    record->event.key.col
                );

                if (fn_row_keycode_supported(target_keycode)) {
                    fn_row_keycode[index]    = target_keycode;
                    fn_row_key_active[index] = true;
                    register_code16(target_keycode);
                    return false;
                }
            }
        }
    }

    switch (keycode) {
        case MO(LAYER_MAC_FN):
            if (dev_info.sys_sw_state != SYS_SW_MAC) {
                if (!record->event.pressed) {
                    fn_held = false;
                }
                return true;
            }

            if (record->event.pressed) {
                fn_held = true;
            } else {
                fn_held = false;
            }
            return true;

        case MO(LAYER_WIN_FN):
            if (dev_info.sys_sw_state != SYS_SW_WIN) {
                if (!record->event.pressed) {
                    fn_held = false;
                }
                return true;
            }

            if (record->event.pressed) {
                fn_held = true;
            } else {
                fn_held = false;
            }
            return true;

        case FN_LOCK:
            if (record->event.pressed && (dev_info.sys_sw_state == SYS_SW_MAC || dev_info.sys_sw_state == SYS_SW_WIN)) {
                f_fn_lock = !f_fn_lock;
                fn_features_save_config();
            }
            return false;

        case TAP_HOLD_1:
            if (record->event.pressed) {
                tap_hold_timer     = timer_read();
                tap_hold_active    = true;
                tap_hold_hold_sent = false;
            } else if (tap_hold_active) {
                tap_hold_active = false;
                if (!tap_hold_hold_sent) {
#ifdef VIA_ENABLE
                    dynamic_keymap_macro_send(timer_elapsed(tap_hold_timer) >= TAP_HOLD_TERM ? 1 : 0);
#else
                    if (timer_elapsed(tap_hold_timer) >= TAP_HOLD_TERM) {
                        tap_code(KC_CAPS);
                    } else {
                        tap_code16(LCTL(KC_SPC));
                    }
#endif
                }
                tap_hold_hold_sent = false;
            }
            return false;

        case TAP_HOLD_2:
            if (record->event.pressed) {
                tap_hold_timer_2     = timer_read();
                tap_hold_active_2    = true;
                tap_hold_hold_sent_2 = false;
            } else if (tap_hold_active_2) {
                tap_hold_active_2 = false;
                if (!tap_hold_hold_sent_2) {
#ifdef VIA_ENABLE
                    dynamic_keymap_macro_send(timer_elapsed(tap_hold_timer_2) >= TAP_HOLD_TERM ? 4 : 3);
#else
                    if (timer_elapsed(tap_hold_timer_2) >= TAP_HOLD_TERM) {
                        tap_code(KC_CAPS);
                    } else {
                        tap_code16(LCTL(KC_SPC));
                    }
#endif
                }
                tap_hold_hold_sent_2 = false;
            }
            return false;
        case RF_DFU:
            if (record->event.pressed) {
                if (dev_info.link_mode != LINK_USB) return false;
                uart_send_cmd(CMD_RF_DFU, 10, 20);
            }
            return false;

        case LNK_USB:
            if (record->event.pressed) {
                m_break_all_key();
            } else {
                dev_info.link_mode = LINK_USB;
                uart_send_cmd(CMD_SET_LINK, 10, 10);
            }
            return false;

        case LNK_RF:
            if (record->event.pressed) {
                if (dev_info.link_mode != LINK_USB) {
                    rf_sw_temp    = LINK_RF_24;
                    f_rf_sw_press = 1;
                    m_break_all_key();
                }
            } else if (f_rf_sw_press) {
                f_rf_sw_press = 0;
                if (rf_sw_press_delay < RF_LONG_PRESS_DELAY) {
                    dev_info.link_mode   = rf_sw_temp;
                    dev_info.rf_channel  = rf_sw_temp;
                    dev_info.ble_channel = rf_sw_temp;
                    uart_send_cmd(CMD_SET_LINK, 10, 20); 
                }
            }
            return false;

        case LNK_BLE1:
            if (record->event.pressed) {
                if (dev_info.link_mode != LINK_USB) {
                    rf_sw_temp    = LINK_BT_1;
                    f_rf_sw_press = 1;
                    m_break_all_key();
                }
            } else if (f_rf_sw_press) {
                f_rf_sw_press = 0;
                if (rf_sw_press_delay < RF_LONG_PRESS_DELAY) {
                    dev_info.link_mode   = rf_sw_temp;
                    dev_info.rf_channel  = rf_sw_temp;
                    dev_info.ble_channel = rf_sw_temp;
                    uart_send_cmd(CMD_SET_LINK, 10, 20);
                }
            }
            return false;

        case LNK_BLE2:
            if (record->event.pressed) {
                if (dev_info.link_mode != LINK_USB) {
                    rf_sw_temp    = LINK_BT_2;
                    f_rf_sw_press = 1;
                    m_break_all_key();
                }
            } else if (f_rf_sw_press) {
                f_rf_sw_press = 0;
                if (rf_sw_press_delay < RF_LONG_PRESS_DELAY) {
                    dev_info.link_mode   = rf_sw_temp;
                    dev_info.rf_channel  = rf_sw_temp;
                    dev_info.ble_channel = rf_sw_temp;
                    uart_send_cmd(CMD_SET_LINK, 10, 20);  
                }
            }
            return false;

        case LNK_BLE3:
            if (record->event.pressed) {
                if (dev_info.link_mode != LINK_USB) {
                    rf_sw_temp    = LINK_BT_3;
                    f_rf_sw_press = 1;
                    m_break_all_key();
                }
            } else if (f_rf_sw_press) {
                f_rf_sw_press = 0;
                if (rf_sw_press_delay < RF_LONG_PRESS_DELAY) {
                    dev_info.link_mode   = rf_sw_temp;
                    dev_info.rf_channel  = rf_sw_temp;
                    dev_info.ble_channel = rf_sw_temp;
                    uart_send_cmd(CMD_SET_LINK, 10, 20);
                }
            }
            return false;

        case MAC_TASK:
            if (record->event.pressed) {
                host_consumer_send(0x029F);
            } else {
                host_consumer_send(0);
            }
            return false;

        case MAC_SEARCH:
            if (record->event.pressed) {
                register_code(KC_LGUI);
                register_code(KC_SPACE);
                wait_ms(50);
                unregister_code(KC_LGUI);
                unregister_code(KC_SPACE);
            }
            return false;

        case MAC_VOICE:
            if (record->event.pressed) {
                host_consumer_send(0xcf);
            } else {
                host_consumer_send(0);
            }
            return false;

        case MAC_CONSOLE:
            if (record->event.pressed) {
                host_consumer_send(0x02A0);
            } else {
                host_consumer_send(0);
            }
            return false;

        case MAC_DND:
            if (record->event.pressed) {
                host_system_send(0x9b);
            } else {
                host_system_send(0);
            }
            return false;

        case MAC_PRT:
            if (record->event.pressed) {
                register_code(KC_LGUI);
                register_code(KC_LSFT);
                register_code(KC_3);
                wait_ms(50);
                unregister_code(KC_3);
                unregister_code(KC_LSFT);
                unregister_code(KC_LGUI);
            }
            return false;

        case MAC_PRTA:
            if (record->event.pressed) {
                // win
                if (keymap_config.nkro) {
                    register_code(KC_LGUI);
                    register_code(KC_LSFT);
                    register_code(KC_S);
                    wait_ms(50);
                    unregister_code(KC_S);
                    unregister_code(KC_LSFT);
                    unregister_code(KC_LGUI);
                }
                // mac
                else {
                    register_code(KC_LGUI);
                    register_code(KC_LSFT);
                    register_code(KC_4);
                    wait_ms(50);
                    unregister_code(KC_4);
                    unregister_code(KC_LSFT);
                    unregister_code(KC_LGUI);
                }
            }
            return false;

        case SIDE_VAI:
            if (record->event.pressed) {
                if(low_bat_flag && (side_light == 1)) return false;
                light_level_control(1);
            }
            return false;

        case SIDE_VAD:
            if (record->event.pressed) {
                light_level_control(0);
            }
            return false;

        case SIDE_MOD_A:
            if (record->event.pressed) {
                side_mode_a_control(1);
            }
            return false;

        case SIDE_MOD_B:
            if (record->event.pressed) {
                side_mode_b_control(1);
            }
            return false;

        case SIDE_HUI:
            if (record->event.pressed) {
                side_colour_control(1);
            }
            return false;

        case SIDE_SPI:
            if (record->event.pressed) {
                light_speed_control(1);
            }
            return false;

        case SIDE_SPD:
            if (record->event.pressed) {
                light_speed_control(0);
            }
            return false;

        case DEV_RESET:
            if (record->event.pressed) {
                f_dev_reset_press = 1;
                m_break_all_key();  
            } else {
                f_dev_reset_press = 0;
            }
            return false;

        case SLEEP_MODE:
            if (record->event.pressed) {
                if(f_dev_sleep_enable) f_dev_sleep_enable = false;
                else f_dev_sleep_enable = true;
                f_sleep_show       = 1;
                eeconfig_update_user_datablock(&user_config); 
            }
            return false;

        case BAT_SHOW:
            if (record->event.pressed) {
                f_bat_hold = !f_bat_hold;
            }
            return false;
        case RGB_VAI:
            if(low_bat_flag && (rgb_matrix_config.hsv.v == RGB_MATRIX_VAL_STEP)) 
                return false;
            return true;
        case RGB_TOG:
            if (record->event.pressed) {
                if(rgb_matrix_config.hsv.v)
                {
                    rgb_light_old = rgb_matrix_config.hsv.v;
                    rgb_matrix_config.hsv.v = 0;
                }
                else
                {
                    if(rgb_light_old) rgb_matrix_config.hsv.v = rgb_light_old;
                    else rgb_matrix_config.hsv.v = (255 - RGB_MATRIX_SPD_STEP * 2);
                }
                 
            }
            return false;
        
        default:
            return true;
    }
    return true;
}



/**
    @brief  timer process.
 */
void timer_pro(void)
{
    static uint32_t interval_timer = 0;
    static bool f_first            = true;

    if (f_first) {
        f_first        = false;
        interval_timer = timer_read32();
        m_host_driver  = host_get_driver();
    }

    if (timer_elapsed32(interval_timer) < 10) {
        return;
    } else if (timer_elapsed32(interval_timer) > 20) {
        interval_timer = timer_read32();
    } else {
        interval_timer += 10;  
    }

    if (rf_link_show_time < RF_LINK_SHOW_TIME)
        rf_link_show_time++;

    if (no_act_time < 0xffff)
        no_act_time++;

    if (rf_linking_time < 0xffff)
        rf_linking_time++;

}


/**
 * @brief  londing eeprom data.
 */
void m_londing_eeprom_data(void)
{
    eeconfig_read_user_datablock(&user_config);
    if (user_config.default_brightness_flag != 0xA5) {
        rgb_matrix_sethsv(RGB_DEFAULT_COLOUR, 255, RGB_MATRIX_MAXIMUM_BRIGHTNESS - RGB_MATRIX_VAL_STEP * 2); 
        user_config.default_brightness_flag = 0xA5;
        user_config.ee_side_mode_a           = side_mode_a;
        user_config.ee_side_mode_b           = side_mode_b;
        user_config.ee_side_light           = side_light;
        user_config.ee_side_speed           = side_speed;
        user_config.ee_side_rgb             = side_rgb;
        user_config.ee_side_colour          = side_colour;
        f_dev_sleep_enable                  = true;
        f_fn_lock                           = false;
        fn_features_magic_set(&user_config);
        eeconfig_update_user_datablock(&user_config); 
    } else {
        side_mode_a   = user_config.ee_side_mode_a;
        side_mode_b   = user_config.ee_side_mode_b;
        side_light  = user_config.ee_side_light;
        side_speed  = user_config.ee_side_speed;
        side_rgb    = user_config.ee_side_rgb;
        side_colour = user_config.ee_side_colour;

        if (!fn_features_magic_is_valid(&user_config)) {
            f_fn_lock = false;
            fn_features_magic_set(&user_config);
            eeconfig_update_user_datablock(&user_config);
        }
    }
}
/**
   qmk keyboard post init
 */
static void ind_rescan(void);

void keyboard_post_init_kb(void)
{
    m_gpio_init();             
    rf_uart_init();
    wait_ms(500);
    rf_device_init();
    m_break_all_key();
    m_londing_eeprom_data();
    fn_features_validate_configuration();
    ind_rescan();
    m_power_on_dial_sw_scan();
    rf_link_show_time = 0;
    keyboard_post_init_user();
}

/**
   rgb_matrix_indicators_user
 */

// Current hardware-mode base layer: Mac LAYER_MAC_BASE, Windows LAYER_WIN_BASE.
static uint8_t ind_base_layer_for_mode(void) {
    if (dev_info.sys_sw_state == SYS_SW_MAC) {
        return LAYER_MAC_BASE;
    }
    return LAYER_WIN_BASE;
}

// Cache the LED positions of keys mapped to KC_CAPS (green), TAP_HOLD_1
// (blue) and the Fn key (orange) in the current base layer, so the
// indicators follow VIA remaps instead of being hard-coded.
// keycode_at_keymap_location() reads the VIA EEPROM cache when available and
// the compile-time keymap otherwise, so both build targets behave the same.
static void ind_rescan(void) {
    uint8_t  base   = ind_base_layer_for_mode();
    uint16_t fn_kc  = (base == LAYER_MAC_BASE) ? MO(LAYER_MAC_FN) : MO(LAYER_WIN_FN);

    ind_caps_count     = 0;
    ind_tap_hold_count = 0;
    ind_fn_pos         = NO_LED;

    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        for (uint8_t col = 0; col < MATRIX_COLS; col++) {
            uint16_t kc  = keycode_at_keymap_location(base, row, col);
            uint8_t  led = g_led_config.matrix_co[row][col];
            if (led == NO_LED) {
                continue;
            }

            if (kc == KC_CAPS) {
                if (ind_caps_count < IND_MAX_POS) {
                    ind_caps_pos[ind_caps_count++] = led;
                }
            } else if (kc == TAP_HOLD_1) {
                if (ind_tap_hold_count < IND_MAX_POS) {
                    ind_tap_hold_pos[ind_tap_hold_count++] = led;
                }
            } else if (kc == fn_kc) {
                ind_fn_pos = led;
            }
        }
    }
}

bool rgb_matrix_indicators_kb(void)
{
    if(!rgb_matrix_indicators_user()){
        return false;
    }
    return true;
}
bool rgb_matrix_indicators_advanced_kb(uint8_t led_min, uint8_t led_max)
{
    if (keymap_config.no_gui) {
        rgb_matrix_set_color(89, 0x00, 0x80, 0x00);
    }

    if (host_keyboard_led_state().caps_lock) {
        // Tap/Hold keys first in blue, then KC_CAPS in green, so a key that
        // appears in both lists keeps the green Caps colour (Caps priority).
        for (uint8_t i = 0; i < ind_tap_hold_count; i++) {
            uint8_t led = ind_tap_hold_pos[i];
            if (led >= led_min && led < led_max) {
                rgb_matrix_set_color(led, IND_TAP_HOLD_R, IND_TAP_HOLD_G, IND_TAP_HOLD_B);
            }
        }
        for (uint8_t i = 0; i < ind_caps_count; i++) {
            uint8_t led = ind_caps_pos[i];
            if (led >= led_min && led < led_max) {
                rgb_matrix_set_color(led, IND_CAPS_LOCK_R, IND_CAPS_LOCK_G, IND_CAPS_LOCK_B);
            }
        }
    }

    if (f_fn_lock && ind_fn_pos != NO_LED && ind_fn_pos >= led_min && ind_fn_pos < led_max) {
        rgb_matrix_set_color(ind_fn_pos, IND_FN_LOCK_R, IND_FN_LOCK_G, IND_FN_LOCK_B);
    }

    return true;
}
/**
   housekeeping_task_kb
 */
void housekeeping_task_kb(void)
{

    if (fn_features_validate_pending) {
        fn_features_validate_pending = false;
        fn_features_validate_configuration();
    }

    if (tap_hold_active && !tap_hold_hold_sent && timer_elapsed(tap_hold_timer) >= TAP_HOLD_TERM) {
        tap_hold_hold_sent = true;
#ifdef VIA_ENABLE
        dynamic_keymap_macro_send(1);
#else
        tap_code(KC_CAPS);
#endif
    }

    if (tap_hold_active_2 && !tap_hold_hold_sent_2 && timer_elapsed(tap_hold_timer_2) >= TAP_HOLD_TERM) {
        tap_hold_hold_sent_2 = true;
#ifdef VIA_ENABLE
        dynamic_keymap_macro_send(4);
#else
        tap_code(KC_CAPS);
#endif
    }

    // Refresh indicator positions at most every 500 ms; this follows VIA
    // keymap writes and Mac/Windows hardware-mode switches without scanning
    // the keymap on every RGB frame.
    if (timer_elapsed32(ind_rescan_timer) >= 500) {
        ind_rescan_timer = timer_read32();
        ind_rescan();
    }

    timer_pro();

    uart_receive_pro();

    uart_send_report_func();

    dev_sts_sync();

    long_press_key();

    dial_sw_scan();

    m_side_led_show();

    Sleep_Handle();

}
