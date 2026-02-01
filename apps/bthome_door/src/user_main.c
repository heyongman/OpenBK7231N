#include <string.h>
#include <stdint.h>
#include "tuya_os_init.h"
#include "tuya_hal_bt.h"
#include "tuya_gpio.h"
#include "tuya_adc.h"
#include "tuya_error_code.h"
#include "rtos_pub.h"
#include "wlan_ui_pub.h"
#include "manual_ps_pub.h"

const char *CFG_GetOpenBekenHostName(void)
{
#ifdef APP_BIN_NAME
    return APP_BIN_NAME;
#else
    return "bthome_door";
#endif
}

// -----------------------------
// User config
// -----------------------------
#define HALL_PIN                TY_GPIOA_16
#define LED_PIN                 TY_GPIOA_26
#define LED_ACTIVE_HIGH         1

// Hall sensor logic: 1 = door open when pin is high, 0 = door open when pin is low
#define DOOR_OPEN_WHEN_HIGH     1

// Hall sensor pull mode
#define HALL_PULL_MODE          TY_GPIO_PULLUP

// Advertise for a short window after wakeup
#define ADV_DURATION_MS         500
#define ADV_READY_TIMEOUT_MS    2000

// Battery via ADC (optional)
#define BATT_ADC_ENABLE         1
#define BATT_ADC_CH             1   // ADC channel index (0..3). Set to match your HW.
#define BATT_DIVIDER_NUM        2   // Battery voltage divider ratio numerator
#define BATT_DIVIDER_DEN        1   // Battery voltage divider ratio denominator
#define BATT_MIN_MV             2000
#define BATT_MAX_MV             3000

// BTHome v2 constants
#define BTHOME_UUID16_L         0xD2
#define BTHOME_UUID16_H         0xFC
#define BTHOME_INFO_V2          0x40
#define BTHOME_INFO_TRIGGER     0x04
#define BTHOME_OBJ_DOOR         0x1A
#define BTHOME_OBJ_BATTERY      0x01

static uint8_t g_adv_data[31];
static uint8_t g_scan_rsp_data[1];
static tuya_ble_data_buf_t g_adv_buf;
static tuya_ble_data_buf_t g_scan_buf;
static volatile BOOL_T g_adv_ready = FALSE;

static void led_set(BOOL_T on)
{
    if (LED_ACTIVE_HIGH) {
        tuya_gpio_write(LED_PIN, on ? TRUE : FALSE);
    } else {
        tuya_gpio_write(LED_PIN, on ? FALSE : TRUE);
    }
}

static BOOL_T read_door_open(void)
{
    INT_T val = tuya_gpio_read(HALL_PIN);
    if (DOOR_OPEN_WHEN_HIGH) {
        return (val != 0) ? TRUE : FALSE;
    }
    return (val == 0) ? TRUE : FALSE;
}

#if BATT_ADC_ENABLE
static uint8_t read_battery_percent(void)
{
    tuya_adc_t adc;
    tuya_adc_cfg_t cfg;
    uint16_t raw = 0;
    uint16_t mv = 0;
    uint32_t batt_mv;

    memset(&adc, 0, sizeof(adc));
    memset(&cfg, 0, sizeof(cfg));

    TUYA_ADC_CFG(&adc, BATT_ADC_CH, 0);
    if (tuya_adc_init(&adc) != OPRT_OK) {
        return 0xFF;
    }

    if (tuya_adc_voltage(&adc, &raw, &mv, 1) != OPRT_OK) {
        tuya_adc_deinit(&adc);
        return 0xFF;
    }

    tuya_adc_deinit(&adc);

    batt_mv = (uint32_t)mv * BATT_DIVIDER_NUM / BATT_DIVIDER_DEN;
    if (batt_mv <= BATT_MIN_MV) {
        return 0;
    }
    if (batt_mv >= BATT_MAX_MV) {
        return 100;
    }
    return (uint8_t)((batt_mv - BATT_MIN_MV) * 100 / (BATT_MAX_MV - BATT_MIN_MV));
}
#endif

static void build_bthome_adv(BOOL_T door_open)
{
    uint8_t *p = g_adv_data;
    uint8_t *len_ptr;
    uint8_t info = (uint8_t)(BTHOME_INFO_V2 | BTHOME_INFO_TRIGGER);

    // Flags: LE General Discoverable, BR/EDR not supported
    *p++ = 0x02;
    *p++ = 0x01;
    *p++ = 0x06;

    // Service Data - 16-bit UUID (BTHome)
    len_ptr = p++;
    *p++ = 0x16;
    *p++ = BTHOME_UUID16_L;
    *p++ = BTHOME_UUID16_H;
    *p++ = info;
#if BATT_ADC_ENABLE
    {
        uint8_t batt = read_battery_percent();
        if (batt != 0xFF) {
            *p++ = BTHOME_OBJ_BATTERY;
            *p++ = batt;
        }
    }
#endif
    *p++ = BTHOME_OBJ_DOOR;
    *p++ = door_open ? 1 : 0;

    *len_ptr = (uint8_t)(p - len_ptr - 1);

    g_adv_buf.data = g_adv_data;
    g_adv_buf.len = (unsigned int)(p - g_adv_data);

    g_scan_buf.data = g_scan_rsp_data;
    g_scan_buf.len = 0;
}

static void bt_msg_cb(INT_T id, ty_bt_cb_event_t e, tuya_ble_data_buf_t *buf)
{
    (void)id;
    (void)buf;

    if (e == TY_BT_EVENT_ADV_READY) {
        tuya_hal_bt_reset_adv(&g_adv_buf, &g_scan_buf);
        g_adv_ready = TRUE;
    }
}

static void bt_init_and_wait(void)
{
    ty_bt_param_t param;

    memset(&param, 0, sizeof(param));
    strncpy(param.name, "BTHomeDoor", sizeof(param.name) - 1);
    param.mode = TY_BT_MODE_PERIPHERAL;
    param.link_num = 1;
    param.cb = bt_msg_cb;
    param.adv = g_adv_buf;
    param.scan_rsp = g_scan_buf;

    tuya_hal_bt_port_init(&param);

    // wait for ADV_READY
    {
        uint32_t waited = 0;
        while (!g_adv_ready && waited < ADV_READY_TIMEOUT_MS) {
            rtos_delay_milliseconds(10);
            waited += 10;
        }
    }
}

static void enter_deep_sleep_next_edge(BOOL_T current_level)
{
    PS_DEEP_CTRL_PARAM deep_param;
    uint32_t idx = (uint32_t)HALL_PIN;
    BOOL_T wake_on_low = current_level ? TRUE : FALSE; // wake on opposite level

    memset(&deep_param, 0, sizeof(deep_param));
    deep_param.wake_up_way = PS_DEEP_WAKEUP_GPIO;

    if (idx < 32) {
        deep_param.gpio_index_map = (1UL << idx);
        if (wake_on_low) {
            deep_param.gpio_edge_map = (1UL << idx); // 1: low level wakeup
        }
    } else {
        uint32_t bit = idx - 32;
        deep_param.gpio_last_index_map = (1UL << bit);
        if (wake_on_low) {
            deep_param.gpio_last_edge_map = (1UL << bit); // 1: low level wakeup
        }
    }

    bk_enter_deep_sleep_mode(&deep_param);
}

void user_main(void)
{
    BOOL_T door_open;

    tuya_os_init();

    // GPIO setup
    tuya_gpio_inout_set(HALL_PIN, TRUE);
    // tuya_gpio_mode_set(HALL_PIN, HALL_PULL_MODE);

    tuya_gpio_inout_set(LED_PIN, FALSE);
    // led_set(FALSE);

    // Read door state and build BTHome advertisement
    door_open = read_door_open();
    led_set(TRUE);
    build_bthome_adv(door_open);

    bt_init_and_wait();

    // Keep advertising briefly, then sleep
    rtos_delay_milliseconds(ADV_DURATION_MS);
    tuya_hal_bt_stop_adv();

    led_set(FALSE);

    // Enter deep sleep and wake on next edge of the hall sensor
    enter_deep_sleep_next_edge((BOOL_T)tuya_gpio_read(HALL_PIN));
}
