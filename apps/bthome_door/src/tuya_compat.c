#include <string.h>
#include "tuya_error_code.h"
#include "tuya_os_adapter.h"
#include "tuya_hal_bt.h"
#include "tuya_gpio.h"
#include "tuya_adc.h"
#include "tuya_pin.h"
#include "tuya_driver.h"

#include "gpio_pub.h"
#include "saradc_pub.h"

// -----------------------------------------------------------------------------
// Minimal Tuya core stubs/compat to satisfy SDK link for this project
// -----------------------------------------------------------------------------

OPERATE_RET tuya_os_adapt_reg_intf(INTF_TYPE_ENUM type, void *intf_ptr)
{
    (void)type;
    (void)intf_ptr;
    return OPRT_OK;
}

#define TUYA_COMPAT_MAX_DRIVERS 32
static struct {
    uint8_t type;
    uint8_t port;
    void *node;
} s_drv_table[TUYA_COMPAT_MAX_DRIVERS];

void *tuya_driver_find(uint8_t type, uint8_t port)
{
    for (int i = 0; i < TUYA_COMPAT_MAX_DRIVERS; i++) {
        if (s_drv_table[i].node && s_drv_table[i].type == type && s_drv_table[i].port == port) {
            return s_drv_table[i].node;
        }
    }
    return NULL;
}

int tuya_driver_register(tuya_drv_node_t *node, uint8_t type, uint8_t port)
{
    for (int i = 0; i < TUYA_COMPAT_MAX_DRIVERS; i++) {
        if (s_drv_table[i].node == NULL) {
            s_drv_table[i].node = node;
            s_drv_table[i].type = type;
            s_drv_table[i].port = port;
            return OPRT_OK;
        }
    }
    return OPRT_COM_ERROR;
}

// -----------------------------------------------------------------------------
// Tuya HAL BT wrappers -> OS adapter BT
// -----------------------------------------------------------------------------
extern int tuya_os_adapt_bt_port_init(ty_bt_param_t *p);
extern int tuya_os_adapt_bt_port_deinit(void);
extern int tuya_os_adapt_bt_gap_disconnect(void);
extern int tuya_os_adapt_bt_send(unsigned char *data, unsigned char len);
extern int tuya_os_adapt_bt_reset_adv(tuya_ble_data_buf_t *adv, tuya_ble_data_buf_t *scan_resp);
extern int tuya_os_adapt_bt_get_rssi(signed char *rssi);
extern int tuya_os_adapt_bt_start_adv(void);
extern int tuya_os_adapt_bt_stop_adv(void);

OPERATE_RET tuya_hal_bt_port_init(ty_bt_param_t *p)
{
    return tuya_os_adapt_bt_port_init(p);
}

OPERATE_RET tuya_hal_bt_port_deinit(void)
{
    return tuya_os_adapt_bt_port_deinit();
}

OPERATE_RET tuya_hal_bt_gap_disconnect(void)
{
    return tuya_os_adapt_bt_gap_disconnect();
}

OPERATE_RET tuya_hal_bt_send(BYTE_T *data, UINT8_T len)
{
    return tuya_os_adapt_bt_send(data, len);
}

OPERATE_RET tuya_hal_bt_reset_adv(tuya_ble_data_buf_t *adv, tuya_ble_data_buf_t *scan_resp)
{
    return tuya_os_adapt_bt_reset_adv(adv, scan_resp);
}

OPERATE_RET tuya_hal_bt_get_rssi(SCHAR_T *rssi)
{
    return tuya_os_adapt_bt_get_rssi(rssi);
}

OPERATE_RET tuya_hal_bt_start_adv(void)
{
    return tuya_os_adapt_bt_start_adv();
}

OPERATE_RET tuya_hal_bt_stop_adv(void)
{
    return tuya_os_adapt_bt_stop_adv();
}

// -----------------------------------------------------------------------------
// Tuya GPIO minimal implementation using BK GPIO
// -----------------------------------------------------------------------------
static inline UINT32 tuya_gpio_to_bk(TY_GPIO_PORT_E port)
{
    return (UINT32)port;
}

OPERATE_RET tuya_gpio_inout_set(IN CONST TY_GPIO_PORT_E port, IN CONST BOOL_T in)
{
    UINT32 gpio = tuya_gpio_to_bk(port);
    if (in) {
        bk_gpio_config_input(gpio);
    } else {
        bk_gpio_config_output(gpio);
        bk_gpio_output(gpio, 0);
    }
    return OPRT_OK;
}

OPERATE_RET tuya_gpio_inout_set_select(IN CONST TY_GPIO_PORT_E port, IN CONST BOOL_T in, IN CONST BOOL_T high)
{
    UINT32 gpio = tuya_gpio_to_bk(port);
    if (in) {
        bk_gpio_config_input(gpio);
    } else {
        bk_gpio_config_output(gpio);
        bk_gpio_output(gpio, high ? 1 : 0);
    }
    return OPRT_OK;
}

OPERATE_RET tuya_gpio_mode_set(IN CONST TY_GPIO_PORT_E port, IN CONST TY_GPIO_MODE_E mode)
{
    UINT32 gpio = tuya_gpio_to_bk(port);

    switch (mode) {
    case TY_GPIO_PULLUP:
        bk_gpio_config_input_pup(gpio);
        break;
    case TY_GPIO_PULLDOWN:
        bk_gpio_config_input_pdwn(gpio);
        break;
    case TY_GPIO_FLOATING:
    default:
        bk_gpio_config_input(gpio);
        break;
    }
    return OPRT_OK;
}

INT_T tuya_gpio_read(IN CONST TY_GPIO_PORT_E port)
{
    UINT32 gpio = tuya_gpio_to_bk(port);
    return (INT_T)bk_gpio_input(gpio);
}

OPERATE_RET tuya_gpio_write(IN CONST TY_GPIO_PORT_E port, IN CONST BOOL_T high)
{
    UINT32 gpio = tuya_gpio_to_bk(port);
    bk_gpio_output(gpio, high ? 1 : 0);
    return OPRT_OK;
}

OPERATE_RET tuya_gpio_irq_init(IN CONST TY_GPIO_PORT_E port, IN CONST TY_GPIO_IRQ_CB gpio_irq_cb,
                              IN CONST TY_GPIO_IRQ_TRIG_E trig_type, UINT_T id)
{
    (void)port;
    (void)gpio_irq_cb;
    (void)trig_type;
    (void)id;
    // Not used in this project; implement as no-op
    return OPRT_OK;
}

// -----------------------------------------------------------------------------
// Tuya PIN minimal implementation (optional)
// -----------------------------------------------------------------------------
static tuya_pin_ops_t *s_pin_ops = NULL;

int tuya_pin_register(tuya_pin_ops_t *ops)
{
    s_pin_ops = ops;
    return OPRT_OK;
}

int tuya_pin_init(tuya_pin_name_t pin, tuya_pin_mode_t mode)
{
    if (s_pin_ops && s_pin_ops->init) {
        return s_pin_ops->init(pin, mode);
    }
    return OPRT_OK;
}

int tuya_pin_write(tuya_pin_name_t pin, tuya_pin_level_t level)
{
    if (s_pin_ops && s_pin_ops->write) {
        return s_pin_ops->write(pin, level);
    }
    return OPRT_OK;
}

int tuya_pin_read(tuya_pin_name_t pin)
{
    if (s_pin_ops && s_pin_ops->read) {
        return s_pin_ops->read(pin);
    }
    return 0;
}

int tuya_pin_control(tuya_pin_name_t pin, uint8_t cmd, void *arg)
{
    if (s_pin_ops && s_pin_ops->control) {
        return s_pin_ops->control(pin, cmd, arg);
    }
    return OPRT_OK;
}

int tuya_pin_irq_init(tuya_pin_name_t pin, tuya_pin_mode_t irq_mode, tuya_pin_irq_cb cb, void *arg)
{
    (void)pin;
    (void)irq_mode;
    (void)cb;
    (void)arg;
    return OPRT_OK;
}

int tuya_pin_irq_enable(tuya_pin_name_t pin)
{
    (void)pin;
    return OPRT_OK;
}

int tuya_pin_irq_disable(tuya_pin_name_t pin)
{
    (void)pin;
    return OPRT_OK;
}

// -----------------------------------------------------------------------------
// Tuya ADC minimal implementation using tuya_os_adapter driver if available
// -----------------------------------------------------------------------------
int tuya_adc_init(tuya_adc_t *adc)
{
    if (adc == NULL) {
        return OPRT_COM_ERROR;
    }
    if (adc->ops == NULL) {
        tuya_adc_t *dev = (tuya_adc_t *)tuya_driver_find(TUYA_DRV_ADC, adc->cfg.pin);
        if (dev && dev->ops) {
            adc->ops = dev->ops;
        }
    }
    if (adc->ops && adc->ops->init) {
        return adc->ops->init(adc, &adc->cfg);
    }
    return OPRT_OK;
}

int tuya_adc_control(tuya_adc_t *adc, uint8_t cmd, void *arg)
{
    if (adc && adc->ops && adc->ops->control) {
        return adc->ops->control(adc, cmd, arg);
    }
    return OPRT_OK;
}

int tuya_adc_deinit(tuya_adc_t *adc)
{
    if (adc && adc->ops && adc->ops->deinit) {
        return adc->ops->deinit(adc);
    }
    return OPRT_OK;
}

int tuya_adc_convert(tuya_adc_t *adc, uint16_t *data, uint16_t num)
{
    if (adc && adc->ops && adc->ops->convert && data && num > 0) {
        return adc->ops->convert(adc, data);
    }
    return OPRT_OK;
}

int tuya_adc_voltage(tuya_adc_t *adc, uint16_t *data, uint16_t *voltage, uint16_t count)
{
    if (adc && adc->ops && adc->ops->control && voltage) {
        return adc->ops->control(adc, TUYA_ADC_VOLTAGE_CMD, voltage);
    }
    return OPRT_OK;
}
