
#ifndef BLE_DAHAO_H
#define BLE_DAHAO_H

#include <string.h>
#include <stdint.h>
#include "PUBLIC\Std_Types.h"
#include "ble.h"
#include "ble_conn_params.h"
#include "ble_types.h"

#define BLE_UUID_DAHAO_SERVICE                                         0xFEE7
#define BLE_UUID_DAHAO_READ_WRITE_CHARACTERISTICS         0xFEC6

#define BLE_FRAME_MAX_DATA_LEN            (GATT_MTU_SIZE_DEFAULT - 3)                             /**<Set the Maximum Data length og WECHAT to the (GATT Maximum Transmission Unit -3) */

#define                                    COMPANY_IDENTIFIER                        0x0056
#define BLE_MANUF_DATA_LEN    11
enum
{
    BLE_ADV_STOP,
    BLE_ADV_START_FAST,
    BLE_ADV_FAST,
    BLE_ADV_SLOW,
    BLE_ADV_START_STOP
};

typedef enum
{
    CONNECTED,
    INDICATION_SENT,
    RESPONSE_SUCCESS,
    DISCONNECTED,
}IndicationStatus;

typedef struct
{
        uint16_t                   service_handle;
        ble_gatts_char_handles_t   read_write_handles;
        uint16_t                   conn_handle; 
} ble_dahao_t;

typedef struct
{
        uint8_t *data;
        uint16_t len;
        uint16_t offset;
} data_info;


extern ble_dahao_t      m_ble_dahao;
extern uint8_t m_addl_adv_data[BLE_FRAME_MAX_DATA_LEN];
extern uint8_t m_mac_add[6];
extern uint8_t m_addl_adv_manuf_data[BLE_MANUF_DATA_LEN];
extern uint16_t m_disconnect_timer;
extern uint8_t m_ble_adv_status;
extern uint8_t m_ble_adv_timer;
extern uint16_t m_conn_handle;
extern uint8_t m_ble_adv_restart_timer;

#define BLE_UPDATA_DISCONNECT_TIMER(x)   m_disconnect_timer = x
#define IS_CONNECTED()                       (m_conn_handle != BLE_CONN_HANDLE_INVALID) 

extern uint32_t ble_dahao_notify_data(ble_dahao_t *p_wcs, uint8_t *data, uint16_t len);
extern void ble_dahao_on_ble_evt(ble_dahao_t *p_wcs,ble_evt_t * p_ble_evt);
extern uint32_t ble_dahao_add_service(ble_dahao_t *p_dahaoBle);
extern uint32_t ble_dahao_add_characteristics(ble_dahao_t *p_dahaoBle);
extern void ble_dahao_get_scan_data(void);

extern void ble_dahao_timer_proc(void);
extern void ble_dahao_start_advert(uint8 mode);
extern uint8 ble_dahao_stop_advert(void);

#endif // #ifdef BLE_DAHAO_H

