#include "PUBLIC\Std_Types.h"
#include "Protocol\Proto_ComDef.h"
#include "FACTORY\Factory.h"
#include "ble_types.h"
#include "ble_dahao.h"

uint8 Factory_State = 0;
uint8 Factory_Sub = 0;
uint8 Factory_Time64ms;

extern uint16_t m_conn_handle;

/****************************************************************************************************
**Function:
   void Factory_Entern_Sleep(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Factory_Entern_Sleep(void)
{
    switch(Factory_Sub)
    {
        case CASE_IN:
        {
            if(m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                Factory_Sub = CASE_01;
                Factory_Time64ms = 1000/32;
            }
            break;
        }
        case CASE_01:
        {
                Factory_Sub = CASE_END;
                ble_dahao_stop_advert();
                m_ble_adv_status = BLE_ADV_STOP;
                Factory_State = COMD_FATORY_IDLE;
            break;
        }
        default:
        {
            Factory_Sub = CASE_END;
            break;
        }
    }
}

/****************************************************************************************************
**Function:
   void Factory_Proc(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void Factory_Proc(void)
{
    if(Factory_Time64ms != 0)
    {
        Factory_Time64ms--;
        return;
    }
    switch(Factory_State)
    {
        case COMD_FATORY_IDLE:
        {

            break;
        }
        case COMD_FATORY_ENTERN_SLEEP:
        {
            Factory_Entern_Sleep();
            break;
        }
        default:
        {
            break;
        }
    }
}


