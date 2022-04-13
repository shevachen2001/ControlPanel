/*****************************************************************************
|  File Name: NrfFlash.c
|
|  Description: 
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
| Copyright (c) 2002-2012 by DAHAO TECHNOLOGY Co., Ltd. All rights reserved.
|
|               This software is copyright protected and proprietary 
|               to DAHAO TECHNOLOGY Co., Ltd. DAHAO TECHNOLOGY Co., Ltd 
|               grants to you only those rights as set out in the 
|               license conditions. All other rights remain with 
|               DAHAO TECHNOLOGY Co., Ltd.
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     -------------------------------------
| LXW          LXW               DAHAO TECHNOLOGY Co., Ltd
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2013-10 1.00    lxw     Initial version 
|****************************************************************************/

/****************************************************************************/ 
/* Include file                                                             */                                                 
/****************************************************************************/

#include "Flash\Flash_Nrf.h"
#include "pstorage.h"
#include "Access\LockKey.h"
#include "SYSTEM\sys.h"
#include "crc32.h"

typedef struct {
    void (*CallbackHandler)(pstorage_handle_t * p_handle,
                                      uint8_t             op_code,
                                      uint32_t            result,
                                      uint8_t           * p_data,
                                      uint32_t            data_len);
}Flash_CallbackHandlerType;

static pstorage_handle_t      m_nrf_storage_handle[NRF_FLASH_END];  
pstorage_handle_t block_id ;

/*static uint8 nrf_write_flag[];
static Std_Bit8Flag nrf_write_flag = 0;

#define NRF_FLAG_WRITE_OK   nrf_write_flag.bit7*/
static uint8 nrf_write_time = 0;
static uint8 nrf_flag_write_wait = 0;
static uint8 nrf_flag_write_index = NRF_FLASH_END;
static uint8 nrf_write_flag[NRF_FLASH_END];


static void pstorage_nrf_callback_handler(pstorage_handle_t * p_handle,
                                      uint8_t             op_code,
                                      uint32_t            result,
                                      uint8_t           * p_data,
                                      uint32_t            data_len)
{
    // If we are in BOOTLOADER_SETTINGS_SAVING state and we receive an PSTORAGE_STORE_OP_CODE
    // response then settings has been saved and update has completed.
    switch(op_code)
    {      
        case PSTORAGE_UPDATE_OP_CODE:
        {
            if (result == NRF_SUCCESS)
            {
                nrf_flag_write_wait = 1;
                if(nrf_flag_write_index < NRF_FLASH_END)
                {
                    nrf_write_flag[nrf_flag_write_index] = 0;
                }
            }
            else
            {
                // Update operation failed.
                nrf_write_time = 2000/32;
                nrf_flag_write_wait = 2;
            }
            break;
        }
    }

    APP_ERROR_CHECK(result);
}

/****************************************************************************************************
**Function:
   void NrfFlash_Init(void)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void NrfFlash_Init(void)
{
    NrfFlash_PstorageType Message;

    for(uint8 i=0; i<NRF_FLASH_END; i++)
    {
        Message.Sector =i;
        Message.pData= NULL;
        Message.Add = 0;
        Message.Lenth = 0;
        NrfFlashDrive.open(&Message);
    }
}

/****************************************************************************************************
**Function:
   void NrfFlash_Open(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
void NrfFlash_Open(void *pData)
{
    pstorage_module_param_t module_param;
    uint8 sector = ((NrfFlash_PstorageType *)pData)->Sector;
    
    module_param.block_count=1;
    module_param.block_size=1024;//块的大小为8
    module_param.cb=pstorage_nrf_callback_handler;//CallBackHandlerTable[sector].CallbackHandler;
    //err_code =pstorage_init();//初始化
    (void)pstorage_register(&module_param,&m_nrf_storage_handle[sector]);
    nrf_write_flag[sector] = 0;
}

/****************************************************************************************************
**Function:
   Std_ReturnType NrfFlash_Read(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType NrfFlash_Read(void *pData)
{
    Std_ReturnType Err;
    if(((NrfFlash_PstorageType *)pData)->pData == NULL)
    {
        
        Err = E_INPAR_ERR;
    }
    else 
    {
        uint32_t err_code;
        pstorage_handle_t dest_block_id;
        
        uint8 sector = ((NrfFlash_PstorageType *)pData)->Sector;
                    
        err_code = pstorage_block_identifier_get(&m_nrf_storage_handle[sector],0,&dest_block_id );
        APP_ERROR_CHECK(err_code);
        err_code = pstorage_load(((NrfFlash_PstorageType *)pData)->pData, &dest_block_id,((NrfFlash_PstorageType *)pData)->Lenth,0);
        if(err_code == NRF_SUCCESS)
        {
            Err = E_OK;
        }
        else
        {
            Err = E_NOT_OK;
        }
    }
    return Err;
}

/****************************************************************************************************
**Function:
   Std_ReturnType NrfFlash_Write(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType NrfFlash_Write(void *pData)
{
    Std_ReturnType Err;
    if(((NrfFlash_PstorageType *)pData)->pData == NULL)
    {
        
        Err = E_INPAR_ERR;
    }
    else 
    {
        uint32_t err_code;
        pstorage_handle_t dest_block_id;
        uint8 timeout = 200;
        uint16 length = ((NrfFlash_PstorageType *)pData)->Lenth;
        
        uint8 sector = ((NrfFlash_PstorageType *)pData)->Sector;

        err_code = pstorage_block_identifier_get(&m_nrf_storage_handle[sector],0,&dest_block_id );
        APP_ERROR_CHECK(err_code);

        if(length > 1024)
        {
            length = 1024;
        }
        nrf_flag_write_wait = 0;
        err_code = pstorage_update(&dest_block_id,((NrfFlash_PstorageType *)pData)->pData, length, 0);
        while(timeout--)
        {
            if(nrf_flag_write_wait > 0)
            {
                break;
            }
            Sys_GenFunDelayUs(100);
        }
        if(err_code == NRF_SUCCESS)
        {
            if(nrf_flag_write_wait == 1)
            {
                Err = E_OK;
            }
            else
            {
                Err = E_NOT_OK;
            }
        }
        else
        {
            Err = E_NOT_OK;
        }
    }
    return Err;
}

/****************************************************************************************************
**Function:
   Std_ReturnType NrfFlash_Write(void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType NrfFlash_Store(uint8 item)
{
    Std_ReturnType ret = E_OK;
    NrfFlash_PstorageType Message;

    nrf_flag_write_index = item;
    switch(item)
    {
        case NRF_FLASH_SYS_PRAR:
        {
            Message.Sector = NRF_FLASH_SYS_PRAR;
            Message.pData= (uint8_t *)&Sys_PataCfg;
            Message.Add = 0;
            Message.Lenth = sizeof(Sys_PataCfgType);
            Sys_PataCfg.KeyWord = 0;
            Sys_PataCfg.KeyWord = crc32_compute((uint8_t *)&Sys_PataCfg, sizeof(Sys_PataCfgType), NULL);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
        case NRF_FLASH_SYS_PRARBK:
        {
            Message.Sector = NRF_FLASH_SYS_PRARBK;
            Message.pData= (uint8_t *)&Sys_PataCfg;
            Message.Add = 0;
            Message.Lenth = sizeof(Sys_PataCfgType);
            Sys_PataCfg.KeyWord = 0;
            Sys_PataCfg.KeyWord = crc32_compute((uint8_t *)&Sys_PataCfg, sizeof(Sys_PataCfgType), NULL);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
        case NRF_FLASH_BLACK_LIST:
        {
            Message.Sector = NRF_FLASH_BLACK_LIST;
            Message.pData= (uint8_t *)&LockKeyBlackList;
            Message.Add = 0;
            Message.Lenth = sizeof(LockKey_BlackListType);
            LockKeyBlackList.KeyWord = 0;
            LockKeyBlackList.KeyWord = crc32_compute((uint8_t *)&LockKeyBlackList, sizeof(LockKey_BlackListType), NULL);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
        case NRF_FLASH_BLACK_LISTBK:
        {
            Message.Sector = NRF_FLASH_BLACK_LISTBK;
            Message.pData= (uint8_t *)&LockKeyBlackList;
            Message.Add = 0;
            Message.Lenth = sizeof(LockKey_BlackListType);
            LockKeyBlackList.KeyWord = 0;
            LockKeyBlackList.KeyWord = crc32_compute((uint8_t *)&LockKeyBlackList, sizeof(LockKey_BlackListType), NULL);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
        case NRF_FLASH_SERVE_KEY:
        {
            Message.Sector = NRF_FLASH_SERVE_KEY;
            Message.pData= (uint8_t *)&LockKeyServeKeyList;
            Message.Add = 0;
            Message.Lenth = sizeof(LockKey_ServeKeyListType);
            LockKeyServeKeyList.KeyWord = 0;
            LockKeyServeKeyList.KeyWord = crc32_compute((uint8_t *)&LockKeyServeKeyList, sizeof(LockKey_ServeKeyListType), NULL);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
        case NRF_FLASH_SERVE_KEYBK:
        {
            Message.Sector = NRF_FLASH_SERVE_KEYBK;
            Message.pData= (uint8_t *)&LockKeyServeKeyList;
            Message.Add = 0;
            Message.Lenth = sizeof(LockKey_ServeKeyListType);
            LockKeyServeKeyList.KeyWord = 0;
            LockKeyServeKeyList.KeyWord = crc32_compute((uint8_t *)&LockKeyServeKeyList, sizeof(LockKey_ServeKeyListType), NULL);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
    #if 0
        case NRF_FLASH_APP_KEY:
        {
            Message.Sector = NRF_FLASH_APP_KEY;
            Message.pData= (uint8_t *)&LockKeyAppKeyList;
            Message.Add = 0;
            Message.Lenth = sizeof(LockKey_AppKeyListType);
            LockKeyAppKeyList.KeyWord = 0;
            LockKeyAppKeyList.KeyWord = crc32_compute((uint8_t *)&LockKeyAppKeyList, sizeof(LockKey_AppKeyListType), NULL);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
        case NRF_FLASH_APP_KEYBK:
        {
            Message.Sector = NRF_FLASH_APP_KEYBK;
            Message.pData= (uint8_t *)&LockKeyAppKeyList;
            Message.Add = 0;
            Message.Lenth = sizeof(LockKey_AppKeyListType);
            LockKeyAppKeyList.KeyWord = 0;
            LockKeyAppKeyList.KeyWord = crc32_compute((uint8_t *)&LockKeyAppKeyList, sizeof(LockKey_AppKeyListType), NULL);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
    #endif
        case NRF_FLASH_RECORD:
        {
            Message.Sector = NRF_FLASH_RECORD;
            Message.pData= (uint8_t *)&AccRcord;
            Message.Add = 0;
            Message.Lenth = sizeof(AccRcordParaType);
            ret = NrfFlashDrive.write(&Message);
            return ret;
        }
        default:
            break;
    }
    return E_NOT_OK;
}


/****************************************************************************************************
**Function:
   Std_ReturnType NrfFlash_IoCtl(Key_CmdType Cmd,void *pData)
**Author: lxw
**Description:
**Input: 
**Output: 
****************************************************************************************************/
Std_ReturnType NrfFlash_IoCtl(uint8 Cmd,void *pData)
{
    if(nrf_write_time != 0)
    {
        nrf_write_time--;
    }
    if(Cmd >= NRFFLASH_CMD_MAX)
    {
        return E_INPAR_ERR;
    }
    else 
    {
        switch(Cmd)
        {
            case NRFFLASH_SCAN:
            {
                if((nrf_flag_write_wait == 1) || (nrf_write_time != 0))
                {
                    return E_OK;
                }
                
                for(uint8 i=0; i<NRF_FLASH_END; i++)
                {
                    if(nrf_write_flag[i] == 1)
                    {
                        NrfFlash_Store(i);
                        nrf_write_time = 1600/32;
                        break;
                    }
                }
                break;
            }
            case NRFFLASH_STORE:
            {
                 uint8 sector = ((NrfFlash_PstorageType *)pData)->Sector;
                 nrf_write_flag[sector] = 1;
                 nrf_write_time = 2000/32;
                 break;
            }
            case NRFFLASH_STORE_DIR:
            {
                 uint8 sector = ((NrfFlash_PstorageType *)pData)->Sector;
                NrfFlash_Store(sector);
                 break;
            }
            default: break;
         }
    }
      return E_OK;
}


const Dr_OpertonType NrfFlashDrive =
{
    NrfFlash_Open,NULL, NrfFlash_Read, NrfFlash_Write, NrfFlash_IoCtl
};

#if (NRF_FLASH_CHECK == STD_TRUE)
/*******************************************************************************
* Function Name  : 
void Can_Check(void)
* Description    : 
*                  reset values.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void NrfFlash_Check(void)
{
    uint8 count;
    
    uint8 Databuffer[1024];

    NrfFlash_PstorageType Message;
    Message.Sector = NRF_FLASH_BLACK_LIST;
    Message.pData= Databuffer;
    Message.Add = 0;
    Message.Lenth = 100;
    //NrfFlashDrive.open(&Message);
    for(uint8 i=0 ;i<10;i++)
    {
        Databuffer[i] = 1;
    }
    for(;;)
    {
//        Num = NrfFlash_SpiReadID();
//        NrfFlashDrive.ioctl(FLASH_ERASE_SECT,&(Message.Add ));    
        /*Message.Sector = NRF_FLASH_BLACK_LIST;
        NrfFlashDrive.write(&Message);*/
        BlackList_LoadPara();
        //Sys_GenFunDelayMs(1000);
        //BlackList_StorePara1();
        while(nrf_flag_write_wait == 0)
        {
            Sys_GenFunDelayMs(10);
        }
        
        #if 0
        memset(Databuffer,0,10);
        NrfFlashDrive.read(&Message);
        Sys_GenFunDelayMs(1000);

        Message.Sector = NRF_FLASH_SERVE_KEY;
        NrfFlashDrive.write(&Message);
        while(nrf_flag_write_wait == 0)
        {
            Sys_GenFunDelayMs(10);
        }
        memset(Databuffer,0,10);
        NrfFlashDrive.read(&Message);
        Sys_GenFunDelayMs(1000);

        Message.Sector = NRF_FLASH_APP_KEY;
        NrfFlashDrive.write(&Message);
        while(nrf_flag_write_wait == 0)
        {
            Sys_GenFunDelayMs(10);
        }
        memset(Databuffer,0,10);
        NrfFlashDrive.read(&Message);
        Sys_GenFunDelayMs(1000);
    //    TxMessage.Lenth= 10;
    //    NrfFlashDrive.write((void *)&TxMessage);
        #else
        //NrfFlashDrive.read((void *)&TxMessage);
        #endif
    }
}
#endif 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

