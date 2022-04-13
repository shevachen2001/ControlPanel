/*****************************************************************************
|  File Name: Flash.h
|
|  Description: Header File defined
|
|-----------------------------------------------------------------------------
|               C O P Y R I G H T
|-----------------------------------------------------------------------------
|-----------------------------------------------------------------------------
|               A U T H O R   I D E N T I T Y
|-----------------------------------------------------------------------------
| Initials     Name                      Company
| --------     ---------------------     -------------------------------------
| LXW          LXW               
|-----------------------------------------------------------------------------
|               R E V I S I O N   H I S T O R Y
|-----------------------------------------------------------------------------
| Date        Ver     Author  Description
| ---------   ----    ------  ------------------------------------------------
| 2013-10      LXW     Initial version 
|****************************************************************************/

#ifndef NRF_FLASH_H
#define NRF_FLASH_H
#include "Flash\Flash_Cfg.h"

/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
//#define NRF_FLASH_ADDR        ((u32)0x08000000)
//#define NRF_PAGE_SIZE            ((u32)0x0400)
//#define NRF_START_PAGE        ((u32)120)
//#define NRF_END_PAGE            ((u32)8)
//#define NRF_FLASH_CHECK  STD_TRUE

enum
{
    NRF_FLASH_SYS_PRAR = 0,
    NRF_FLASH_SYS_PRARBK,
    NRF_FLASH_BLACK_LIST,
    NRF_FLASH_BLACK_LISTBK,
    NRF_FLASH_SERVE_KEY,
    NRF_FLASH_SERVE_KEYBK,
    /*NRF_FLASH_APP_KEY,
    NRF_FLASH_APP_KEYBK,*/
    NRF_FLASH_RECORD,
    NRF_FLASH_END
};


/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/
typedef struct
{
    uint8    Sector;
    uint16    Lenth;
    uint8       *pData;
    uint32    Add;
} NrfFlash_PstorageType;

/*typedef enum
{
    NRF_FLASH_ERASE_SECT = 0,
    NRF_FLASH_ERASE_CHIP,
    NRF_FLASH_CMD_MAX
} NrfFlash_CmdType;*/

typedef enum
{
    NRFFLASH_SCAN = 0,
    NRFFLASH_STORE,
    NRFFLASH_STORE_DIR,  /* ֱ�Ӵ洢*/
    NRFFLASH_CMD_MAX
} NrfFlash_CmdType;

/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/
extern const Dr_OpertonType NrfFlashDrive ;
/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/
extern void NrfFlash_Init(void);
#if (NRF_FLASH_CHECK == STD_TRUE)
extern void Flash_Check(void);
#endif

#endif    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

