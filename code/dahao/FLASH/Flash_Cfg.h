/*****************************************************************************
|  File Name: Flash_Cfg.h
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

#ifndef FLASH_CFG_H
#define FLASH_CFG_H
#include"PUBLIC\Std_Types.h"

/******************************************************************************
**********************Macro definition*******************************************
******************************************************************************/
/* SPI Flash supported commands */
#if 0    /* SST25VF016B */
#define FLASH_CMD_AAI       0xAD  /* AAI ¨¢?D?¡À¨¤3¨¬??¨¢? */
#define FLASH_CMD_DISWR      0x04    /* ???1D¡ä, ¨ª?3?AAI¡Á¡ä¨¬? */

#define FLASH_CMD_EWRSR      0x50    /* ?¨ºD¨ªD¡ä¡Á¡ä¨¬???¡ä??¡Â */
#define FLASH_CMD_WRSR      0x01  /* Write Status Register instruction */
#define FLASH_CMD_WREN      0x06  /* Write enable instruction */
#define FLASH_CMD_READ      0x03  /* Read from Memory instruction */
#define FLASH_CMD_RDSR      0x05  /* Read Status Register instruction  */
#define FLASH_CMD_RDID      0x9F  /* Read identification */
#define FLASH_CMD_SE        0x20  /* Sector Erase instruction */
#define FLASH_CMD_BE        0xC7  /* Bulk Erase instruction */

#define WIP_FLAG      0x01  /* Write In Progress (WIP) flag */
#define DUMMY_BYTE    0xA5
#else    /*     M25P64 */   /*     W25Q32JV */
#define FLASH_CMD_WRITE     0x02  /* Write to Memory instruction  Page Program */
#define FLASH_CMD_WRSR      0x01  /* Write Status Register instruction */
#define FLASH_CMD_WREN      0x06  /* Write enable instruction */
#define FLASH_CMD_READ      0x03  /* Read from Memory instruction */
#define FLASH_CMD_RDSR      0x05  /* Read Status Register instruction  */
#define FLASH_CMD_RDID      0x9F  /* Read identification */
#define FLASH_CMD_SE        0x20//0xD8  /* Sector Erase instruction */
#define FLASH_CMD_BL        0xD8//0xD8  /* BLock Erase instruction */
#define FLASH_CMD_BE        0xC7  /* Bulk Erase instruction */
#define WIP_FLAG      0x01  /* Write In Progress (WIP) flag */
#define DUMMY_BYTE    0xA5
#endif

#if 0    /* SST25VF016B */
#define SPI_FLASH_PAGESIZE    (4*1024)
#else
#define SPI_FLASH_PAGESIZE    256
#define SPI_FLASH_SECTORSIZE  128
#define SPI_FLASH_BLOCKSIZE   4
#endif
#define FLASH_CHECK        STD_FALSE
#define FLASH_SIZE    SPI_FLASH_PAGESIZE*SPI_FLASH_SECTORSIZE*SPI_FLASH_BLOCKSIZE

#define STM32FLASH_EN  STD_TRUE

/******************************************************************************
**********************Type statement*******************************************
******************************************************************************/


/******************************************************************************
**********************Variable definition*******************************************
******************************************************************************/

/******************************************************************************
**********************Function declared*******************************************
******************************************************************************/

#endif
