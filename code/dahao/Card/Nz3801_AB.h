#ifndef __NZ_3801_AB_H_
#define __NZ_3801_AB_H_

#include "nrf_gpio.h"


#define NZ3801_CS    13
#define NZ3801_RST   15
#define NZ3801_MISO  10
#define NZ3801_MOSI  11
#define NZ3801_SCK   12

#define SPI_CS_Enable()   nrf_gpio_pin_clear(NZ3801_CS)
#define SPI_CS_Disable()  nrf_gpio_pin_set(NZ3801_CS)

#define Nz3801RST_Dis()  	nrf_gpio_pin_set(NZ3801_RST)
#define Nz3801RST_En()    nrf_gpio_pin_clear(NZ3801_RST)

#define Max_Len_NZ3801  10


#define  NZ3802_AB   1

#define BIT0	1		        //bit0 set mask
#define BIT1	2		        //bit1 set mask
#define BIT2	4		        //bit2 set mask
#define BIT3	8		        //bit3 set mask
#define BIT4	0x10		    //bit4 set mask
#define BIT5	0x20		    //bit5 set mask
#define BIT6	0x40		    //bit6 set mask
#define BIT7	0x80		    //bit7 set mask


//============================================================================
#define  BFL_JREG_EXT_REG_ENTRANCE    0x0F    //ext register entrance
//============================================================================
#define  BFL_JBIT_EXT_REG_WR_ADDR     0X40    //wrire address cycle
#define  BFL_JBIT_EXT_REG_RD_ADDR     0X80    //read address cycle
#define  BFL_JBIT_EXT_REG_WR_DATA     0XC0    //write data cycle
#define  BFL_JBIT_EXT_REG_RD_DATA     0X00    //read data cycle

/*******************************************************************************
* ---------------------页寄存器地址定义---------------------
*****************************************/
//---------------Page 0--------------------
#define  PAGE0           0x00    //Page register in page 0
#define  COMMAND         0x01    //Contains Command bits, PowerDown bit and bit to switch receiver off.
#define  COMMIEN         0x02    //Contains Communication interrupt enable bits andbit for Interrupt inversion.
#define  DIVIEN          0x03    //Contains RfOn, RfOff, CRC and Mode Interrupt enable and bit to switch Interrupt pin to PushPull mode.
#define  COMMIRQ         0x04    //Contains Communication interrupt request bits.
#define  DIVIRQ          0x05    //Contains RfOn, RfOff, CRC and Mode Interrupt request.
#define  REGERROR        0x06    //Contains Protocol, Parity, CRC, Collision, Buffer overflow, Temperature and RF error flags.
#define  STATUS1         0x07    //Contains status information about Lo- and HiAlert, RF-field on, Timer, Interrupt request and CRC status.
#define  STATUS2         0x08    //Contains information about internal states (Modemstate), states and possibility to switch Temperature sensor off.
#define  FIFODATA        0x09    //Gives access to FIFO. Writing to register increments theFIFO level (register 0x0A), reading decrements it.
#define  FIFOLEVEL       0x0A    //Contains the actual level of the FIFO.
#define  WATERLEVEL      0x0B    //Contains the Waterlevel value for the FIFO
#define  CONTROL         0x0C    //Contains information about last received bits, Initiator mode bit, bit to copy Nz3801ID to FIFO and to Start and stopthe Timer unit.
#define  BITFRAMING      0x0D    //Contains information of last bits to send, to align received bits in FIFO and activate sending in Transceive*/
#define  COLL            0x0E    //Contains all necessary bits for Collission handling
#define  RFU0F           0x0F    //Currently not used.

//---------------Page 1--------------------
#define  PAGE1           0x10    //Page register in page 1
//#define  MODE            0x11    //Contains bits for auto wait on Rf, to detect SYNC byte in Nz3801 mode and MSB first for CRC calculation
#define  TXMODE          0x12    //Contains Transmit Framing, Speed, CRC enable, bit for inverse mode and TXMix bit.
#define  RXMODE          0x13    //Contains Transmit Framing, Speed, CRC enable, bit for multiple receive and to filter errors.
#define  TXCONTROL       0x14    //Contains bits to activate and configure Tx1 and Tx2 and bit to activate 100% modulation.
#define  TXAUTO          0x15    //Contains bits to automatically switch on/off the Rf and to do the collission avoidance and the initial rf-on.
#define  TXSEL           0x16    //Contains SigoutSel, DriverSel and LoadModSel bits.
#define  RXSEL           0x17    //Contains UartSel and RxWait bits.
#define  RXTRESHOLD      0x18    //Contains MinLevel and CollLevel for detection.
#define  DEMOD           0x19    //Contains bits for time constants, hysteresis and IQ demodulator settings.
#define  FELICANz3801       0x1A    //Contains bits for minimum FeliCa length received and for FeliCa syncronisation length.
#define  FELICANz38012      0x1B    //Contains bits for maximum FeliCa length received.
#define  REG1CH          0x1C    //Contains Miller settings, TxWait settings and  halted mode bit.
#define  MANUALRCV       0x1D    //Currently not used.
#define  TYPEBREG        0x1E    //Currently not used.
#define  SERIALSPEED     0x1F    //Contains speed settings for serila interface.

//---------------Page 2--------------------
#define  PAGE2           0x20    //Page register in page 2
#define  CRCRESULT1      0x21    //Contains MSByte of CRC Result.
#define  CRCRESULT2      0x22    //Contains LSByte of CRC Result.
#define  GSNLOADMOD      0x23    //Contains the conductance and the modulation settings for the N-MOS transistor only for load modulation (See difference to BFL_JREG_GSN!).
#define  MODWIDTH        0x24    //Contains modulation width setting.
#define  TXBITPHASE      0x25    //Contains TxBitphase settings and receive clock change.
#define  RFCFG           0x26    //Contains sensitivity of Rf Level detector, the receiver gain factor and the RfLevelAmp.
#define  GSN             0x27    //Contains the conductance and the modulation settings for the N-MOS transistor during active modulation (no load modulation setting!).
#define  CWGSP           0x28    //Contains the conductance for the P-Mos transistor.
#define  MODGSP          0x29    //Contains the modulation index for the PMos transistor.
#define  TMODE           0x2A    //Contains all settings for the timer and the highest 4 bits of the prescaler.
#define  TPRESCALER      0x2B    //Contais the lowest byte of the prescaler.
#define  TRELOADHI       0x2C    //Contains the high byte of the reload value.
#define  TRELOADLO       0x2D    //Contains the low byte of the reload value.
#define  TCOUNTERVALHI   0x2E    //Contains the high byte of the counter value.
#define  TCOUNTERVALLO   0x2F    //Contains the low byte of the counter value.

//---------------Page 3--------------------
#define  PAGE3           0x30    //Page register in page 3
#define  TESTSEL1        0x31    //Test register
#define  TESTSEL2        0x32    //Test register
#define  TESTPINEN       0x33    //Test register
#define  TESTPINVALUE    0x34    //Test register
#define  TESTBUS         0x35    //Test register
#define  AUTOTEST        0x36    //Test register
#define  VERSION         0x37    //Contains the product number and the version .
#define  ANALOGTEST      0x38    //Test register
#define  TESTDAC1        0x39    //Test register
#define  TESTDAC2        0x3A    //Test register
#define  TESTADC         0x3B    //Test register
#define  ANALOGUETEST1   0x3C    //Test register
#define  ANALOGUETEST0   0x3D    //Test register
#define  ANALOGUETPD_A   0x3E    //Test register
#define  ANALOGUETPD_B   0x3F    //Test register


/*******************************************************************************
* ---------------------命令码---------------------
*****************************************/
#define  COMMAND_IDLE          0x00			//
#define  COMMAND_CONFIG        0x01			//
#define  COMMAND_RANDOMIDS     0x02			//
#define  COMMAND_CALCCRC       0x03			//
#define  COMMAND_TRANSMIT      0x04			//
#define  COMMAND_NOCMDCHANGE   0x07			//
#define  COMMAND_RECEIVE       0x08			//
#define  COMMAND_TRANSCEIVE    0x0C			//
#define  COMMAND_AUTOCOLL      0x0D			//
#define  COMMAND_AUTHENT       0x0E			//
#define  COMMAND_SOFTRESET     0x0F			//
#define  COMMAND_TRANSCEIVE_TO 0x8C			//



/*******************************************************************************
* ---------------------位操作定义，按页区分---------------------
*****************************************/ 
//-----------------------Page 0 位操作宏---------------
/* Command Register                          */
#define  BFL_JBIT_RCVOFF             0x20   /*Switches the receiver on/off. */
#define  BFL_JBIT_POWERDOWN          0x10   /*Switches NZ3801 to Power Down mode. */

/* CommIEn Register                          */
#define  BFL_JBIT_IRQINV             0x80   /*Inverts the output of IRQ Pin. */

/* DivIEn Register                           */
#define  BFL_JBIT_IRQPUSHPULL        0x80   /*Sets the IRQ pin to Push Pull mode. */

/* CommIEn and CommIrq Register              */
#define  BFL_JBIT_TXI                0x40   /*Bit position for Transmit Interrupt Enable/Request. */
#define  BFL_JBIT_RXI                0x20   /*Bit position for Receive Interrupt Enable/Request. */
#define  BFL_JBIT_IDLEI              0x10   /*Bit position for Idle Interrupt Enable/Request. */
#define  BFL_JBIT_HIALERTI           0x08   /*Bit position for HiAlert Interrupt Enable/Request. */
#define  BFL_JBIT_LOALERTI           0x04   /*Bit position for LoAlert Interrupt Enable/Request. */
#define  BFL_JBIT_ERRI               0x02   /*Bit position for Error Interrupt Enable/Request. */
#define  BFL_JBIT_TIMERI             0x01   /*Bit position for Timer Interrupt Enable/Request. */

/* DivIEn and DivIrq Register                */
#define  BFL_JBIT_MODEI              0x08   /*Bit position for Mode Interrupt Enable/Request. */
#define  BFL_JBIT_CRCI               0x04   /*Bit position for CRC Interrupt Enable/Request. */

/* CommIrq and DivIrq Register               */
#define  BFL_JBIT_SET                0x80   /*Bit position to set/clear dedicated IRQ bits. */

/* Error Register                            */
#define  BFL_JBIT_WRERR              0x40   /*Bit position for Write Access Error. */
#define  BFL_JBIT_TEMPERR            0x40   /*Bit position for Temerature Error. */
#define  BFL_JBIT_BUFFEROVFL         0x10   /*Bit position for Buffer Overflow Error. */
#define  BFL_JBIT_COLLERR            0x08   /*Bit position for Collision Error. */
#define  BFL_JBIT_CRCERR             0x04   /*Bit position for CRC Error. */
#define  BFL_JBIT_PARITYERR          0x02   /*Bit position for Parity Error. */
#define  BFL_JBIT_PROTERR            0x01   /*Bit position for Protocol Error. */

/* Status 1 Register                         */
#define  BFL_JBIT_CRCOK              0x40   /*Bit position for status CRC OK. */
#define  BFL_JBIT_CRCREADY           0x20   /*Bit position for status CRC Ready. */
#define  BFL_JBIT_IRQ                0x10   /*Bit position for status IRQ is active. */
#define  BFL_JBIT_TRUNNUNG           0x08   /*Bit position for status Timer is running. */
#define  BFL_JBIT_HIALERT            0x02   /*Bit position for status HiAlert. */
#define  BFL_JBIT_LOALERT            0x01   /*Bit position for status LoAlert. */

/* Status 2 Register                         */
#define  BFL_JBIT_TEMPSENSOFF        0x80   /*Bit position to switch Temperture sensors on/off. */
#define  BFL_JBIT_I2CFORCEHS         0x40   /*Bit position to forece High speed mode for I2C Interface. */
#define  BFL_JBIT_CRYPTO1ON          0x08   /*Bit position for reader status Crypto is on. */

/* FIFOLevel Register                        */
#define  BFL_JBIT_FLUSHBUFFER        0x80   /*Clears FIFO buffer if set to 1 */

/* Control Register                          */
#define  BFL_JBIT_TSTOPNOW           0x80   /*Stops timer if set to 1. */
#define  BFL_JBIT_TSTARTNOW          0x40   /*Starts timer if set to 1. */

/* BitFraming Register                       */
#define  BFL_JBIT_STARTSEND          0x80   /*Starts transmission in transceive command if set to 1. */

/* BitFraming Register                       */
#define  BFL_JBIT_VALUESAFTERCOLL    0x80   /*Activates mode to keep data after collision. */

//-----------------------Page 1 位操作宏--------------- 
/* Mode Register                             */
#define  BFL_JBIT_MSBFIRST           0x80   /*Sets CRC coprocessor with MSB first. */
#define  BFL_JBIT_TXWAITRF           0x20   /*Tx waits until Rf is enabled until transmit is startet, else transmit is started immideately. */
#define  BFL_JBIT_POLSIGIN           0x08   /*Inverts polarity of SiginActIrq, if bit is set to 1 IRQ occures when Sigin line is 0. */

/* TxMode Register                           */
#define  BFL_JBIT_INVMOD             0x08   /*Activates inverted transmission mode. */

/* RxMode Register                           */
#define  BFL_JBIT_RXNOERR            0x08   /*If 1, receiver does not receive less than 4 bits. */
#define  BFL_JBIT_RXMULTIPLE         0x04   /*Activates reception mode for multiple responses. */

/* Definitions for Tx and Rx                 */
#define  BFL_JBIT_106KBPS            0x00   /*Activates speed of 106kbps. */
#define  BFL_JBIT_212KBPS            0x10   /*Activates speed of 212kbps. */
#define  BFL_JBIT_424KBPS            0x20   /*Activates speed of 424kbps. */
#define  BFL_JBIT_848KBPS            0x30   /*Activates speed of 848kbps. */

#define  BFL_JBIT_TYPEA              0x00   /*Activates TYPEA communication mode. */
#define  BFL_JBIT_TYPEB              0x03   /*Activates TYPEB communication mode. */
#define  BFL_JBIT_CRCEN              0x80   /*Activates transmit or receive CRC. */

/* TxControl Register                        */
#define  BFL_JBIT_INVTX2ON           0x80   /*Inverts the Tx2 output if drivers are switched on. */
#define  BFL_JBIT_INVTX1ON           0x40   /*Inverts the Tx1 output if drivers are switched on. */
#define  BFL_JBIT_INVTX2OFF          0x20   /*Inverts the Tx2 output if drivers are switched off. */
#define  BFL_JBIT_INVTX1OFF          0x10   /*Inverts the Tx1 output if drivers are switched off. */
#define  BFL_JBIT_TX2CW              0x08   /*Does not modulate the Tx2 output, only constant wave. */
#define  BFL_JBIT_TX2RFEN            0x02   /*Switches the driver for Tx2 pin on. */
#define  BFL_JBIT_TX1RFEN            0x01   /*Switches the driver for Tx1 pin on. */

/* TxAuto Register                           */
#define  BFL_JBIT_FORCE100ASK        0x40   /*Activates 100%ASK mode independent of driver settings. */
/* Demod Register                            */
#define  BFL_JBIT_FIXIQ              0x20   /*If set to 1 and the lower bit of AddIQ is set to 0, the receiving is fixed to I channel.
                                                    If set to 1 and the lower bit of AddIQ is set to 1, the receiving is fixed to Q channel. */
/* RFU 0x1D Register                         */
#define  BFL_JBIT_PARITYDISABLE      0x10   /*Disables the parity generation and sending independent from the mode. */

//-----------------------Page 2 位操作宏--------------- 
/* TMode Register                           (2A) */
#define  BFL_JBIT_TAUTO              0x80   /*Sets the Timer start/stop conditions to Auto mode. */
#define  BFL_JBIT_TAUTORESTART       0x10   /*Restarts the timer automatically after finished counting down to 0. */

//-----------------------Page 3 位操作宏--------------- 
/* AutoTest Register                        (36) */
#define  BFL_JBIT_AMPRCV             0x40   /* */


/*******************************************************************************
* ---------------------位操作掩码---------------------
*****************************************/
/* Command register                 (0x01)*/
#define  BFL_JMASK_COMMAND           0x0F   /*Bitmask for Command bits in Register BFL_JREG_COMMAND. */
#define  BFL_JMASK_COMMAND_INV       0xF0   /*Inverted bitmask of BFL_JMASK_COMMAND. */

/* Waterlevel register              (0x0B)*/
#define  BFL_JMASK_WATERLEVEL        0x3F   /*Bitmask for Waterlevel bits in register BFL_JREG_WATERLEVEL. */

/* Control register                 (0x0C)*/
#define  BFL_JMASK_RXBITS            0x07   /*Bitmask for RxLast bits in register BFL_JREG_CONTROL. */

/* Mode register                    (0x11)*/
#define  BFL_JMASK_CRCPRESET         0x03   /*Bitmask for CRCPreset bits in register BFL_JREG_MODE. */

/* TxMode register                  (0x12, 0x13)*/
#define  BFL_JMASK_SPEED             0x70   /*Bitmask for Tx/RxSpeed bits in register BFL_JREG_TXMODE and BFL_JREG_RXMODE. */
#define  BFL_JMASK_FRAMING           0x03   /*Bitmask for Tx/RxFraming bits in register BFL_JREG_TXMODE and BFL_JREG_RXMODE. */

/* TxSel register                   (0x16)*/
#define  BFL_JMASK_LOADMODSEL        0xC0   /*Bitmask for LoadModSel bits in register BFL_JREG_TXSEL. */
#define  BFL_JMASK_DRIVERSEL         0x30   /*Bitmask for DriverSel bits in register BFL_JREG_TXSEL. */
#define  BFL_JMASK_SIGOUTSEL         0x0F   /*Bitmask for SigoutSel bits in register BFL_JREG_TXSEL. */

/* RxSel register                   (0x17)*/
#define  BFL_JMASK_UARTSEL           0xC0   /*Bitmask for UartSel bits in register BFL_JREG_RXSEL. */
#define  BFL_JMASK_RXWAIT            0x3F   /*Bitmask for RxWait bits in register BFL_JREG_RXSEL. */

/* RxThreshold register             (0x18)*/
#define  BFL_JMASK_MINLEVEL          0xF0   /*Bitmask for MinLevel bits in register BFL_JREG_RXTHRESHOLD. */
#define  BFL_JMASK_COLLEVEL          0x07   /*Bitmask for CollLevel bits in register BFL_JREG_RXTHRESHOLD. */

/* Demod register                   (0x19)*/
#define  BFL_JMASK_ADDIQ             0xC0   /*Bitmask for ADDIQ bits in register BFL_JREG_DEMOD. */
#define  BFL_JMASK_TAURCV            0x0C   /*Bitmask for TauRcv bits in register BFL_JREG_DEMOD. */
#define  BFL_JMASK_TAUSYNC           0x03   /*Bitmask for TauSync bits in register BFL_JREG_DEMOD. */

/* FeliCa / FeliCa2 register        (0x1A, 0x1B)*/
#define  BFL_JMASK_FELICASYNCLEN     0xC0   /*Bitmask for FeliCaSyncLen bits in registers BFL_JREG_FELICANz3801. */
#define  BFL_JMASK_FELICALEN         0x3F   /*Bitmask for FeliCaLenMin and FeliCaLenMax in
                                                    registers BFL_JREG_FELICANz3801 and BFL_JREG_FELICANz38012. */
/* REG1CH register                  (0x1C)*/
#define  BFL_JMASK_SENSMILLER        0xE0   /*Bitmask for SensMiller bits in register BFL_JREG_REG1CH. */
#define  BFL_JMASK_TAUMILLER         0x18   /*Bitmask for TauMiller bits in register BFL_JREG_REG1CH. */
#define  BFL_JMASK_TXWAIT            0x03   /*Bitmask for TxWait bits in register BFL_JREG_REG1CH. */

/* Manual Rcv register              (0x1D)*/
#define  BFL_JMASK_HPCF				0x03   /*Bitmask for HPCF filter adjustments. */

/* TxBitPhase register              (0x25)*/
#define  BFL_JMASK_TXBITPHASE        0x7F   /*Bitmask for TxBitPhase bits in register BFL_JREG_TXBITPHASE. */

/* RFCfg register                   (0x26)*/
#define  BFL_JMASK_RXGAIN            0x70   /*Bitmask for RxGain bits in register BFL_JREG_RFCFG. */
#define  BFL_JMASK_RFLEVEL           0x0F   /*Bitmask for RfLevel bits in register BFL_JREG_RFCFG. */

/* GsN register                     (0x27)*/
#define  BFL_JMASK_CWGSN             0xF0   /*Bitmask for CWGsN bits in register BFL_JREG_GSN. */
#define  BFL_JMASK_MODGSN            0x0F   /*Bitmask for ModGsN bits in register BFL_JREG_GSN. */

/* CWGsP register                   (0x28)*/
#define  BFL_JMASK_CWGSP             0x3F   /*Bitmask for CWGsP bits in register BFL_JREG_CWGSP. */

/* ModGsP register                  (0x29)*/
#define  BFL_JMASK_MODGSP            0x3F   /*Bitmask for ModGsP bits in register BFL_JREG_MODGSP. */

/* TMode register                   (0x2A)*/
#define  BFL_JMASK_TGATED            0x60   /*Bitmask for TGated bits in register BFL_JREG_TMODE. */
#define  BFL_JMASK_TPRESCALER_HI     0x0F   /*Bitmask for TPrescalerHi bits in register BFL_JREG_TMODE. */


/*******************************************************************************
* ---------------------B卡操作相关宏---------------------
*****************************************/ 
#define ISO14443B_PUPI_LENGTH        4
#define ISO14443B_APPDATA_LENGTH     4
#define ISO14443B_PROTINFO_LENGTH    3

#define ISO14443B_PARAM_APF          0x5
#define ISO14443B_MAX_AC_LOOP_COUNT  5


/*******************************************************************************
* ---------------------错误码定义---------------------
*****************************************/ 
#define ERR_NONE           0 
#define SUCCEED_NZ3801  	 ERR_NONE

#define ERR_INIT           1
#define ERR_CMD_TIMEOUT    2
#define ERR_KEY            3
#define ERR_ACCESS         4
#define ERR_FIFO_OFL       5
#define ERR_CRC            6
#define ERR_FRAMING        7
#define ERR_PARITY         8
#define ERR_COLL           9
#define ERR_UID            10
#define ERR_PARA           11
#define ERR_DATA           12
#define ERR_FDT            13
#define ERR_TIMEOUT        14
#define ERR_TRANSMIT       15
#define ERR_OVERLOAD       16
#define ERR_PROTOCOL       17
#define ERR_ONOFFFIELD     18
#define ERR_EOT_IND        19
#define ERR_EOT_IND_30MS   20
#define ERR_NOTFOUND       21
#define ERR_GENERAL        99

#ifndef NZ3801_FALSE
   #define NZ3801_FALSE			0
#endif

#ifndef NZ3801_TRUE
   #define NZ3801_TRUE			(!NZ3801_FALSE)
#endif


/*******************************************************************************
* ---------------------通信参数配置---------------------
*****************************************/ 
#define ISO14443A_MAX_UID_LENGTH 	 		10
#define ISO14443A_MAX_CASCADE_LEVELS 	3
#define ISO14443A_CASCADE_LENGTH 	 		7
#define ISO14443A_RESPONSE_CT  		 		0x88


/*******************************************************************************
* 库编译配置   A卡B卡使能，Debug信息
*******************************************************************************/

#define FIELD_ONOFF_RETRY_EN  0   //场强启动容错机制使能开关: 1使能，0禁止；容错次数5次


//====================================================================================================
//   ---End of Macro Define
//====================================================================================================



/*******************************************************************************
* ---------------------结构体定义---------------------
*****************************************/ 
//----------A卡和B卡宏定义-------------------
typedef enum
{
    CT_A = 0,	// TypeA模式
    CT_B,		// TypeB模式
} teCardType;

//----------卡操作命令码-------------------
typedef enum
{
    TA_REQA,
    TA_WUPA,
    TA_HLTA,
    TA_ANT,
    TA_SELECT,
    TA_RATS,
    TA_PPS,
    TA_IBLOCK,
    TA_RSBLOCK,
    TA_REQB,
    TB_WUPB,
    TB_ATTRIB,
    TB_HLTB,
    TB_IBLOCK,
    TB_xBLOCK, //二代身份证
    CMD_TOTAL
} eCmd;

//--- PCD command set.A---------------------------------/
typedef enum
{
    ISO14443A_CMD_REQA = 0x26,  /*!< command REQA */
    ISO14443A_CMD_WUPA = 0x52, /*!< command WUPA */
    ISO14443A_CMD_SELECT_CL1 = 0x93, /*!< command SELECT cascade level 1 */
    ISO14443A_CMD_SELECT_CL2 = 0x95, /*!< command SELECT cascade level 2 */
    ISO14443A_CMD_SELECT_CL3 = 0x97, /*!< command SELECT cascade level 3 */
    ISO14443A_CMD_HLTA = 0x50, /*!< command HLTA */
    ISO14443A_CMD_PPSS = 0xd0, /*!< command PPSS */
    ISO14443A_CMD_RATS = 0xe0, /*!< command RATS */
} iso14443ACommand_t;

//--- PCD command set.B---------------------------------/
typedef enum
{
    ISO14443B_CMD_REQB   = 0x00, /*!< command REQB */
    ISO14443B_CMD_WUPB   = 0x08, /*!< command WUPB */
    ISO14443B_CMD_HLTB   = 0x50, /*!< command HLTB */
    ISO14443B_CMD_ATTRIB = 0x1D, /*!< command ATTRIB */
} iso14443BCommand_t;

//--- slot count (N parameter) used for Bcard anticollision ---------------------------------/
typedef enum
{
    ISO14443B_SLOT_COUNT_1  = 0,
    ISO14443B_SLOT_COUNT_2  = 1,
    ISO14443B_SLOT_COUNT_4  = 2,
    ISO14443B_SLOT_COUNT_8  = 3,
    ISO14443B_SLOT_COUNT_16 = 4,
} iso14443BSlotCount_t;

//--- struct representing the content of ATQB---------------------------------/
	typedef struct
	{
		unsigned char atqb; /*<! content of answer to request byte */
		unsigned char pupi[ISO14443B_PUPI_LENGTH]; /*!< UID of the PICC */
		unsigned char applicationData[ISO14443B_APPDATA_LENGTH]; /*!< application specific data */
		unsigned char protocolInfo[ISO14443B_PROTINFO_LENGTH]; /*!< protocol info */
		unsigned char collision; /*!< TRUE, if there was a collision which has been resolved,
												otherwise no collision occured */
	} iso14443BProximityCard_t;

//--- struct representing the content of ATQB---------------------------------/
typedef struct
{
    unsigned char param1;
    unsigned char param2;
    unsigned char param3;
    unsigned char param4;
} iso14443BAttribParameter_t;

//--- struct holding the answer to ATTRIB command---------------------------------/
typedef struct
{
    unsigned char mbli; /*!< maximum buffer length */
    unsigned char cid;  /*!< logical card identifier */
} iso14443BAttribAnswer_t;

//----------操作成功率统计-------------------
typedef struct
{
    unsigned int numFieldOnFail;
    unsigned int numFieldOffFail;
    unsigned int numL3OK;
    unsigned int numL4OK;
    unsigned int numWrRegOK;
    unsigned int Totality;
} tsSuccessRate; 

/** struct representing an ISO14443A PICC as returned by * #iso14443ASelect.*/
typedef struct  //_AProximityCard_s
{
    unsigned char uid[ISO14443A_MAX_UID_LENGTH]; /*<! UID of the PICC */
    unsigned char actlength;     /*!< actual UID length */
    unsigned char atqa[2];       /*!< content of answer to request byte */
    unsigned char sak[ISO14443A_MAX_CASCADE_LEVELS]; /*!< SAK bytes */
    unsigned char cascadeLevels; /*!< number of cascading levels */
    unsigned char collision;   /*!< TRUE, if there was a collision which has been resolved,
                        otherwise no collision occured */
    unsigned char fsdi;          /*!< Frame size integer of the PCD. */

    unsigned char fwi;           /*!< Frame wait integer of the PICC. */
    unsigned char fsci;          /*!< Frame size integer of the PICC. */
    unsigned char sfgi;          /*!< Special frame guard time of the PICC. */
    unsigned char TA;            /*!< Data rate bits PICC->PCD. Data rate bits PCD->PICC.*/
} iso14443AProximityCard_t;





//extern unsigned char  CType;		 		// 0 TypeA, 1 TypeB



/*******************************************************************************
---函数：void *mem_copy(void * dest,const void *src, unsigned short count)
---功能：工具函数，缓存指定长度数据拷贝
---输入：dest-拷贝的目的地址， src-拷贝的数据源地址, count-拷贝长度
---输出：
---备注：
*************************************************/
void *mem_copy(void * dest,const void *src, unsigned short count);

/*******************************************************************************
---函数：void KCT_SetBitMask(unsigned char reg,unsigned char mask)
---功能：寄存器置位操作，不会影响到其他bit变换
---输入：reg - 寄存器地址
				 mask - 操作的位，或运算
---输出：
---备注：
*************************************************/
void KCT_SetBitMask(unsigned char reg,unsigned char mask);

/*******************************************************************************
---函数：void KCT_ClearBitMask(unsigned char reg,unsigned char mask)
---功能：寄存器清零操作，不影响其他bit
---输入：reg - 寄存器地址
				 mask - 操作的位值，非与运算
---输出：
---备注：
*************************************************/
void KCT_ClearBitMask(unsigned char reg,unsigned char mask);

/*******************************************************************************
---函数：void KCT_SetRegExt(unsigned char extRegAddr,unsigned char extRegData)
---功能：
---输入：
---输出：
---备注：
*************************************************/
void KCT_SetRegExt(unsigned char extRegAddr,unsigned char extRegData);

/*******************************************************************************
---函数：void KCT_ClearFifo(void)
---功能：清除缓存
---输入：无
---输出：无
---备注：
*************************************************/
void KCT_ClearFifo(void);

/*******************************************************************************
---函数：void KCT_ClearFlag(void)
---功能：清除IRQ标记
---输入：
---输出：
---备注：
*************************************************/
void KCT_ClearFlag(void);

/*******************************************************************************
---函数：void KCT_StartCmd(unsigned char cmd)
---功能：向命令寄存器中写入命令并马上执行
---输入：	cmd = 0x0, 	COMMAND_IDLE 取消当前执行的命令
							= 0x1,	COMMAND_CONFIG，存储25字节到内部Buffer
							= 0x2，	COMMAND_RANDOMIDS，产生10字节随机数
							=	0x3,	COMMAND_CALCCRC，激活CRC协处理器或执行自测试
							=	0x4,	COMMAND_TRANSMIT，发送缓冲区命令
							=	0x7,	COMMAND_NOCMDCHANGE，用于修改命令寄存器的不同位，不出发其他命令
							=	0x8,	COMMAND_RECEIVE，激活接收机
							=	0xC,	COMMAND_TRANSCEIVE，启动发送，发送完成马上接收
							= 0xE,	启动MF卡标准认证流程
							=	0xF,	软件复位RC522
---输出：
---备注：
*************************************************/
void KCT_StartCmd(unsigned char cmd);

/*******************************************************************************
---函数：void KCT_StopCmd(void)
---功能：取消当前执行的命令
---输入：无
---输出：无
---备注：
*************************************************/
void KCT_StopCmd(void);

/*******************************************************************************
---函数：unsigned char KCT_FlagOK(void)
---功能：读取错误标记，并分析是否有CRC错误/协议错误/奇偶错误/
---输入：无
---输出：0-无错误， 非0-有错误
---备注：
*************************************************/
unsigned char KCT_FlagOK(void);

/*******************************************************************************
---函数：unsigned char KCT_CrcOK(void)
---功能：判断是否有CRC错误
---输入：无
---输出：结果
---备注：
*************************************************/
unsigned char KCT_CrcOK(void);

/*******************************************************************************
---函数：void Nz3801_SetTimer(unsigned int fc)
---功能：设置NZ3801-AB的射频超时时间 方式1
---输入：
---输出：
---备注：domod TPrescalEven bit=0
*************************************************/
void Nz3801_SetTimer(unsigned int fc);

/*******************************************************************************
---函数：unsigned char Nz3801_SoftReset(void)
---功能：通过命令触发软件复位
---输入：
---输出：
---备注：
*************************************************/
unsigned char Nz3801_SoftReset(void);

/*******************************************************************************
---函数：unsigned char Nz3801_SoftPwrDown(unsigned char bEnable)
---功能：设置进入PD模式待机，或者唤醒芯片
---输入：
---输出：
---备注：
*************************************************/
unsigned char Nz3801_SoftPwrDown(unsigned char bEnable);

/*******************************************************************************
---函数：unsigned char Nz3801_ActivateField(unsigned char activateField)
---功能：控制开关场强
---输入：activateField = NZ3801_TRUE，开启场强；
											 = NZ3801_FALSE，关闭场强；
---输出：
---备注：
*************************************************/
unsigned char Nz3801_ActivateField(unsigned char activateField);

/*******************************************************************************
---函数：unsigned char Nz3801_Init(teCardType card)
---功能：根据输入的协议类型初始化Nz3801_寄存器
---输入：
---输出：
---备注：
*************************************************/
unsigned char Nz3801_Init_CPU(teCardType card);

/*******************************************************************************
---函数：Nz3801_Xfer
---功能：处理数据传输
---输入：txalign  最后1个字节发送多少bit
				 rxalign  收数据时从第几个bit开始
				 len  发送数据长度，为0时只收不发
---输出：
---备注：
*************************************************/
unsigned char Nz3801_Xfer(eCmd command, const unsigned char *request, 
									unsigned char requestLength, unsigned char txalign,
                  unsigned char *response, unsigned char *responseLength, unsigned char rxalign);


/*******************************************************************************
---函数：unsigned char iso14443ASendHlta(void)
---功能：命令已激活的卡片进入休眠状态，停卡
---输入：none
---输出：result
---备注：
*************************************************/
unsigned char iso14443ASendHlta(void);

/*******************************************************************************
---函数：unsigned char iso14443ASelect(iso14443ACommand_t cmd, iso14443AProximityCard_t* card)
---功能：A卡选卡函数，执行完初始化之后就可以操作选卡
---输入： cmd = 0x26[ISO14443A_CMD_REQA] - 寻未进入休眠状态的卡
							= 0x52[ISO14443A_CMD_WUPA] - 寻所有状态的卡
---输出：iso14443AProximityCard_t* card，寻卡成功时配置如下三个参数:
				card->atqa  : 卡类型
				card->uid	:		卡序列号
				card->sak :		卡SAK
---备注：在一个感应区域内的同一时刻，只可能有一张TypaA卡处于被选中状态
*************************************************/
unsigned char iso14443ASelect(iso14443ACommand_t cmd, iso14443AProximityCard_t* card);

/*******************************************************************************
---函数：unsigned char iso14443AEnterProtocolMode(iso14443AProximityCard_t* card, unsigned char* answer, unsigned char* length)
---功能：请求RATS，激活A卡，进入ISO14443-4应用层
---输入：card ->fsdi -PCD帧的最大长度
				 card ->CID = 0
---输出：card -> fsci - PICC帧的最大长度
				 card -> sfgi - PICC应答ATS后下一帧的保护时间
				 card -> TA -收发位速率
				 answer:卡应答ATS数据
				 length: 返回的ATS长度
---备注：
*************************************************/
unsigned char iso14443AEnterProtocolMode(iso14443AProximityCard_t* card, unsigned char* answer, unsigned char* length);

/*******************************************************************************
---函数：unsigned char iso14443ASendProtocolAndParameterSelection(unsigned char pps1)
---功能：A 协议和参数选择，如PPS，在-4层协议需要用到
---输入：PPS1 - 协商传输位速率
					PPS1 = 0x0, 选择默认106KBd			PPS1 = 0x05, 选择212KBd
					PPS1 = 0x0A, 选择424KBd					PPS1 = 0x0F，选择848KBd
---输出：result
---备注：
*************************************************/
unsigned char iso14443ASendProtocolAndParameterSelection(unsigned char pps1);

//==========End of ISO14443A_EN-- Line710 ~ Line770 =====================================================



unsigned char ApduTransceive(unsigned char  *inf,        unsigned int infLength, 
	                           unsigned char  *response,   unsigned int *responseLength);

//-----------ISO14443B ------------------
/*******************************************************************************
---函数：unsigned char iso14443BSendHltb(iso14443BProximityCard_t* card)
---功能：命令激活的卡片进入休眠状态，停卡
---输入：
---输出：
---备注：
*************************************************/
unsigned char iso14443BSendHltb(iso14443BProximityCard_t* card);

/*******************************************************************************
---函数：
---功能：TypeB 寻卡，防碰撞选卡操作
---输入：cmd =  0x00[ISO14443B_CMD_REQB] - 寻未进入休眠状态的卡
						 =  0x08[ISO14443B_CMD_WUPB] - 寻所有状态的卡
				 afi ： Type B应用族识别符，具体见ISO14443协议
---输出：card -> pupi -Type B伪唯一PICC卡标识符
				 card -> applicationData - 应用数据，用于通知PCD在PICC当前安装了哪些应用
				 card -> protocolInfo - 指示了卡所支持的参数，如帧最大长度/帧等待时间/位速率
---备注：在一个感应区域内的同一时刻，只可有一张B卡处于选中状态
*************************************************/
//unsigned char iso14443BSelect(iso14443BCommand_t cmd, iso14443BProximityCard_t* card, unsigned char afi, iso14443BSlotCount_t slotCount);
unsigned char iso14443BSelect(iso14443BCommand_t cmd,
                              iso14443BProximityCard_t* card,
                              unsigned char afi);

/*******************************************************************************
---函数：iso14443BEnterProtocolMode
---功能：Attrib命令，激活B卡，进入ISO14443-4应用层
---输入：	card -> pupi, TypeB伪唯一 PICC卡标识符，通过iso14443BSelect()选卡获取
					param: baok 4个参数，具体见协议
---输出：	answer: AttriB应答数据，包含MBLI和CID
---备注：
*************************************************/

unsigned char iso14443BEnterProtocolMode(iso14443BProximityCard_t* card,
        iso14443BAttribParameter_t* param,
        iso14443BAttribAnswer_t* answer);

void PcdAntennaOff(void);
void PcdAntennaOn(void);

unsigned char Iso14443ADemonstrate(void);
unsigned char Iso14443BDemonstrate(void);
unsigned char CertificationCPUFob(void);

void Fob_Check(void);
void ADC_CheckCard( unsigned int value  );
void NZ3801_Open( void );

#endif




