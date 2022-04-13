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
* ---------------------ҳ�Ĵ�����ַ����---------------------
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
* ---------------------������---------------------
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
* ---------------------λ�������壬��ҳ����---------------------
*****************************************/ 
//-----------------------Page 0 λ������---------------
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

//-----------------------Page 1 λ������--------------- 
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

//-----------------------Page 2 λ������--------------- 
/* TMode Register                           (2A) */
#define  BFL_JBIT_TAUTO              0x80   /*Sets the Timer start/stop conditions to Auto mode. */
#define  BFL_JBIT_TAUTORESTART       0x10   /*Restarts the timer automatically after finished counting down to 0. */

//-----------------------Page 3 λ������--------------- 
/* AutoTest Register                        (36) */
#define  BFL_JBIT_AMPRCV             0x40   /* */


/*******************************************************************************
* ---------------------λ��������---------------------
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
* ---------------------B��������غ�---------------------
*****************************************/ 
#define ISO14443B_PUPI_LENGTH        4
#define ISO14443B_APPDATA_LENGTH     4
#define ISO14443B_PROTINFO_LENGTH    3

#define ISO14443B_PARAM_APF          0x5
#define ISO14443B_MAX_AC_LOOP_COUNT  5


/*******************************************************************************
* ---------------------�����붨��---------------------
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
* ---------------------ͨ�Ų�������---------------------
*****************************************/ 
#define ISO14443A_MAX_UID_LENGTH 	 		10
#define ISO14443A_MAX_CASCADE_LEVELS 	3
#define ISO14443A_CASCADE_LENGTH 	 		7
#define ISO14443A_RESPONSE_CT  		 		0x88


/*******************************************************************************
* ���������   A��B��ʹ�ܣ�Debug��Ϣ
*******************************************************************************/

#define FIELD_ONOFF_RETRY_EN  0   //��ǿ�����ݴ����ʹ�ܿ���: 1ʹ�ܣ�0��ֹ���ݴ����5��


//====================================================================================================
//   ---End of Macro Define
//====================================================================================================



/*******************************************************************************
* ---------------------�ṹ�嶨��---------------------
*****************************************/ 
//----------A����B���궨��-------------------
typedef enum
{
    CT_A = 0,	// TypeAģʽ
    CT_B,		// TypeBģʽ
} teCardType;

//----------������������-------------------
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
    TB_xBLOCK, //�������֤
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

//----------�����ɹ���ͳ��-------------------
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
---������void *mem_copy(void * dest,const void *src, unsigned short count)
---���ܣ����ߺ���������ָ���������ݿ���
---���룺dest-������Ŀ�ĵ�ַ�� src-����������Դ��ַ, count-��������
---�����
---��ע��
*************************************************/
void *mem_copy(void * dest,const void *src, unsigned short count);

/*******************************************************************************
---������void KCT_SetBitMask(unsigned char reg,unsigned char mask)
---���ܣ��Ĵ�����λ����������Ӱ�쵽����bit�任
---���룺reg - �Ĵ�����ַ
				 mask - ������λ��������
---�����
---��ע��
*************************************************/
void KCT_SetBitMask(unsigned char reg,unsigned char mask);

/*******************************************************************************
---������void KCT_ClearBitMask(unsigned char reg,unsigned char mask)
---���ܣ��Ĵ��������������Ӱ������bit
---���룺reg - �Ĵ�����ַ
				 mask - ������λֵ����������
---�����
---��ע��
*************************************************/
void KCT_ClearBitMask(unsigned char reg,unsigned char mask);

/*******************************************************************************
---������void KCT_SetRegExt(unsigned char extRegAddr,unsigned char extRegData)
---���ܣ�
---���룺
---�����
---��ע��
*************************************************/
void KCT_SetRegExt(unsigned char extRegAddr,unsigned char extRegData);

/*******************************************************************************
---������void KCT_ClearFifo(void)
---���ܣ��������
---���룺��
---�������
---��ע��
*************************************************/
void KCT_ClearFifo(void);

/*******************************************************************************
---������void KCT_ClearFlag(void)
---���ܣ����IRQ���
---���룺
---�����
---��ע��
*************************************************/
void KCT_ClearFlag(void);

/*******************************************************************************
---������void KCT_StartCmd(unsigned char cmd)
---���ܣ�������Ĵ�����д���������ִ��
---���룺	cmd = 0x0, 	COMMAND_IDLE ȡ����ǰִ�е�����
							= 0x1,	COMMAND_CONFIG���洢25�ֽڵ��ڲ�Buffer
							= 0x2��	COMMAND_RANDOMIDS������10�ֽ������
							=	0x3,	COMMAND_CALCCRC������CRCЭ��������ִ���Բ���
							=	0x4,	COMMAND_TRANSMIT�����ͻ���������
							=	0x7,	COMMAND_NOCMDCHANGE�������޸�����Ĵ����Ĳ�ͬλ����������������
							=	0x8,	COMMAND_RECEIVE��������ջ�
							=	0xC,	COMMAND_TRANSCEIVE���������ͣ�����������Ͻ���
							= 0xE,	����MF����׼��֤����
							=	0xF,	�����λRC522
---�����
---��ע��
*************************************************/
void KCT_StartCmd(unsigned char cmd);

/*******************************************************************************
---������void KCT_StopCmd(void)
---���ܣ�ȡ����ǰִ�е�����
---���룺��
---�������
---��ע��
*************************************************/
void KCT_StopCmd(void);

/*******************************************************************************
---������unsigned char KCT_FlagOK(void)
---���ܣ���ȡ�����ǣ��������Ƿ���CRC����/Э�����/��ż����/
---���룺��
---�����0-�޴��� ��0-�д���
---��ע��
*************************************************/
unsigned char KCT_FlagOK(void);

/*******************************************************************************
---������unsigned char KCT_CrcOK(void)
---���ܣ��ж��Ƿ���CRC����
---���룺��
---��������
---��ע��
*************************************************/
unsigned char KCT_CrcOK(void);

/*******************************************************************************
---������void Nz3801_SetTimer(unsigned int fc)
---���ܣ�����NZ3801-AB����Ƶ��ʱʱ�� ��ʽ1
---���룺
---�����
---��ע��domod TPrescalEven bit=0
*************************************************/
void Nz3801_SetTimer(unsigned int fc);

/*******************************************************************************
---������unsigned char Nz3801_SoftReset(void)
---���ܣ�ͨ������������λ
---���룺
---�����
---��ע��
*************************************************/
unsigned char Nz3801_SoftReset(void);

/*******************************************************************************
---������unsigned char Nz3801_SoftPwrDown(unsigned char bEnable)
---���ܣ����ý���PDģʽ���������߻���оƬ
---���룺
---�����
---��ע��
*************************************************/
unsigned char Nz3801_SoftPwrDown(unsigned char bEnable);

/*******************************************************************************
---������unsigned char Nz3801_ActivateField(unsigned char activateField)
---���ܣ����ƿ��س�ǿ
---���룺activateField = NZ3801_TRUE��������ǿ��
											 = NZ3801_FALSE���رճ�ǿ��
---�����
---��ע��
*************************************************/
unsigned char Nz3801_ActivateField(unsigned char activateField);

/*******************************************************************************
---������unsigned char Nz3801_Init(teCardType card)
---���ܣ����������Э�����ͳ�ʼ��Nz3801_�Ĵ���
---���룺
---�����
---��ע��
*************************************************/
unsigned char Nz3801_Init_CPU(teCardType card);

/*******************************************************************************
---������Nz3801_Xfer
---���ܣ��������ݴ���
---���룺txalign  ���1���ֽڷ��Ͷ���bit
				 rxalign  ������ʱ�ӵڼ���bit��ʼ
				 len  �������ݳ��ȣ�Ϊ0ʱֻ�ղ���
---�����
---��ע��
*************************************************/
unsigned char Nz3801_Xfer(eCmd command, const unsigned char *request, 
									unsigned char requestLength, unsigned char txalign,
                  unsigned char *response, unsigned char *responseLength, unsigned char rxalign);


/*******************************************************************************
---������unsigned char iso14443ASendHlta(void)
---���ܣ������Ѽ���Ŀ�Ƭ��������״̬��ͣ��
---���룺none
---�����result
---��ע��
*************************************************/
unsigned char iso14443ASendHlta(void);

/*******************************************************************************
---������unsigned char iso14443ASelect(iso14443ACommand_t cmd, iso14443AProximityCard_t* card)
---���ܣ�A��ѡ��������ִ�����ʼ��֮��Ϳ��Բ���ѡ��
---���룺 cmd = 0x26[ISO14443A_CMD_REQA] - Ѱδ��������״̬�Ŀ�
							= 0x52[ISO14443A_CMD_WUPA] - Ѱ����״̬�Ŀ�
---�����iso14443AProximityCard_t* card��Ѱ���ɹ�ʱ����������������:
				card->atqa  : ������
				card->uid	:		�����к�
				card->sak :		��SAK
---��ע����һ����Ӧ�����ڵ�ͬһʱ�̣�ֻ������һ��TypaA�����ڱ�ѡ��״̬
*************************************************/
unsigned char iso14443ASelect(iso14443ACommand_t cmd, iso14443AProximityCard_t* card);

/*******************************************************************************
---������unsigned char iso14443AEnterProtocolMode(iso14443AProximityCard_t* card, unsigned char* answer, unsigned char* length)
---���ܣ�����RATS������A��������ISO14443-4Ӧ�ò�
---���룺card ->fsdi -PCD֡����󳤶�
				 card ->CID = 0
---�����card -> fsci - PICC֡����󳤶�
				 card -> sfgi - PICCӦ��ATS����һ֡�ı���ʱ��
				 card -> TA -�շ�λ����
				 answer:��Ӧ��ATS����
				 length: ���ص�ATS����
---��ע��
*************************************************/
unsigned char iso14443AEnterProtocolMode(iso14443AProximityCard_t* card, unsigned char* answer, unsigned char* length);

/*******************************************************************************
---������unsigned char iso14443ASendProtocolAndParameterSelection(unsigned char pps1)
---���ܣ�A Э��Ͳ���ѡ����PPS����-4��Э����Ҫ�õ�
---���룺PPS1 - Э�̴���λ����
					PPS1 = 0x0, ѡ��Ĭ��106KBd			PPS1 = 0x05, ѡ��212KBd
					PPS1 = 0x0A, ѡ��424KBd					PPS1 = 0x0F��ѡ��848KBd
---�����result
---��ע��
*************************************************/
unsigned char iso14443ASendProtocolAndParameterSelection(unsigned char pps1);

//==========End of ISO14443A_EN-- Line710 ~ Line770 =====================================================



unsigned char ApduTransceive(unsigned char  *inf,        unsigned int infLength, 
	                           unsigned char  *response,   unsigned int *responseLength);

//-----------ISO14443B ------------------
/*******************************************************************************
---������unsigned char iso14443BSendHltb(iso14443BProximityCard_t* card)
---���ܣ������Ŀ�Ƭ��������״̬��ͣ��
---���룺
---�����
---��ע��
*************************************************/
unsigned char iso14443BSendHltb(iso14443BProximityCard_t* card);

/*******************************************************************************
---������
---���ܣ�TypeB Ѱ��������ײѡ������
---���룺cmd =  0x00[ISO14443B_CMD_REQB] - Ѱδ��������״̬�Ŀ�
						 =  0x08[ISO14443B_CMD_WUPB] - Ѱ����״̬�Ŀ�
				 afi �� Type BӦ����ʶ����������ISO14443Э��
---�����card -> pupi -Type BαΨһPICC����ʶ��
				 card -> applicationData - Ӧ�����ݣ�����֪ͨPCD��PICC��ǰ��װ����ЩӦ��
				 card -> protocolInfo - ָʾ�˿���֧�ֵĲ�������֡��󳤶�/֡�ȴ�ʱ��/λ����
---��ע����һ����Ӧ�����ڵ�ͬһʱ�̣�ֻ����һ��B������ѡ��״̬
*************************************************/
//unsigned char iso14443BSelect(iso14443BCommand_t cmd, iso14443BProximityCard_t* card, unsigned char afi, iso14443BSlotCount_t slotCount);
unsigned char iso14443BSelect(iso14443BCommand_t cmd,
                              iso14443BProximityCard_t* card,
                              unsigned char afi);

/*******************************************************************************
---������iso14443BEnterProtocolMode
---���ܣ�Attrib�������B��������ISO14443-4Ӧ�ò�
---���룺	card -> pupi, TypeBαΨһ PICC����ʶ����ͨ��iso14443BSelect()ѡ����ȡ
					param: baok 4�������������Э��
---�����	answer: AttriBӦ�����ݣ�����MBLI��CID
---��ע��
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




