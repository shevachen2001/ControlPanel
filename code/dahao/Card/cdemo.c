#include "cdemo.h"
#include "nrf_delay.h"
#include "Nz3801_AB.h"
#include "nrf_drv_spi.h"
#include "des.h"
#include "Access\Access.h"
#include <stdlib.h>
#include <string.h> 
//uint8_t ExteralKey[8]={0x11, 0x22, 0x33, 0x44, 0xAA, 0xBB, 0xCC, 0xDD};

uint8_t ExteralKey[24]={0xB0,0xBE,0xCC,0xA9,0x8F,0x9D,0xCD,0x45,
						0x9A,0xAA,0xE1,0x1A,0xFB,0x03,0xB1,0xE5,
						0xB0,0xBE,0xCC,0xA9,0x8F,0x9D,0xCD,0x45};

uint8_t  ExteralKey2[24]={0x23, 0x45, 0x8D, 0xFA, 0xC5, 0x7B, 0x0C, 0x40,
	                   0x30, 0x4A, 0xCF, 0x89, 0x9B, 0x1E, 0x27, 0x8C,
	                   0x23, 0x45, 0x8D, 0xFA, 0xC5, 0x7B, 0x0C, 0x40,
                         };

uint8_t InternalKey2[24]={0xFF, 0x00, 0x11, 0x22, 0x23, 0x33, 0x44, 0x45,
                     0x56, 0x67, 0x78, 0x89, 0x9A, 0xAB, 0xBC, 0xCD,
	                   0xFF, 0x00, 0x11, 0x22, 0x23, 0x33, 0x44, 0x45,
                     };

uint8_t Txbuf[64]={0};
uint8_t TxLen=0;

uint8_t  Rxbuf[128]={0};
unsigned int  RxLen=0;

uint8_t  radomTab[8]={0};


uint8_t GetRandomNumber(uint8_t *buf)
{
	uint8_t       apdu_getRandom[5]={0x00, 0x84, 0x00, 0x00, 0x04};
	unsigned int  len=0;
	
	ApduTransceive((uint8_t*)apdu_getRandom, 5, buf, &len);

	if(buf[4]==0x90 && buf[5]==0x00)
	{
		return(1);
	}
	else 
	{
		return(0);
	}
		
}

void des_encryption(uint8_t *input, uint8_t *output, uint8_t *key)
{
	 	DES_encrypt(output, input, key);
}

uint8_t ExternalAuthenticate(void)
{
	 uint8_t inbuf[8]={0}, outbuf[8]={0};
	 uint8_t i=0;
	 
	 if( GetRandomNumber(inbuf)==1 )
	 {
		   inbuf[4]=0x00; 
		   inbuf[5]=0x00; 
		   inbuf[6]=0x00; 
		   inbuf[7]=0x00; 
		 
		   des_encryption(inbuf, outbuf, ExteralKey);
		 
		   TxLen=0;
		   Txbuf[TxLen++]=0x00;
		   Txbuf[TxLen++]=0x82;
		   Txbuf[TxLen++]=0x00;
		   Txbuf[TxLen++]=0x00;
		   Txbuf[TxLen++]=0x08;
		 
		   for(i=0; i<8; i++)
		   {
		       Txbuf[TxLen++]=outbuf[i];
		   }
		 
		   ApduTransceive((uint8_t*)Txbuf, TxLen, Rxbuf, &RxLen);
	 }
	 return(1);
}


void getRadom(void)
{
   //ProtoAnaly_UpdateTime();	
   //srand(ProtoAnaly_RtcLocalTime);	
}

uint8_t InternalCertification(void)
{
	 uint8_t tab[5]={0x00, 0x88, 0x00, 0x01, 0x08};
	 uint8_t intab[8];
	 uint8_t outab[8];
	 uint8_t i=0;
	
	 getRadom();
	 
	 for(i=0,TxLen=0; i<5; i++)
	 {
		  Txbuf[TxLen++]=tab[i];
	 }
	 
	 for(i=0; i<8; i++)
	 {
		  Txbuf[TxLen++]=radomTab[i];
	 }
	 
	 RxLen=0;
	 ApduTransceive((uint8_t*)Txbuf, TxLen, Rxbuf, &RxLen);
	 
	 for(i=0; i<8; i++)
	 {
	    intab[i]=Rxbuf[i];
	 }
	 DES_tripleDecrypt(outab, intab, InternalKey2);
	 
	return(0);
}

uint8_t  testtemp[5];
uint8_t ExternalAuthenticate2(void)
{
	 uint8_t inbuf[8]={0}, outbuf[8]={0};
	 uint8_t i=0;
	 
	 if( GetRandomNumber(inbuf)==1 )
	 {
		   inbuf[4]=0x00; 
		   inbuf[5]=0x00; 
		   inbuf[6]=0x00; 
		   inbuf[7]=0x00; 

         //DES_tripleEncrypt(outbuf, inbuf, ExteralKey);
		   DES_tripleEncrypt(outbuf, inbuf, ExteralKey2);
		 
		   TxLen=0;
		   Txbuf[TxLen++]=0x00;
		   Txbuf[TxLen++]=0x82;
		   Txbuf[TxLen++]=0x00;
		   Txbuf[TxLen++]=0x00;
		   Txbuf[TxLen++]=0x08;
		  testtemp[0]  = 0xaa;
		  for(i=0; i<8; i++)
		  {
		     Txbuf[TxLen++]=outbuf[i];
		  }
		 
		  RxLen=0;
		  Rxbuf[0]=0;
		  Rxbuf[1]=0;
      memset(Rxbuf , 0 ,256);
		  
		  memset(testtemp , 0 ,5);

		  ApduTransceive((uint8_t*)Txbuf, TxLen, Rxbuf, &RxLen);

		 testtemp[0]  = 0xaa;

	 }
	 
	 if(Rxbuf[0]==0x90 && Rxbuf[1]==0x00)
	 {
	      testtemp[1] = 0xbb;

		  return(1);
	 }
// 6983  Card locked
	 if(Rxbuf[0]==0x69 && Rxbuf[1]==0x83)
	 {
	    return(CARD_CPU_LOCK_ERR);
	 }
	 testtemp[2] = 0xcc;

	 return(0);
}

uint8_t SelectMainFile(void)
{
	 uint8_t i=0;
	 uint8_t cosComm[7]={0x00, 0xA4, 0x00, 0x00, 0x02, 0x3F, 0x00};
    for(i=0, TxLen=0; i<7; i++)
    {
		  Txbuf[TxLen++]=cosComm[i];
	}
	 
	 RxLen=0;
	 ApduTransceive((uint8_t*)Txbuf, TxLen, Rxbuf, &RxLen);	 

	 if(Rxbuf[RxLen-2]==0x90 && Rxbuf[RxLen-1]==0x00)
	 {
		return(1);
	 }
	 else
	 {
	   	return(0);
	 }
}


void Read_BinFile(void)
{
	 unsigned char i=0;
	
   uint8_t cosComm[5]={0x00, 0xB0, 0x81, 0x00, 0x80};
	 
	 for(i=0, TxLen=0; i<5; i++)
   {
		  Txbuf[TxLen++]=cosComm[i];
	 }
	  
	 RxLen=0;
	 ApduTransceive((uint8_t*)Txbuf, TxLen, Rxbuf, &RxLen);
	 
}


