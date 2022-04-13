#ifndef __CDEMO_H_
#define __CDEMO_H_

extern unsigned char ExteralKey2[24];

unsigned char ExternalAuthenticate(void);
unsigned char GetRandomNumber(unsigned char *buf);
unsigned char InternalCertification(void);
unsigned char ExternalAuthenticate2(void);
unsigned char ExternalAuthenticate(void);
unsigned char SelectMainFile(void);

void getRadom(void);
void Read_BinFile(void);
void des_encryption(unsigned char *input, unsigned char *output, unsigned char *key);

#endif


