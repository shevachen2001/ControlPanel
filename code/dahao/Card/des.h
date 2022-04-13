#ifndef __DES_H_
#define __DES_H_

#include <stdbool.h>
#include <stdint.h>

#define tdea_enc tdes_enc
#define tdea_dec tdes_dec


	void DES_encrypt(void* out, const void* in, const void* key);
	void DES_decrypt(void* out, const void* in, const unsigned char* key);
	
	void DES_tripleEncrypt(void* out, void* in, const void* key);
	void DES_tripleDecrypt(void* out, void* in, const unsigned char* key);
	

	void DES_DES(void);

	void DES_init(const void* m_key, unsigned long IVCl);
	void DES_init_1(const void* m_key);
	void DES_change_key(const void* m_key);
	

	void DES_set_IV(unsigned long long int IVCl);
	void DES_get_IV(unsigned char *out);
	

	unsigned long  DES_get_IV_int(void);
	
	void DES_iv_inc(void);
	
	unsigned char* DES_get_key(void);

	int DES_get_size(void);

	void DES_set_size(int sizel);
	void DES_calc_size_n_pad(int p_size);
	void DES_padPlaintext(void* in,unsigned char* out);

	bool DES_CheckPad(unsigned char* in,int size);
	
	void DES_tdesCbcEncipher(unsigned char* in,unsigned char* out);
	void DES_tdesCbcDecipher(unsigned char* in,unsigned char* out);

	//void DES_printArray(byte output[], bool p_pad = true);
	//void DES_printArray(byte output[],int sizel);
	

 void DES_do_3des_encrypt(unsigned char *plain,int size_p,unsigned char *cipher,const void *key, bool inc);
 void DES_do_3des_decrypt(unsigned char *cipher, int size_c, unsigned char *plain, const void *key, unsigned long ivl);

	void DES_permute(const unsigned char *ptable, const unsigned char *in, unsigned char *out);
	void DES_changeendian32(unsigned long * a);
	
	void DES_shiftkey(unsigned char *key);

	void DES_shiftkey_inv(unsigned char *key);
	
	uint64_t DES_splitin6bitwords(uint64_t a);

	unsigned char DES_substitute(unsigned char a, unsigned char * sbp);
	
	unsigned long DES_des_f(unsigned long r, unsigned char* kr);
	

	
unsigned char pgm_read_byte (const unsigned char *abc);

void desExample(void);
void tripledesExample(void);

#endif

