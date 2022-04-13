#ifndef        _HC595_H_
#define        _HC595_H_

#define LED_00_Pin  ((unsigned int)(1<<8))
#define LED_01_Pin  ((unsigned int)(1<<6))
#define LED_02_Pin  ((unsigned int)(1<<1))
#define LED_03_Pin  ((unsigned int)(1<<2))
#define LED_04_Pin  ((unsigned int)(1<<5))
#define LED_05_Pin  ((unsigned int)(1<<4))
#define LED_06_Pin  ((unsigned int)(1<<3))
#define LED_07_Pin  ((unsigned int)(1<<14))
#define LED_08_Pin  ((unsigned int)(1<<9))
#define LED_09_Pin  ((unsigned int)(1<<15))
#define LED_10_Pin  ((unsigned int)(1<<0))
#define LED_11_Pin  ((unsigned int)(1<<10))
#define LED_12_Pin  ((unsigned int)(1<<11))
#define LED_13_Pin  ((unsigned int)(1<<7))
#define LED_14_Pin  ((unsigned int)(1<<13))
#define LED_15_Pin  ((unsigned int)(1<<12))

#define KEY_0 LED_10_Pin
#define KEY_1 LED_05_Pin
#define KEY_2 LED_06_Pin
#define KEY_3 LED_00_Pin
#define KEY_4 LED_04_Pin
#define KEY_5 LED_03_Pin
#define KEY_6 LED_15_Pin
#define KEY_7 LED_01_Pin
#define KEY_8 LED_02_Pin
#define KEY_9 LED_12_Pin

#define KEY_HORIZONTAL_BLUE     LED_07_Pin
#define KEY_HORIZONTAL_GREEN    LED_09_Pin
#define KEY_HORIZONTAL_RED      LED_14_Pin

#define KEY_BELL                LED_08_Pin
#define KEY_HASH                LED_11_Pin
#define KEY_STAR                LED_13_Pin

extern void HC595_Init(void);
extern void HC595_LedWriteData(uint16 Data);
#endif
