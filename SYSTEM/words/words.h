#ifndef WORDS_H
#define WORDS_H

#include "main.h"


//汉字结构体
typedef struct
{
	uint16_t Index;			// 汉字内码
	uint8_t	 Msk[32];		// 汉字字模

}typFNF_GB16;




extern const uint8_t ASCII_8_16[][16];
extern const typFNF_GB16 CN_16_16[70];






#endif

