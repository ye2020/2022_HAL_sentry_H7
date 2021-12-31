/* - (���Ĺ��������ļ�
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (c) 2016-2018, Armink, <armink.ztl@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: It is the configure head file for this library.
 * Created on: 2016-04-23
 */

#ifndef _SFUD_CFG_H_
#define _SFUD_CFG_H_

//#define SFUD_DEBUG_MODE  //ʹ�ܵ������

/* 
 * ע�⣺�رպ�ֻ���ѯ�ÿ��� /sfud/inc/sfud_flash_def.h ���ṩ�� Flash ��Ϣ����
 * ������Ȼ�ή�������������ԣ������ٴ�������
 */
#define SFUD_USING_SFDP  //�Ƿ�ʹ�� SFDP ��������

/*
 * ע�⣺�رպ�ÿ�ֻ����֧�� SFDP �淶�� Flash��Ҳ���ʵ��Ľ��Ͳ��ִ�������
 * ���� SFUD_USING_SFDP �� SFUD_USING_FLASH_INFO_TABLE �������궨�����ٶ���һ�֣�Ҳ�������ַ�ʽ��ѡ��
 */
#define SFUD_USING_FLASH_INFO_TABLE  //�Ƿ�ʹ�øÿ��Դ��� Flash ������Ϣ��

enum {
    SFUD_W25Q32JVSSIQ_DEVICE_INDEX = 0,  //����ʵ������޸�
};

//Flash�豸��
#define SFUD_FLASH_DEVICE_TABLE                                                \
{                                                                              \
    [SFUD_W25Q32JVSSIQ_DEVICE_INDEX] = {.name = "W25Q32BV", \
																				.spi.name = "SPI1",	}}
	
																			//	.chip = {"W25Q32BV", SFUD_MF_ID_WINBOND, 0x40, 0x16, 4L*1024L*1024L, SFUD_WM_PAGE_256B, 4096, 0x20}},   \
}

//#define SFUD_USING_QSPI  //ʹ��QSPIͨѶģʽ

#endif /* _SFUD_CFG_H_ */