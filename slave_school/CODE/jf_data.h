#ifndef __DATAJF_H__
#define __DATAJF_H__

void seekfree_sendimg_03x2(UARTN_enum uartn, uint8 *image, uint16 width, uint16 height);
void GetHistGram1(uint8_t *image, uint32 len);
uint8_t OSTUThreshold1();
void uart_putbuff2(UARTN_enum uartn, uint8 *buff, uint32 len, uint8 tu);
int regression(uint8 c, uint8 datw[]);

#endif
