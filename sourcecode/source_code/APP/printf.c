#include"printf.h"


// 发送数据
int fputc(int ch, FILE *f)
{
   USART_SendData(USART1, (unsigned char) ch);// USART1 可以换成 USART2 等
   while (!(USART1->SR & USART_FLAG_TXE));
   return (ch);
}
// 接收数据
int GetKey (void)  
{
   while (!(USART1->SR & USART_FLAG_RXNE));
   return ((int)(USART1->DR & 0x1FF));
}
