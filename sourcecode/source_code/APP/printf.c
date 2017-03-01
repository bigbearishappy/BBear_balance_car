#include"printf.h"


// send data
int fputc(int ch, FILE *f)
{
   USART_SendData(USART1, (unsigned char) ch);// it can be changed to USART2 or USART3 ...
   while (!(USART1->SR & USART_FLAG_TXE));
   return (ch);
}
// receive data
int GetKey (void)  
{
   while (!(USART1->SR & USART_FLAG_RXNE));
   return ((int)(USART1->DR & 0x1FF));
}
