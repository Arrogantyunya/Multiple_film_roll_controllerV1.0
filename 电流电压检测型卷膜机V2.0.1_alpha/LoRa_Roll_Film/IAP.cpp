#include "IAP.h"
#include <Arduino.h>

IAP_Fun Jump_Addr;

void MSR_MSP(unsigned int addr)
{
  asm volatile 
  (
    "MSR MSP, r0\n\t"
    "BX r14\n\t" 
  );
}

/*
 @brief   : 检查栈顶地址合法性，跳转到指定的Flash地址运行该地址的程序
            Check the validity of the top address on the stack,
            jump to the specified flash address and run the program
            at that address.
 @para    : object address
 @return  : None
 */
void Jump_OBJ(unsigned int obj_addr)
{
  if (((*(volatile unsigned int *)obj_addr) & 0x2FFE0000) == 0x20000000)
  {
    Jump_Addr = (IAP_Fun)*(volatile unsigned int*)(obj_addr + 4);
    MSR_MSP(*(volatile unsigned int *)obj_addr);
    Jump_Addr();
  }
}