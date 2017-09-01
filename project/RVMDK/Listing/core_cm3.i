#line 1 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.c"
 




















 

#line 1 "E:\\keli\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "E:\\keli\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "E:\\keli\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "E:\\keli\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 25 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.c"

 




#line 44 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.c"


 


 







 
__asm uint32_t __get_PSP(void)
{
  mrs r0, psp
  bx lr
}








 
__asm void __set_PSP(uint32_t topOfProcStack)
{
  msr psp, r0
  bx lr
}








 
__asm uint32_t __get_MSP(void)
{
  mrs r0, msp
  bx lr
}








 
__asm void __set_MSP(uint32_t mainStackPointer)
{
  msr msp, r0
  bx lr
}








 
__asm uint32_t __REV16(uint16_t value)
{
  rev16 r0, r0
  bx lr
}








 
__asm int32_t __REVSH(int16_t value)
{
  revsh r0, r0
  bx lr
}


#line 252 "..\\Libraries\\CMSIS\\CM3\\CoreSupport\\core_cm3.c"



