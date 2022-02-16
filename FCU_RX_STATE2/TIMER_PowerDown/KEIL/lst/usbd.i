#line 1 "..\\..\\..\\..\\Library\\StdDriver\\src\\usbd.c"
 







 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 11 "..\\..\\..\\..\\Library\\StdDriver\\src\\usbd.c"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
 
 
 
 




 








 












#line 38 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"


  



    typedef unsigned int size_t;    
#line 54 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"




extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 193 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 209 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 232 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 247 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 270 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 502 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\string.h"



 

#line 12 "..\\..\\..\\..\\Library\\StdDriver\\src\\usbd.c"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NuMicro.h"
 






 



#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
 








 



















 









 




 



 

typedef enum IRQn {
     
    NonMaskableInt_IRQn         = -14,     
    HardFault_IRQn              = -13,     
    SVCall_IRQn                 = -5,      
    PendSV_IRQn                 = -2,      
    SysTick_IRQn                = -1,      

     
    BOD_IRQn                  = 0,         
    WDT_IRQn                  = 1,         
    EINT0_IRQn                = 2,         
    EINT1_IRQn                = 3,         
    GPAB_IRQn                 = 4,         
    GPCDEF_IRQn               = 5,         
    PWMA_IRQn                 = 6,         
    PWMB_IRQn                 = 7,         
    TMR0_IRQn                 = 8,         
    TMR1_IRQn                 = 9,         
    TMR2_IRQn                 = 10,        
    TMR3_IRQn                 = 11,        
    UART02_IRQn               = 12,        
    UART1_IRQn                = 13,        
    SPI0_IRQn                 = 14,        
    SPI1_IRQn                 = 15,        
    I2C0_IRQn                 = 18,        
    I2C1_IRQn                 = 19,        
    USBD_IRQn                 = 23,        
    PDMA_IRQn                 = 26,        
    PWRWU_IRQn                = 28,        
    ADC_IRQn                  = 29,        
    IRC_IRQn                  = 30,        
    RTC_IRQn                  = 31         
} IRQn_Type;






 

 





   


#line 1 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
 




















 













 












 




 


 

 













#line 89 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"


 







#line 114 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
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


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 116 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"
 




















 






 


 



 


 









 







 







 






 








 







 







 









 









 
__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 
__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 



#line 268 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"



#line 619 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmInstr.h"

   

   

#line 117 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"
#line 1 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"
 




















 






 

 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}


#line 260 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"


#line 296 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"


#line 615 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cmFunc.h"

 

   

#line 118 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"








 
#line 143 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

 






 
#line 159 "..\\..\\..\\..\\Library\\CMSIS\\Include\\core_cm0.h"

 










 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[1];                  
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];                  
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];                  
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];                  
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];                    
}  NVIC_Type;

 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
       uint32_t RESERVED0;
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];                   
  volatile uint32_t SHCSR;                    
} SCB_Type;

 















 



























 















 









 






 



 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 








 
 






 

 










 









 

 



 




 

 
 










 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2)));  }  
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (1UL << 2));
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 








   

#line 103 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\system_NUC029xEE.h"
 










 






 
 
 

 




 






extern uint32_t SystemCoreClock;     
extern uint32_t CyclesPerUs;         
extern uint32_t PllClock;            









 
extern void SystemInit(void);










 
extern void SystemCoreClockUpdate(void);





#line 104 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"


#pragma anon_unions



#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\adc_reg.h"
 






 



 



 


 



 


typedef struct
{





















































































































































































































































 

    volatile const  uint32_t ADDR[8];        
    volatile uint32_t ADCR;           
    volatile uint32_t ADCHER;         
    volatile uint32_t ADCMPR[2];      
    volatile uint32_t ADSR;           
    volatile const  uint32_t RESERVE0[3];  
    volatile const  uint32_t ADPDMA;         
    volatile const  uint32_t RESERVE1[3];  
    volatile const  uint32_t ADDR1[4];        

} ADC_T;





 



 









 






























 









 


















 



























 



 









   
   
   




#line 111 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\clk_reg.h"
 






 



 



 


 



 



typedef struct
{

























































































































































































































































































































































































































 

    volatile uint32_t PWRCON;         
    volatile uint32_t AHBCLK;         
    volatile uint32_t APBCLK;         
    volatile uint32_t CLKSTATUS;      
    volatile uint32_t CLKSEL0;        
    volatile uint32_t CLKSEL1;        
    volatile uint32_t CLKDIV;         
    volatile uint32_t CLKSEL2;        
    volatile uint32_t PLLCON;         
    volatile uint32_t FRQDIV;            

} CLK_T;





 


 


































 









 

























































 

























 









 







































 



























 












 





















 











   
   
   




#line 112 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\crc_reg.h"
 






 



 



 


 



 


typedef struct
{








































































































































































 

    volatile uint32_t CTL;            
    volatile uint32_t DMASAR;         
    volatile const  uint32_t RESERVED0;    
    volatile uint32_t DMABCR ;        
    volatile const  uint32_t RESERVED1;    
    volatile const  uint32_t DMACSAR;        
    volatile const  uint32_t RESERVED2;    
    volatile const  uint32_t DMACBCR;        
    volatile uint32_t DMAIER ;        
    volatile uint32_t DMAISR;         
    volatile const  uint32_t RESERVED3[22];
    volatile uint32_t WDATA;          
    volatile uint32_t SEED;           
    volatile const  uint32_t CHECKSUM;       

} CRC_T;





 


 



























 



 



 



 



 






 






 



 



 


   
   

   





#line 113 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\ebi_reg.h"
 






 




 



 


 



 


typedef struct
{































































 

    volatile uint32_t EBICON;         
    volatile uint32_t EXTIME;         
    volatile uint32_t EBICON2;        

} EBI_T;






 

 












 












 








   
   

   






#line 114 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\fmc_reg.h"
 






 



 



 


 



 



typedef struct
{























































































































































 

    volatile uint32_t ISPCON;         
    volatile uint32_t ISPADR;         
    volatile uint32_t ISPDAT;         
    volatile uint32_t ISPCMD;         
    volatile uint32_t ISPTRG;         
    volatile const  uint32_t DFBADR;         
    volatile uint32_t FATCON;         
    volatile const  uint32_t RESERVED[9];  
    volatile uint32_t ISPSTA;         

} FMC_T;





 


 
























 



 



 









 



 



 
























   
   
   




#line 115 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\gpio_reg.h"
 






 



 



 


 



 



typedef struct
{
















































































































































 

    volatile uint32_t PMD;            
    volatile uint32_t OFFD;           
    volatile uint32_t DOUT;           
    volatile uint32_t DMASK;          
    volatile const  uint32_t PIN;            
    volatile uint32_t DBEN;           
    volatile uint32_t IMD;            
    volatile uint32_t IEN;            
    volatile uint32_t ISRC;           

} GPIO_T;





typedef struct
{

































 

    volatile uint32_t DBNCECON;       

} GPIO_DBNCECON_T;





 


 
















































 



 



 



 



 



 



 






 



 








   
   
   



#line 116 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\i2c_reg.h"
 






 



 



 



 



 


typedef struct
{




















































































































































































































 

    volatile uint32_t I2CON;          
    volatile uint32_t I2CADDR0;       
    volatile uint32_t I2CDAT;         
    volatile const  uint32_t I2CSTATUS;      
    volatile uint32_t I2CLK;          
    volatile uint32_t I2CTOC;         
    volatile uint32_t I2CADDR1;       
    volatile uint32_t I2CADDR2;       
    volatile uint32_t I2CADDR3;       
    volatile uint32_t I2CADM0;        
    volatile uint32_t I2CADM1;        
    volatile uint32_t I2CADM2;        
    volatile uint32_t I2CADM3;        
    volatile const  uint32_t RESERVED0[2]; 
    volatile uint32_t I2CWKUPCON;     
    volatile uint32_t I2CWKUPSTS;     

} I2C_T;





 



 


















 






 



 



 



 









 



 



 


   
   
   


#line 117 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\pdma_reg.h"
 






 



 



 




 



 


typedef struct
{







































































































































 

    volatile uint32_t CSR;            
    volatile uint32_t SAR;            
    volatile uint32_t DAR;            
    volatile uint32_t BCR;            
    volatile const  uint32_t POINT;          
    volatile const  uint32_t CSAR;           
    volatile const  uint32_t CDAR;           
    volatile const  uint32_t CBCR;           
    volatile uint32_t IER;            
    volatile uint32_t ISR;            
    volatile const  uint32_t RESERVE[22];  
    volatile const  uint32_t SBUF;           

} PDMA_T;





typedef struct
{


































































































































































































 

    volatile uint32_t GCRCSR;         
    volatile uint32_t PDSSR0;         
    volatile uint32_t PDSSR1;         
    volatile uint32_t GCRISR;         
    volatile uint32_t PDSSR2;         

} PDMA_GCR_T;






 


 





















 



 



 




 






 






 






























 
























 















 

































 





   
   
   

#line 118 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\pwm_reg.h"
 






 



 



 




 



 



typedef struct
{












































































































































































































































































































































































































































































































































































































































































































































































































 

    volatile uint32_t PPR;            
    volatile uint32_t CSR;            
    volatile uint32_t PCR;            
    volatile uint32_t CNR0;           
    volatile uint32_t CMR0;           
    volatile const  uint32_t PDR0;           
    volatile uint32_t CNR1;           
    volatile uint32_t CMR1;           
    volatile const  uint32_t PDR1;           
    volatile uint32_t CNR2;           
    volatile uint32_t CMR2;           
    volatile const  uint32_t PDR2;           
    volatile uint32_t CNR3;           
    volatile uint32_t CMR3;           
    volatile const  uint32_t PDR3;           
    volatile uint32_t PBCR;           
    volatile uint32_t PIER;           
    volatile uint32_t PIIR;           
    volatile const  uint32_t RESERVE1[2];  
    volatile uint32_t CCR0;           
    volatile uint32_t CCR2;           
    volatile uint32_t CRLR0;          
    volatile uint32_t CFLR0;          
    volatile uint32_t CRLR1;          
    volatile uint32_t CFLR1;          
    volatile uint32_t CRLR2;          
    volatile uint32_t CFLR2;          
    volatile uint32_t CRLR3;          
    volatile uint32_t CFLR3;          
    volatile uint32_t CAPENR;         
    volatile uint32_t POE;            
    volatile uint32_t TCON;           
    volatile uint32_t TSTATUS;        
    volatile uint32_t SYNCBUSY0;      
    volatile uint32_t SYNCBUSY1;      
    volatile uint32_t SYNCBUSY2;      
    volatile uint32_t SYNCBUSY3;      

} PWM_T;





 


 












 












 




























































 



 



 



 



 































 
























 










































 










































 



 



 












 












 













 













 



 



 



 


   
   
   


#line 119 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\rtc_reg.h"
 






 




 



 




 



 


typedef struct
{

































































































































































































































 

    volatile uint32_t INIR;           
    volatile uint32_t AER;            
    volatile uint32_t FCR;            
    volatile uint32_t TLR;            
    volatile uint32_t CLR;            
    volatile uint32_t TSSR;           
    volatile uint32_t DWR;            
    volatile uint32_t TAR;            
    volatile uint32_t CAR;            
    volatile const  uint32_t LIR;            
    volatile uint32_t RIER;           
    volatile uint32_t RIIR;           
    volatile uint32_t TTR;            
    volatile const  uint32_t RESERVED[2];  
    volatile uint32_t SPRCTL;         
    volatile uint32_t SPR[20];        

} RTC_T;





 



 






 






 






 


















 


















 



 



 


















 


















 



 









 









 



 

















   
   
   


#line 120 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\spi_reg.h"
 






 



 



 



 



 



typedef struct
{























































































































































































































































































































































































































 

    volatile uint32_t CNTRL;          
    volatile uint32_t DIVIDER;        
    volatile uint32_t SSR;            
    volatile const  uint32_t RESERVE0;     
    volatile const  uint32_t RX;             
    volatile const  uint32_t RESERVE1;     
    volatile const  uint32_t RESERVE2;     
    volatile const  uint32_t RESERVE3;     
    volatile  uint32_t TX;             
    volatile const  uint32_t RESERVE4;     
    volatile const  uint32_t RESERVE5;     
    volatile const  uint32_t RESERVE6;     
    volatile const  uint32_t RESERVE7;     
    volatile uint32_t VARCLK;         
    volatile uint32_t DMA;            
    volatile uint32_t CNTRL2;         
    volatile uint32_t FIFO_CTL;       
    volatile uint32_t STATUS;         

} SPI_T;






 


 



















































 






 















 









 
























 
























 



































   
   
   



#line 121 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\sys_reg.h"
 






 




 



 




 



 



typedef struct
{



































































































































































































































































































































































































































































































































































































































































































































































































































































 

    volatile const  uint32_t PDID;           
    volatile uint32_t RSTSRC;         
    volatile uint32_t IPRSTC1;        
    volatile uint32_t IPRSTC2;        
    volatile uint32_t IPRSTC3;        
    volatile const  uint32_t RESERVE0;     
    volatile uint32_t BODCR;          
    volatile uint32_t TEMPCR;         
    volatile const  uint32_t RESERVE1;     
    volatile uint32_t PORCR;          
    volatile const  uint32_t RESERVE2[2];  
    volatile uint32_t GPA_MFP;        
    volatile uint32_t GPB_MFP;        
    volatile uint32_t GPC_MFP;        
    volatile const  uint32_t RESERVE3;
    volatile uint32_t GPE_MFP;        
    volatile uint32_t GPF_MFP;        
    volatile const  uint32_t RESERVE4[2];  
    volatile uint32_t ALT_MFP;        
    volatile const  uint32_t RESERVE5[2];     
    volatile uint32_t ALT_MFP2;       
    volatile const  uint32_t RESERVE6[8];  
    volatile uint32_t IRCTCTL;        
    volatile uint32_t IRCTIEN;        
    volatile uint32_t IRCTSTS;        
    volatile const  uint32_t RESERVE7[1];
    volatile uint32_t HIRCTCTL;       
    volatile uint32_t HIRCTIEN;       
    volatile uint32_t HIRCTSTS;       
    volatile const  uint32_t RESERVE8[25];
    volatile uint32_t REGWRPROT;      

} GCR_T;





 


 





















 












 
















































 





















 



 



 






 






 






 






 









 







































 















 












 






 









 


















 






 












 





   



typedef struct
{










































































 

    volatile const  uint32_t IRQSRC[32];    
    volatile uint32_t NMISEL;        
    volatile uint32_t MCUIRQ;        
    volatile uint32_t MCUIRQCR;       

} GCR_INT_T;





 


 



 






 



 



   
   
   

#line 122 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\timer_reg.h"
 






 



 



 



 



 


typedef struct
{


















































































































































 

    volatile uint32_t TCSR;           
    volatile uint32_t TCMPR;          
    volatile uint32_t TISR;           
    volatile const  uint32_t TDR;            
    volatile const  uint32_t TCAP;           
    volatile uint32_t TEXCON;         
    volatile uint32_t TEXISR;         

} TIMER_T;





 


 






























 



 






 



 



 





















 


   
   
   


#line 123 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\uart_reg.h"
 






 




 



 


 



 



typedef struct
{
























































































































































































































































































































































































































































































































































































































































 

    union {
        volatile uint32_t DATA;           
        volatile uint32_t THR;            
        volatile uint32_t RBR;            
    };
    volatile uint32_t IER;            
    volatile uint32_t FCR;            
    volatile uint32_t LCR;            
    volatile uint32_t MCR;            
    volatile uint32_t MSR;            
    volatile uint32_t FSR;            
    volatile uint32_t ISR;            
    volatile uint32_t TOR;            
    volatile uint32_t BAUD;           
    volatile uint32_t IRCR;           
    volatile uint32_t ALT_CSR;        
    volatile uint32_t FUN_SEL;        
    volatile uint32_t LIN_CTL;        
    volatile uint32_t LIN_SR;         
    

} UART_T;






 


 



 



 







































 















 


















 









 









 







































 


































































 






 












 









 
























 



 










































 


















   
   
   


#line 124 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\usbd_reg.h"
 






 



 



 



 



 


typedef struct
{










































































 

    volatile uint32_t BUFSEG;         
    volatile uint32_t MXPLD;          
    volatile uint32_t CFG;            
    volatile uint32_t CFGP;           

} USBD_EP_T;






typedef struct
{


















































































































































































































































 

    volatile uint32_t INTEN;          
    volatile uint32_t INTSTS;         
    volatile uint32_t FADDR;          
    volatile const  uint32_t EPSTS;          
    volatile uint32_t ATTR;           
    volatile const  uint32_t FLDET;          
    volatile uint32_t STBUFSEG;       
    volatile const  uint32_t RESERVE1[28];
    volatile const  uint32_t FN;             
    volatile uint32_t DRVSE0;         
    volatile const  uint32_t RESERVE2[283];
    USBD_EP_T     EP[8];          

} USBD_T;





 


 


















 










































 



 



























 






























 



 



 



 



 



 















 






 




   
   
   

#line 125 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\wdt_reg.h"
 






 




 



 



 



 


typedef struct
{


















































































 

    volatile uint32_t WTCR;           
    volatile uint32_t WTCRALT;        

} WDT_T;






 



 






























 


   
   
   



#line 126 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\wwdt_reg.h"
 






 



 



 


 



 


typedef struct
{













































































 

    volatile uint32_t WWDTRLD;        
    volatile uint32_t WWDTCR;         
    volatile uint32_t WWDTSR;         
    volatile const  uint32_t WWDTCVR;        

} WWDT_T;





 



 



 















 






 


   
   
   


#line 127 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"


 
 
 



 
 






 
#line 152 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"




































#line 197 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"









   

 
 
 




 
#line 222 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"


































#line 265 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"










   











 

typedef volatile unsigned char  vu8;        
typedef volatile unsigned short vu16;       
typedef volatile unsigned long  vu32;       





 







 







 








 







 








 







 







 






 








 







 








 







 







 






 


   







 













 
#line 469 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"

 










   

   


 
 
 
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"
 








 










 



 



 


 
 
 
#line 53 "..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"


 
 
 
#line 64 "..\\..\\..\\..\\Library\\StdDriver\\inc\\sys.h"


 
 
 







 




























































































































































































































































































































































































































































































































































































































































































































































   



 







 







 







 







 








 









 







 







 







 











 








 








 








 








 








 








 








 







 







 







 







 














 









 
static __inline void SYS_LockReg(void)
{
    ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0;
}








 
static __inline void SYS_UnlockReg(void)
{
    while(((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT != (1ul << 0))
    {
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x59;
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x16;
        ((GCR_T *) ((( uint32_t)0x50000000) + 0x00000))->REGWRPROT = 0x88;
    }
}


void SYS_ClearResetSrc(uint32_t u32Src);
uint32_t SYS_GetBODStatus(void);
uint32_t SYS_GetResetSrc(void);
uint32_t SYS_IsRegLocked(void);
uint32_t  SYS_ReadPDID(void);
void SYS_ResetChip(void);
void SYS_ResetCPU(void);
void SYS_ResetModule(uint32_t u32ModuleIndex);
void SYS_EnableBOD(int32_t i32Mode, uint32_t u32BODLevel);
void SYS_DisableBOD(void);


   

   

   








 
#line 490 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"
 










 











 



 



 
 
 
 
























 
 
 




 
 
 
#line 76 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"

 
 
 




 
 
 




 
 
 





 
 
 



 
 
 





   



 











 








 







 








 












 












 








 









 










 









 







 
















 
#line 256 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"






 
















 
#line 289 "..\\..\\..\\..\\Library\\StdDriver\\inc\\adc.h"






 










 











 







 








 


void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);



   

   

   







 
#line 491 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"
 










 











 



 



 


 
 
 







 
 
 



 
 
 
#line 61 "..\\..\\..\\..\\Library\\StdDriver\\inc\\fmc.h"


   



 

 
 
 









 












 













 













 














 











 













 












 













 















 














 



 
 
 











 
static __inline void FMC_Write(uint32_t u32addr, uint32_t u32data)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x21;    
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;               
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT = u32data;               
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                   
    __isb(0xF);                             
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                  
}










 
static __inline uint32_t FMC_Read(uint32_t u32addr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x00;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;          
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;              
    __isb(0xF);                        
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);             

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}













 
static __inline int32_t FMC_Erase(uint32_t u32addr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x22;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32addr;                
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                    
    __isb(0xF);                              
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                   

     
    if(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCON & (1ul << 6))
    {
        ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCON |= (1ul << 6);
        return -1;
    }
    return 0;
}










 
static __inline uint32_t FMC_ReadUID(uint8_t u8index)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x04;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = (u8index << 2);       
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                  
    __isb(0xF);                            
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);                 

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}











 
static __inline uint32_t FMC_ReadCID(void)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x0B;            
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = 0x0;                            
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);           
    __isb(0xF);                                      
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0)) ;   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}










 
static __inline uint32_t FMC_ReadPID(void)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x0C;           
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = 0x04;                          
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);          
    __isb(0xF);                                     
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0));   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}










 
static __inline uint32_t FMC_ReadUCID(uint32_t u32Index)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x04;           
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = (0x04 * u32Index) + 0x10;      
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = (1ul << 0);          
    __isb(0xF);                                     
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG & (1ul << 0));   

    return ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPDAT;
}















 
static __inline void FMC_SetVectorPageAddr(uint32_t u32PageAddr)
{
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPCMD = 0x2e;  
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPADR = u32PageAddr;        
    ((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG = 0x1;                
    __isb(0xF);                          
    while(((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPTRG);               
}














 
static __inline uint32_t FMC_GetVECMAP(void)
{
    return (((FMC_T *) ((( uint32_t)0x50000000) + 0x0C000))->ISPSTA & (0xFFFul << 9));
}

extern void FMC_Open(void);
extern void FMC_Close(void);
extern void FMC_EnableAPUpdate(void);
extern void FMC_DisableAPUpdate(void);
extern void FMC_EnableConfigUpdate(void);
extern void FMC_DisableConfigUpdate(void);
extern void FMC_EnableLDUpdate(void);
extern void FMC_DisableLDUpdate(void);
extern int32_t FMC_ReadConfig(uint32_t *u32Config, uint32_t u32Count);
extern int32_t FMC_WriteConfig(uint32_t *u32Config, uint32_t u32Count);
extern void FMC_SetBootSource(int32_t i32BootSrc);
extern int32_t FMC_GetBootSource(void);
extern uint32_t FMC_ReadDataFlashBaseAddr(void);

   

   

   








#line 492 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"
 








 











 



 



 


 
 
 





 
 
 






 
 
 



 
 
 






#line 81 "..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"















 
#line 182 "..\\..\\..\\..\\Library\\StdDriver\\inc\\gpio.h"


   




 












 













 













 













 













 













 













 














 



















 










 











 










 















 














 
















 














 



void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode);
void GPIO_EnableInt(GPIO_T *port, uint32_t u32Pin, uint32_t u32IntAttribs);
void GPIO_DisableInt(GPIO_T *port, uint32_t u32Pin);


   

   

   







 
#line 493 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\i2c.h"
 










 











 



 



 

 
 
 
#line 46 "..\\..\\..\\..\\Library\\StdDriver\\inc\\i2c.h"




   



 









 










 










 










 











 










 











 











 










 


 
 
 









 
static __inline void I2C_STOP(I2C_T *i2c)
{
    (i2c)->I2CON |= ((1ul << 3) | (1ul << 4));
    while((i2c)->I2CON & (1ul << 4));
}

void I2C_ClearTimeoutFlag(I2C_T *i2c);
void I2C_Close(I2C_T *i2c);
void I2C_Trigger(I2C_T *i2c, uint8_t u8Start, uint8_t u8Stop, uint8_t u8Si, uint8_t u8Ack);
void I2C_DisableInt(I2C_T *i2c);
void I2C_EnableInt(I2C_T *i2c);
uint32_t I2C_GetBusClockFreq(I2C_T *i2c);
uint32_t I2C_SetBusClockFreq(I2C_T *i2c, uint32_t u32BusClock);
uint32_t I2C_GetIntFlag(I2C_T *i2c);
uint32_t I2C_GetStatus(I2C_T *i2c);
uint32_t I2C_Open(I2C_T *i2c, uint32_t u32BusClock);
uint8_t I2C_GetData(I2C_T *i2c);
void I2C_SetSlaveAddr(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddr, uint8_t u8GCMode);
void I2C_SetSlaveAddrMask(I2C_T *i2c, uint8_t u8SlaveNo, uint8_t u8SlaveAddrMask);
void I2C_EnableTimeout(I2C_T *i2c, uint8_t u8LongTimeout);
void I2C_DisableTimeout(I2C_T *i2c);
void I2C_EnableWakeup(I2C_T *i2c);
void I2C_DisableWakeup(I2C_T *i2c);
void I2C_SetData(I2C_T *i2c, uint8_t u8Data);

   

   

   

#line 494 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"
 









 











 



 



 
#line 44 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"
 
 
 






   




 












 
#line 81 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"










 











 














 


















 














 















 

















 
#line 195 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pwm.h"


uint32_t PWM_ConfigCaptureChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32UnitTimeNsec,
                                  uint32_t u32CaptureEdge);
uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                 uint32_t u32ChannelNum,
                                 uint32_t u32Frequncy,
                                 uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableCapture(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_DisableCaptureInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void PWM_ClearCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t PWM_GetCaptureIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);



   

   

   







 
#line 495 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\spi.h"
 







 











 



 



 













#line 49 "..\\..\\..\\..\\Library\\StdDriver\\inc\\spi.h"









   




 






 







 







 







 







 








 







 







 







 







 







 







 








 








 








 







 








 









 







 







 







 







 










 







 







 








 








 









 




 
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void SPI_DisableFIFO(SPI_T *spi);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);





 



 



 




 
#line 496 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\crc.h"
 









 











 



 



 
 
 
 





 
 
 





 
 
 




   




 









 










 










 










 













 










 










 



 
void CRC_Open(uint32_t u32Mode, uint32_t u32Attribute, uint32_t u32Seed, uint32_t u32DataLen);
void CRC_StartDMATransfer(uint32_t u32SrcAddr, uint32_t u32ByteCount);
uint32_t CRC_GetChecksum(void);

   

   

   







 
#line 497 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\timer.h"
 









 











 



 



 

#line 44 "..\\..\\..\\..\\Library\\StdDriver\\inc\\timer.h"

   




 










 












 











 










 
static __inline void TIMER_Start(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 30);
}









 
static __inline void TIMER_Stop(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 30);
}










 
static __inline void TIMER_EnableWakeup(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 23);
}









 
static __inline void TIMER_DisableWakeup(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 23);
}









 
static __inline void TIMER_EnableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 6);
}









 
static __inline void TIMER_DisableCaptureDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 6);
}









 
static __inline void TIMER_EnableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 7);
}









 
static __inline void TIMER_DisableEventCounterDebounce(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 7);
}









 
static __inline void TIMER_EnableInt(TIMER_T *timer)
{
    timer->TCSR |= (1ul << 29);
}









 
static __inline void TIMER_DisableInt(TIMER_T *timer)
{
    timer->TCSR &= ~(1ul << 29);
}









 
static __inline void TIMER_EnableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON |= (1ul << 5);
}









 
static __inline void TIMER_DisableCaptureInt(TIMER_T *timer)
{
    timer->TEXCON &= ~(1ul << 5);
}










 
static __inline uint32_t TIMER_GetIntFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 0) ? 1 : 0);
}









 
static __inline void TIMER_ClearIntFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 0);
}










 
static __inline uint32_t TIMER_GetCaptureIntFlag(TIMER_T *timer)
{
    return timer->TEXISR;
}









 
static __inline void TIMER_ClearCaptureIntFlag(TIMER_T *timer)
{
    timer->TEXISR = (1ul << 0);
}










 
static __inline uint32_t TIMER_GetWakeupFlag(TIMER_T *timer)
{
    return (timer->TISR & (1ul << 1) ? 1 : 0);
}









 
static __inline void TIMER_ClearWakeupFlag(TIMER_T *timer)
{
    timer->TISR = (1ul << 1);
}









 
static __inline uint32_t TIMER_GetCaptureData(TIMER_T *timer)
{
    return timer->TCAP;
}









 
static __inline uint32_t TIMER_GetCounter(TIMER_T *timer)
{
    return timer->TDR;
}

uint32_t TIMER_Open(TIMER_T *timer, uint32_t u32Mode, uint32_t u32Freq);
void TIMER_Close(TIMER_T *timer);
void TIMER_Delay(TIMER_T *timer, uint32_t u32Usec);
void TIMER_EnableCapture(TIMER_T *timer, uint32_t u32CapMode, uint32_t u32Edge);
void TIMER_DisableCapture(TIMER_T *timer);
void TIMER_EnableEventCounter(TIMER_T *timer, uint32_t u32Edge);
void TIMER_DisableEventCounter(TIMER_T *timer);
uint32_t TIMER_GetModuleClock(TIMER_T *timer);

   

   

   







 
#line 498 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\wdt.h"
 









 











 



 



 
 
 
 
#line 43 "..\\..\\..\\..\\Library\\StdDriver\\inc\\wdt.h"

 
 
 





   




 









 










 










 











 











 











 












 










 
static __inline void WDT_Close(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR = 0;
    return;
}









 
static __inline void WDT_EnableInt(void)
{
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR |= (1ul << 6);
    return;
}









 
static __inline void WDT_DisableInt(void)
{
    
    ((WDT_T *) ((( uint32_t)0x40000000) + 0x4000))->WTCR &= ~((1ul << 6) | (1ul << 2) | (1ul << 3) | (1ul << 5));
    return;
}

void WDT_Open(uint32_t u32TimeoutInterval, uint32_t u32ResetDelay, uint32_t u32EnableReset, uint32_t u32EnableWakeup);

   

   

   







 
#line 499 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\wwdt.h"
 









 











 



 



 
 
 
 
#line 51 "..\\..\\..\\..\\Library\\StdDriver\\inc\\wwdt.h"



   




 









 










 











 











 










 












 


void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt);

   

   

   







 
#line 500 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"
 









 











 



 



 















#line 55 "..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"

#line 63 "..\\..\\..\\..\\Library\\StdDriver\\inc\\rtc.h"







   




 


 
typedef struct
{
    uint32_t u32Year;            
    uint32_t u32Month;           
    uint32_t u32Day;             
    uint32_t u32DayOfWeek;       
    uint32_t u32Hour;            
    uint32_t u32Minute;          
    uint32_t u32Second;          
    uint32_t u32TimeScale;       
    uint32_t u32AmPm;            
} S_RTC_TIME_DATA_T;

   




 










 










 










 











 











 










 











 












 













 










 
static __inline void RTC_WaitAccessEnable(void)
{
     
    while((((RTC_T *) ((( uint32_t)0x40000000) + 0x08000))->AER & (1ul << 16)) == (1ul << 16));
    ((RTC_T *) ((( uint32_t)0x40000000) + 0x08000))->AER = 0x0000A965UL;

     
    while((((RTC_T *) ((( uint32_t)0x40000000) + 0x08000))->AER & (1ul << 16)) == 0x0);
}

void RTC_Open(S_RTC_TIME_DATA_T *sPt);
void RTC_Close(void);
void RTC_32KCalibration(int32_t i32FrequencyX100);
void RTC_GetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_GetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetAlarmDateAndTime(S_RTC_TIME_DATA_T *sPt);
void RTC_SetDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day, uint32_t u32DayOfWeek);
void RTC_SetTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
void RTC_SetAlarmDate(uint32_t u32Year, uint32_t u32Month, uint32_t u32Day);
void RTC_SetAlarmTime(uint32_t u32Hour, uint32_t u32Minute, uint32_t u32Second, uint32_t u32TimeMode, uint32_t u32AmPm);
uint32_t RTC_GetDayOfWeek(void);
void RTC_SetTickPeriod(uint32_t u32TickSelection);
void RTC_EnableInt(uint32_t u32IntFlagMask);
void RTC_DisableInt(uint32_t u32IntFlagMask);
void RTC_EnableSpareRegister(void);
void RTC_DisableSpareRegister(void);
void RTC_EnableSnooperDetection(uint32_t u32PinCondition);
void RTC_DisableSnooperDetection(void);

   

   

   







 
#line 501 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\uart.h"
 








 












 



 



 

 
 
 





 
 
 











 
 
 
















 
 
 



 
 
 



 
 
 





 
 
 
#line 113 "..\\..\\..\\..\\Library\\StdDriver\\inc\\uart.h"

 
 
 




   




 











 











 












 










 












 












 











 











 











 












 











 












 












 




















 



















 



































 











 
static __inline void UART_CLEAR_RTS(UART_T* uart)
{
    (uart)->MCR |= (1ul << 9);
    (uart)->MCR &= ~(1ul << 1);
}









 
static __inline void UART_SET_RTS(UART_T* uart)
{
    (uart)->MCR |= (1ul << 9) | (1ul << 1);
}










 











 



void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart);
void UART_DisableFlowCtrl(UART_T* uart);
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_EnableFlowCtrl(UART_T* uart);
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag);
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr);
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength);
uint32_t UART_Write(UART_T* uart, uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


   

   

   







 

#line 502 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"
 







 







 



 



 


typedef struct s_usbd_info
{
    const uint8_t *gu8DevDesc;             
    const uint8_t *gu8ConfigDesc;          
    const uint8_t **gu8StringDesc;         
    const uint8_t **gu8HidReportDesc;      
    const uint32_t *gu32HidReportSize;     
    const uint32_t *gu32ConfigHidDescIdx;  

} S_USBD_INFO_T;

extern const S_USBD_INFO_T gsInfo;

   





 







#line 63 "..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"


 




 
#line 82 "..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 
#line 91 "..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 



 
#line 103 "..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"

 







 



 
 
 














#line 146 "..\\..\\..\\..\\Library\\StdDriver\\inc\\usbd.h"















   





 










 












 












 











 











 











 











 











 











 











 














 











 














 











 















 












 











 












 












 













 











 













 













 











 










 










 












 















 
static __inline void USBD_MemCopy(uint8_t *dest, uint8_t *src, int32_t size)
{
    while(size--) *dest++ = *src++;
}











 
static __inline void USBD_SetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for(i = 0; i < 8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg | (1ul << 1));
            break;
        }
    }
}









 
static __inline void USBD_ClearStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for(i = 0; i < 8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFGP;  
            u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

            *((volatile uint32_t *)(u32CfgAddr)) = (u32Cfg & ~(1ul << 1));
            break;
        }
    }
}











 
static __inline uint32_t USBD_GetStall(uint8_t epnum)
{
    uint32_t u32CfgAddr;
    uint32_t u32Cfg;
    int i;

    for(i = 0; i < 8; i++)
    {
        u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFG;  
        u32Cfg = *((volatile uint32_t *)(u32CfgAddr));

        if((u32Cfg & 0xf) == epnum)
        {
            u32CfgAddr = (uint32_t)(i << 4) + (uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0].CFGP;  
            break;
        }
    }

    return ((*((volatile uint32_t *)(u32CfgAddr))) & (1ul << 1));
}


extern volatile uint8_t g_usbd_RemoteWakeupEn;
typedef void (*VENDOR_REQ)(void);            
typedef void (*CLASS_REQ)(void);             
typedef void (*SET_INTERFACE_REQ)(void);     
typedef void (*SET_CONFIG_CB)(void);        

 
void USBD_Open(const S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface);
void USBD_Start(void);
void USBD_GetSetupPacket(uint8_t *buf);
void USBD_ProcessSetupPacket(void);
void USBD_StandardRequest(void);
void USBD_PrepareCtrlIn(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlIn(void);
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size);
void USBD_CtrlOut(void);
void USBD_SwReset(void);
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq);
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback);
void USBD_LockEpStall(uint32_t u32EpBitmap);

   

   

   




 
#line 503 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"
 










 






 



 



 

 
 
 




 
 
 





 
 
 
#line 63 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"


   



 









 










 











 











 











 











 












 
#line 160 "..\\..\\..\\..\\Library\\StdDriver\\inc\\pdma.h"









 


void PDMA_Open(uint32_t u32Mask);
void PDMA_Close(void);
void PDMA_SetTransferCnt(uint32_t u32Ch, uint32_t u32Width, uint32_t u32TransCount);
void PDMA_SetTransferAddr(uint32_t u32Ch, uint32_t u32SrcAddr, uint32_t u32SrcCtrl, uint32_t u32DstAddr, uint32_t u32DstCtrl);
void PDMA_SetTransferMode(uint32_t u32Ch, uint32_t u32Periphral, uint32_t u32ScatterEn, uint32_t u32DescAddr);
void PDMA_Trigger(uint32_t u32Ch);
void PDMA_EnableInt(uint32_t u32Ch, uint32_t u32Mask);
void PDMA_DisableInt(uint32_t u32Ch, uint32_t u32Mask);




 



 



 

#line 504 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"
 








 










 



 



 








 
 
 






#line 53 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"




 
 
 














#line 98 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"





#line 113 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 124 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

 
 
 





#line 143 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"
                                                                               
#line 154 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 165 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 176 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"








 
 
 






 
 
 










#line 216 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 223 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"


 
 
 

 

#line 241 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"

#line 250 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"


#line 324 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"


#line 341 "..\\..\\..\\..\\Library\\StdDriver\\inc\\clk.h"


   




 







 
static __inline uint32_t CLK_GetPLLClockFreq(void)
{
    uint32_t u32PllFreq = 0, u32PllReg;
    uint32_t u32FIN, u32NF, u32NR, u32NO;
    uint8_t au8NoTbl[4] = {1, 2, 2, 4};

    u32PllReg = ((CLK_T *) ((( uint32_t)0x50000000) + 0x00200))->PLLCON;

    if(u32PllReg & ((1ul << 16) | (1ul << 18)))
        return 0;            

    if(u32PllReg & 0x00080000UL)
        u32FIN = (22118400UL);     
    else
        u32FIN = (12000000UL);      

    if(u32PllReg & (1ul << 17))
        return u32FIN;       

     
    u32NO = au8NoTbl[((u32PllReg & (3ul << 14)) >> 14)];
    u32NF = ((u32PllReg & (0x1FFul << 0)) >> 0) + 2;
    u32NR = ((u32PllReg & (0x1Ful << 9)) >> 9) + 2;

     
    u32PllFreq = (((u32FIN >> 2) * u32NF) / (u32NR * u32NO) << 2);

    return u32PllFreq;
}









 
static __inline void CLK_SysTickDelay(uint32_t us)
{
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = us * CyclesPerUs;
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x00);
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) | (1UL << 0);

     
    while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16)) == 0);

     
    ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0;
}








 

static __inline void CLK_SysTickLongDelay(uint32_t us)
{
    uint32_t delay;
        
     
    delay = 233016UL;

    do
    {
        if(us > delay)
        {
            us -= delay;
        }
        else
        {
            delay = us;
            us = 0UL;
        }        
        
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = delay * CyclesPerUs;
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL  = (0x0UL);
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) | (1UL << 0);

         
        while((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16)) == 0UL);

         
        ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = 0UL;
    
    }while(us > 0UL);
    
}

void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);


   

   

   







 
#line 505 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"
#line 1 "..\\..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"
 










 











 



 



 
 
 
 



 
 
 



 
 
 
#line 54 "..\\..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"

#line 62 "..\\..\\..\\..\\Library\\StdDriver\\inc\\ebi.h"

   




 









 











 










 











 










 











 


void EBI_Open(uint32_t u32Bank, uint32_t u32DataWidth, uint32_t u32TimingClass, uint32_t u32BusMode, uint32_t u32CSActiveLevel);
void EBI_Close(uint32_t u32Bank);
void EBI_SetBusTiming(uint32_t u32Bank, uint32_t u32TimingConfig, uint32_t u32MclkDiv);

   

   

   







 
#line 506 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NUC029xEE.h"


   


#line 13 "..\\..\\..\\..\\Library\\Device\\Nuvoton\\NUC029xEE\\Include\\NuMicro.h"



 

#line 13 "..\\..\\..\\..\\Library\\StdDriver\\src\\usbd.c"

#line 23 "..\\..\\..\\..\\Library\\StdDriver\\src\\usbd.c"



 



 




 

 
uint8_t g_usbd_SetupPacket[8] = {0};         
volatile uint8_t g_usbd_RemoteWakeupEn = 0;  

static volatile uint8_t *g_usbd_CtrlInPointer = 0;
static volatile uint32_t g_usbd_CtrlInSize = 0;
static volatile uint8_t *g_usbd_CtrlOutPointer = 0;
static volatile uint32_t g_usbd_CtrlOutSize = 0;
static volatile uint32_t g_usbd_CtrlOutSizeLimit = 0;
static volatile uint32_t g_usbd_UsbAddr = 0;
static volatile uint32_t g_usbd_UsbConfig = 0;
static volatile uint32_t g_usbd_CtrlMaxPktSize = 8;
static volatile uint32_t g_usbd_UsbAltInterface = 0;
static volatile uint32_t g_usbd_CtrlOutToggle = 0;
static volatile uint8_t  g_usbd_CtrlInZeroFlag = 0;

const S_USBD_INFO_T *g_usbd_sInfo;                   

VENDOR_REQ g_usbd_pfnVendorRequest = 0;           
CLASS_REQ g_usbd_pfnClassRequest = 0;             
SET_INTERFACE_REQ g_usbd_pfnSetInterface = 0;     
SET_CONFIG_CB g_usbd_pfnSetConfigCallback = 0;    
uint32_t g_u32EpStallLock                = 0;        











 
void USBD_Open(const S_USBD_INFO_T *param, CLASS_REQ pfnClassReq, SET_INTERFACE_REQ pfnSetInterface)
{
    g_usbd_sInfo = param;
    g_usbd_pfnClassRequest = pfnClassReq;
    g_usbd_pfnSetInterface = pfnSetInterface;

     
    g_usbd_CtrlMaxPktSize = g_usbd_sInfo->gu8DevDesc[7];

     
    ((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->ATTR = 0x7D0;
     
    ((uint32_t)(((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->DRVSE0 |= (1ul << 0)));
}









 
void USBD_Start(void)
{
    CLK_SysTickDelay(100000);
     
    ((uint32_t)(((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->DRVSE0 &= ~(1ul << 0)));

     
    (((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->INTSTS = ((1ul << 0) | (1ul << 1) | (1ul << 2) | ((1ul << 3) | (1ul << 8))));

     
    (((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->INTEN |= ((1ul << 0) | (1ul << 1) | (1ul << 2) | ((1ul << 3) | (1ul << 8))));
}










 
void USBD_GetSetupPacket(uint8_t *buf)
{
    USBD_MemCopy(buf, g_usbd_SetupPacket, 8);
}










 
void USBD_ProcessSetupPacket(void)
{
    g_usbd_CtrlOutToggle = 0;
     
    USBD_MemCopy(g_usbd_SetupPacket, (uint8_t *)(((( uint32_t)0x40000000) + 0x60000)+0x100), 8);

     
    switch(g_usbd_SetupPacket[0] & 0x60)
    {
        case 0x00:   
        {
            USBD_StandardRequest();
            break;
        }
        case 0x20:   
        {
            if(g_usbd_pfnClassRequest != 0)
            {
                g_usbd_pfnClassRequest();
            }
            break;
        }
        case 0x40:   
        {
            if(g_usbd_pfnVendorRequest != 0)
            {
                g_usbd_pfnVendorRequest();
            }
            break;
        }
        default:   
        {
             
            (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((0) << 4))) |= (1ul << 1));
            (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((1) << 4))) |= (1ul << 1));
            break;
        }
    }
}










 
void USBD_GetDescriptor(void)
{
    uint32_t u32Len;

    g_usbd_CtrlInZeroFlag = (uint8_t)0;
    u32Len = 0;
    u32Len = g_usbd_SetupPacket[7];
    u32Len <<= 8;
    u32Len += g_usbd_SetupPacket[6];

    switch(g_usbd_SetupPacket[3])
    {
        
        case 0x01:
        {
            u32Len = ((u32Len)<(18) ? (u32Len) : (18));
            ;
            USBD_PrepareCtrlIn((uint8_t *)g_usbd_sInfo->gu8DevDesc, u32Len);
            USBD_PrepareCtrlOut(0, 0);
            break;
        }
        
        case 0x02:
        {
            uint32_t u32TotalLen;

            u32TotalLen = g_usbd_sInfo->gu8ConfigDesc[3];
            u32TotalLen = g_usbd_sInfo->gu8ConfigDesc[2] + (u32TotalLen << 8);

            if(u32Len > u32TotalLen)
            {
                u32Len = u32TotalLen;
                if((u32Len % g_usbd_CtrlMaxPktSize) == 0)
                {
                    g_usbd_CtrlInZeroFlag = (uint8_t)1;
                }
            }
            USBD_PrepareCtrlIn((uint8_t *)g_usbd_sInfo->gu8ConfigDesc, u32Len);
            USBD_PrepareCtrlOut(0, 0);
            break;
        }
        
        case 0x21:
        {
            
 
            uint32_t u32ConfigDescOffset;   
            u32Len = ((u32Len)<(9) ? (u32Len) : (9));
            ;
            u32ConfigDescOffset = g_usbd_sInfo->gu32ConfigHidDescIdx[g_usbd_SetupPacket[4]];
            USBD_PrepareCtrlIn((uint8_t *)&g_usbd_sInfo->gu8ConfigDesc[u32ConfigDescOffset], u32Len);
            USBD_PrepareCtrlOut(0, 0);
            break;
        }
        
        case 0x22:
        {
            if(u32Len > g_usbd_sInfo->gu32HidReportSize[g_usbd_SetupPacket[4]])
            {
                u32Len = g_usbd_sInfo->gu32HidReportSize[g_usbd_SetupPacket[4]];
                if((u32Len % g_usbd_CtrlMaxPktSize) == 0)
                {
                    g_usbd_CtrlInZeroFlag = (uint8_t)1;
                }
            }
            USBD_PrepareCtrlIn((uint8_t *)g_usbd_sInfo->gu8HidReportDesc[g_usbd_SetupPacket[4]], u32Len);
            USBD_PrepareCtrlOut(0, 0);
            break;
        }
        
        case 0x03:
        {
            
            if(g_usbd_SetupPacket[2] < 4)
            {
                if(u32Len > g_usbd_sInfo->gu8StringDesc[g_usbd_SetupPacket[2]][0])
                {
                    u32Len = g_usbd_sInfo->gu8StringDesc[g_usbd_SetupPacket[2]][0];
                    if((u32Len % g_usbd_CtrlMaxPktSize) == 0)
                    {
                        g_usbd_CtrlInZeroFlag = (uint8_t)1;
                    }
                }
                USBD_PrepareCtrlIn((uint8_t *)g_usbd_sInfo->gu8StringDesc[g_usbd_SetupPacket[2]], u32Len);
                USBD_PrepareCtrlOut(0, 0);
                break;
            }
            else
            {
                
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((0) << 4))) |= (1ul << 1));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((1) << 4))) |= (1ul << 1));
                ;
                break;
            }
        }
        default:
            
            (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((0) << 4))) |= (1ul << 1));
            (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((1) << 4))) |= (1ul << 1));
            ;
            break;
    }
}










 
void USBD_StandardRequest(void)
{
     
    g_usbd_CtrlInPointer = 0;
    g_usbd_CtrlInSize = 0;

    if(g_usbd_SetupPacket[0] & 0x80)     
    {
        
        switch(g_usbd_SetupPacket[1])
        {
            case 0x08:
            {
                
                 
                (*((vu8 *) ((((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4))))))) = g_usbd_UsbConfig;
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (1));
                 
                USBD_PrepareCtrlOut(0, 0);
                ;
                break;
            }
            case 0x06:
            {
                USBD_GetDescriptor();
                 
                USBD_PrepareCtrlOut(0, 0);
                ;
                break;
            }
            case 0x0A:
            {
                
                 
                (*((vu8 *) ((((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4))))))) = g_usbd_UsbAltInterface;
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (1));
                 
                USBD_PrepareCtrlOut(0, 0);
                ;
                break;
            }
            case 0x00:
            {
                
                if(g_usbd_SetupPacket[0] == 0x80)
                {
                    uint8_t u8Tmp;

                    u8Tmp = 0;
                    if(g_usbd_sInfo->gu8ConfigDesc[7] & 0x40) u8Tmp |= 1; 
                    if(g_usbd_sInfo->gu8ConfigDesc[7] & 0x20) u8Tmp |= (g_usbd_RemoteWakeupEn << 1); 

                    (*((vu8 *) ((((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4))))))) = u8Tmp;
                }
                
                else if(g_usbd_SetupPacket[0] == 0x81)
                    (*((vu8 *) ((((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4))))))) = 0;
                
                else if(g_usbd_SetupPacket[0] == 0x82)
                {
                    uint8_t ep = g_usbd_SetupPacket[4] & 0xF;
                    (*((vu8 *) ((((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4))))))) = USBD_GetStall(ep) ? 1 : 0;
                }

                (*((vu8 *) ((((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4)))) + 1))) = 0;
                 
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (2));
                 
                USBD_PrepareCtrlOut(0, 0);
                ;
                break;
            }
            default:
            {
                 
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((0) << 4))) |= (1ul << 1));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((1) << 4))) |= (1ul << 1));
                ;
                break;
            }
        }
    }
    else
    {
        
        switch(g_usbd_SetupPacket[1])
        {
            case 0x01:
            {
                if(g_usbd_SetupPacket[2] == 0x00)
                {
                    int32_t epNum, i;

                    
 
                    epNum = g_usbd_SetupPacket[4] & 0xF;
                    for(i = 0; i < 8; i++)
                    {
                        if(((((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[i].CFG & 0xF) == epNum) && ((g_u32EpStallLock & (1 << i)) == 0))
                        {
                            ((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[i].CFGP &= ~(1ul << 1);
                            ((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[i].CFG &= ~(1ul << 7);
                            ;
                        }
                    }
                }
                else if(g_usbd_SetupPacket[2] == 0x01)
                    g_usbd_RemoteWakeupEn = 0;
                 
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (0));
                ;
                break;
            }
            case 0x05:
            {
                g_usbd_UsbAddr = g_usbd_SetupPacket[2];
                ;

                
                 
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (0));
                break;
            }
            case 0x09:
            {
                g_usbd_UsbConfig = g_usbd_SetupPacket[2];

                if(g_usbd_pfnSetConfigCallback)
                    g_usbd_pfnSetConfigCallback();
                
                 
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (0));
                ;
                break;
            }
            case 0x03:
            {
                if(g_usbd_SetupPacket[2] == 0x00)
                {
                    USBD_SetStall(g_usbd_SetupPacket[4] & 0xF);
                    ;
                }
                else if(g_usbd_SetupPacket[2] == 0x01)
                {
                    g_usbd_RemoteWakeupEn = 1;
                    ;
                }
                 
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (0));
                break;
            }
            case 0x0B:
            {
                g_usbd_UsbAltInterface = g_usbd_SetupPacket[2];
                if(g_usbd_pfnSetInterface != 0)
                    g_usbd_pfnSetInterface();
                 
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (0));
                ;
                break;
            }
            default:
            {
                 
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((0) << 4))) |= (1ul << 1));
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFGP + (uint32_t)((1) << 4))) |= (1ul << 1));
                ;
                break;
            }
        }
    }
}











 
void USBD_PrepareCtrlIn(uint8_t *pu8Buf, uint32_t u32Size)
{
    ;
    if(u32Size > g_usbd_CtrlMaxPktSize)
    {
        
        g_usbd_CtrlInPointer = pu8Buf + g_usbd_CtrlMaxPktSize;
        g_usbd_CtrlInSize = u32Size - g_usbd_CtrlMaxPktSize;
        (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
        USBD_MemCopy((uint8_t *)(((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4)))), pu8Buf, g_usbd_CtrlMaxPktSize);
        (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (g_usbd_CtrlMaxPktSize));
    }
    else
    {
        
        g_usbd_CtrlInPointer = 0;
        g_usbd_CtrlInSize = 0;
        (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. CFG + (uint32_t)((0) << 4))) |= (1ul << 7));
        USBD_MemCopy((uint8_t *)(((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4)))), pu8Buf, u32Size);
        (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (u32Size));
    }
}










 
void USBD_CtrlIn(void)
{
    ;
    if(g_usbd_CtrlInSize)
    {
        
        if(g_usbd_CtrlInSize > g_usbd_CtrlMaxPktSize)
        {
            
            USBD_MemCopy((uint8_t *)(((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4)))), (uint8_t *)g_usbd_CtrlInPointer, g_usbd_CtrlMaxPktSize);
            (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (g_usbd_CtrlMaxPktSize));
            g_usbd_CtrlInPointer += g_usbd_CtrlMaxPktSize;
            g_usbd_CtrlInSize -= g_usbd_CtrlMaxPktSize;
        }
        else
        {
            
            USBD_MemCopy((uint8_t *)(((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((0) << 4)))), (uint8_t *)g_usbd_CtrlInPointer, g_usbd_CtrlInSize);
            (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (g_usbd_CtrlInSize));
            g_usbd_CtrlInPointer = 0;
            g_usbd_CtrlInSize = 0;
        }
    }
    else 
    {
        
        if((g_usbd_SetupPacket[0] == 0x00) && (g_usbd_SetupPacket[1] == 0x05))
        {
            if((((uint32_t)(((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->FADDR)) != g_usbd_UsbAddr) && (((uint32_t)(((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->FADDR)) == 0))
            {
                (((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->FADDR = (g_usbd_UsbAddr));
            }
        }

         
        if(g_usbd_CtrlInZeroFlag)
        {
            (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((0) << 4))) = (0));
            g_usbd_CtrlInZeroFlag = 0;
        }
        ;
    }
}











 
void USBD_PrepareCtrlOut(uint8_t *pu8Buf, uint32_t u32Size)
{
    g_usbd_CtrlOutPointer = pu8Buf;
    g_usbd_CtrlOutSize = 0;
    g_usbd_CtrlOutSizeLimit = u32Size;
    (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((1) << 4))) = (g_usbd_CtrlMaxPktSize));
}










 
void USBD_CtrlOut(void)
{
    uint32_t u32Size;

    ;
    if(g_usbd_CtrlOutToggle != (((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EPSTS & (7ul << 11)))
    {
        g_usbd_CtrlOutToggle = ((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EPSTS & (7ul << 11);
        if(g_usbd_CtrlOutSize < g_usbd_CtrlOutSizeLimit)
        {
            u32Size = ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((1) << 4))));
            USBD_MemCopy((uint8_t *)g_usbd_CtrlOutPointer, (uint8_t *)(((( uint32_t)0x40000000) + 0x60000)+0x100) + ((uint32_t)*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. BUFSEG + (uint32_t)((1) << 4)))), u32Size);
            g_usbd_CtrlOutPointer += u32Size;
            g_usbd_CtrlOutSize += u32Size;

            if(g_usbd_CtrlOutSize < g_usbd_CtrlOutSizeLimit)
                (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((1) << 4))) = (g_usbd_CtrlMaxPktSize));
        }
    }
    else
    {
        (*((volatile uint32_t *) ((uint32_t)&((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[0]. MXPLD + (uint32_t)((1) << 4))) = (g_usbd_CtrlMaxPktSize));
    }
}










 
void USBD_SwReset(void)
{
    int i;

    
    g_usbd_CtrlInPointer = 0;
    g_usbd_CtrlInSize = 0;
    g_usbd_CtrlOutPointer = 0;
    g_usbd_CtrlOutSize = 0;
    g_usbd_CtrlOutSizeLimit = 0;
    g_u32EpStallLock = 0;
    memset(g_usbd_SetupPacket, 0, 8);

     
    for(i=0; i<8; i++)
        ((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->EP[i].CFG &= ~(1ul << 7);

    
    (((USBD_T *) ((( uint32_t)0x40000000) + 0x60000))->FADDR = (0));
}









 
void USBD_SetVendorRequest(VENDOR_REQ pfnVendorReq)
{
    g_usbd_pfnVendorRequest = pfnVendorReq;
}









 
void USBD_SetConfigCallback(SET_CONFIG_CB pfnSetConfigCallback)
{
    g_usbd_pfnSetConfigCallback = pfnSetConfigCallback;
}











 
void USBD_LockEpStall(uint32_t u32EpBitmap)
{
    g_u32EpStallLock = u32EpBitmap;
}
   

   

   





 
