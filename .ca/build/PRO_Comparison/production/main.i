# 1 "main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "/opt/microchip/xc8/v2.50/pic/include/language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "main.c" 2
# 13 "main.c"
#pragma config CPUDIV = NOCLKDIV
#pragma config USBDIV = OFF


#pragma config FOSC = IRC
#pragma config PLLEN = OFF
#pragma config PCLKEN = ON
#pragma config FCMEN = ON
#pragma config IESO = ON


#pragma config PWRTEN = OFF
#pragma config BOREN = SBORDIS
#pragma config BORV = 19


#pragma config WDTEN = OFF
#pragma config WDTPS = 32768


#pragma config HFOFST = ON
#pragma config MCLRE = ON


#pragma config STVREN = ON
#pragma config LVP = OFF
#pragma config BBSIZ = OFF
#pragma config XINST = OFF


#pragma config CP0 = OFF
#pragma config CP1 = OFF


#pragma config CPB = OFF
#pragma config CPD = OFF


#pragma config WRT0 = OFF
#pragma config WRT1 = OFF


#pragma config WRTC = OFF
#pragma config WRTB = OFF
#pragma config WRTD = OFF


#pragma config EBTR0 = OFF
#pragma config EBTR1 = OFF


#pragma config EBTRB = OFF





# 1 "/opt/microchip/xc8/v2.50/pic/include/xc.h" 1 3
# 18 "/opt/microchip/xc8/v2.50/pic/include/xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);



# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/xc8debug.h" 1 3



# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/stdlib.h" 1 3



# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/musl_xc8.h" 1 3
# 5 "/opt/microchip/xc8/v2.50/pic/include/c99/stdlib.h" 2 3





# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/features.h" 1 3
# 11 "/opt/microchip/xc8/v2.50/pic/include/c99/stdlib.h" 2 3
# 21 "/opt/microchip/xc8/v2.50/pic/include/c99/stdlib.h" 3
# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 1 3
# 24 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef long int wchar_t;
# 128 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef unsigned size_t;
# 174 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef __int24 int24_t;
# 210 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef __uint24 uint24_t;
# 22 "/opt/microchip/xc8/v2.50/pic/include/c99/stdlib.h" 2 3

int atoi (const char *);
long atol (const char *);

long long atoll (const char *);

double atof (const char *);


float strtof (const char *restrict, char **restrict);
double strtod (const char *restrict, char **restrict);
long double strtold (const char *restrict, char **restrict);



long strtol (const char *restrict, char **restrict, int);
unsigned long strtoul (const char *restrict, char **restrict, int);

long long strtoll (const char *restrict, char **restrict, int);
unsigned long long strtoull (const char *restrict, char **restrict, int);


unsigned long __strtoxl(const char * s, char ** endptr, int base, char is_signed);

unsigned long long __strtoxll(const char * s, char ** endptr, int base, char is_signed);
# 55 "/opt/microchip/xc8/v2.50/pic/include/c99/stdlib.h" 3
int rand (void);
void srand (unsigned);

void *malloc (size_t);
void *calloc (size_t, size_t);
void *realloc (void *, size_t);
void free (void *);

          void abort (void);
int atexit (void (*) (void));
          void exit (int);
          void _Exit (int);

void *bsearch (const void *, const void *, size_t, size_t, int (*)(const void *, const void *));







__attribute__((nonreentrant)) void qsort (void *, size_t, size_t, int (*)(const void *, const void *));

int abs (int);
long labs (long);

long long llabs (long long);


typedef struct { int quot, rem; } div_t;
typedef struct { long quot, rem; } ldiv_t;

typedef struct { long long quot, rem; } lldiv_t;


div_t div (int, int);
ldiv_t ldiv (long, long);

lldiv_t lldiv (long long, long long);


typedef struct { unsigned int quot, rem; } udiv_t;
typedef struct { unsigned long quot, rem; } uldiv_t;
udiv_t udiv (unsigned int, unsigned int);
uldiv_t uldiv (unsigned long, unsigned long);
# 5 "/opt/microchip/xc8/v2.50/pic/include/c99/xc8debug.h" 2 3







#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 24 "/opt/microchip/xc8/v2.50/pic/include/xc.h" 2 3
# 1 "/opt/microchip/xc8/v2.50/pic/include/builtins.h" 1 3



# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/stdint.h" 1 3
# 26 "/opt/microchip/xc8/v2.50/pic/include/c99/stdint.h" 3
# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 1 3
# 133 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef unsigned __int24 uintptr_t;
# 148 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef __int24 intptr_t;
# 164 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;
# 179 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef long int32_t;





typedef long long int64_t;
# 194 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;
# 215 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 235 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 27 "/opt/microchip/xc8/v2.50/pic/include/c99/stdint.h" 2 3

typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 148 "/opt/microchip/xc8/v2.50/pic/include/c99/stdint.h" 3
# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 149 "/opt/microchip/xc8/v2.50/pic/include/c99/stdint.h" 2 3
# 5 "/opt/microchip/xc8/v2.50/pic/include/builtins.h" 2 3


#pragma intrinsic(__nop)
extern void __nop(void);

#pragma intrinsic(__nopf000)
extern void __nopf000(void);
#pragma intrinsic(__nopffff)
extern void __nopffff(void);
#pragma intrinsic(__nop0000)
extern void __nop0000(void);



#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(uint32_t);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(uint32_t);

#pragma intrinsic(_delay3)
extern __attribute__((nonreentrant)) void _delay3(uint8_t);
# 25 "/opt/microchip/xc8/v2.50/pic/include/xc.h" 2 3








# 1 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18.h" 1 3




# 1 "/opt/microchip/xc8/v2.50/pic/include/htc.h" 1 3



# 1 "/opt/microchip/xc8/v2.50/pic/include/xc.h" 1 3
# 5 "/opt/microchip/xc8/v2.50/pic/include/htc.h" 2 3
# 6 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18.h" 2 3


# 1 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18_chip_select.h" 1 3
# 59 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18_chip_select.h" 3
# 1 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 1 3
# 44 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
# 1 "/opt/microchip/xc8/v2.50/pic/include/__at.h" 1 3
# 45 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 2 3







extern volatile unsigned char UEP0 __attribute__((address(0xF53)));

__asm("UEP0 equ 0F53h");


typedef union {
    struct {
        unsigned EPSTALL :1;
        unsigned EPINEN :1;
        unsigned EPOUTEN :1;
        unsigned EPCONDIS :1;
        unsigned EPHSHK :1;
    };
    struct {
        unsigned EP0STALL :1;
        unsigned EP0INEN :1;
        unsigned EP0OUTEN :1;
        unsigned EP0CONDIS :1;
        unsigned EP0HSHK :1;
    };
    struct {
        unsigned EPSTALL0 :1;
        unsigned EPINEN0 :1;
        unsigned EPOUTEN0 :1;
        unsigned EPCONDIS0 :1;
        unsigned EPHSHK0 :1;
    };
} UEP0bits_t;
extern volatile UEP0bits_t UEP0bits __attribute__((address(0xF53)));
# 160 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEP1 __attribute__((address(0xF54)));

__asm("UEP1 equ 0F54h");


typedef union {
    struct {
        unsigned EPSTALL :1;
        unsigned EPINEN :1;
        unsigned EPOUTEN :1;
        unsigned EPCONDIS :1;
        unsigned EPHSHK :1;
    };
    struct {
        unsigned EP1STALL :1;
        unsigned EP1INEN :1;
        unsigned EP1OUTEN :1;
        unsigned EP1CONDIS :1;
        unsigned EP1HSHK :1;
    };
    struct {
        unsigned EPSTALL1 :1;
        unsigned EPINEN1 :1;
        unsigned EPOUTEN1 :1;
        unsigned EPCONDIS1 :1;
        unsigned EPHSHK1 :1;
    };
} UEP1bits_t;
extern volatile UEP1bits_t UEP1bits __attribute__((address(0xF54)));
# 268 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEP2 __attribute__((address(0xF55)));

__asm("UEP2 equ 0F55h");


typedef union {
    struct {
        unsigned EPSTALL :1;
        unsigned EPINEN :1;
        unsigned EPOUTEN :1;
        unsigned EPCONDIS :1;
        unsigned EPHSHK :1;
    };
    struct {
        unsigned EP2STALL :1;
        unsigned EP2INEN :1;
        unsigned EP2OUTEN :1;
        unsigned EP2CONDIS :1;
        unsigned EP2HSHK :1;
    };
    struct {
        unsigned EPSTALL2 :1;
        unsigned EPINEN2 :1;
        unsigned EPOUTEN2 :1;
        unsigned EPCONDIS2 :1;
        unsigned EPHSHK2 :1;
    };
} UEP2bits_t;
extern volatile UEP2bits_t UEP2bits __attribute__((address(0xF55)));
# 376 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEP3 __attribute__((address(0xF56)));

__asm("UEP3 equ 0F56h");


typedef union {
    struct {
        unsigned EPSTALL :1;
        unsigned EPINEN :1;
        unsigned EPOUTEN :1;
        unsigned EPCONDIS :1;
        unsigned EPHSHK :1;
    };
    struct {
        unsigned EP3STALL :1;
        unsigned EP3INEN :1;
        unsigned EP3OUTEN :1;
        unsigned EP3CONDIS :1;
        unsigned EP3HSHK :1;
    };
    struct {
        unsigned EPSTALL3 :1;
        unsigned EPINEN3 :1;
        unsigned EPOUTEN3 :1;
        unsigned EPCONDIS3 :1;
        unsigned EPHSHK3 :1;
    };
} UEP3bits_t;
extern volatile UEP3bits_t UEP3bits __attribute__((address(0xF56)));
# 484 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEP4 __attribute__((address(0xF57)));

__asm("UEP4 equ 0F57h");


typedef union {
    struct {
        unsigned EPSTALL :1;
        unsigned EPINEN :1;
        unsigned EPOUTEN :1;
        unsigned EPCONDIS :1;
        unsigned EPHSHK :1;
    };
    struct {
        unsigned EP4STALL :1;
        unsigned EP4INEN :1;
        unsigned EP4OUTEN :1;
        unsigned EP4CONDIS :1;
        unsigned EP4HSHK :1;
    };
    struct {
        unsigned EPSTALL4 :1;
        unsigned EPINEN4 :1;
        unsigned EPOUTEN4 :1;
        unsigned EPCONDIS4 :1;
        unsigned EPHSHK4 :1;
    };
} UEP4bits_t;
extern volatile UEP4bits_t UEP4bits __attribute__((address(0xF57)));
# 592 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEP5 __attribute__((address(0xF58)));

__asm("UEP5 equ 0F58h");


typedef union {
    struct {
        unsigned EPSTALL :1;
        unsigned EPINEN :1;
        unsigned EPOUTEN :1;
        unsigned EPCONDIS :1;
        unsigned EPHSHK :1;
    };
    struct {
        unsigned EP5STALL :1;
        unsigned EP5INEN :1;
        unsigned EP5OUTEN :1;
        unsigned EP5CONDIS :1;
        unsigned EP5HSHK :1;
    };
    struct {
        unsigned EPSTALL5 :1;
        unsigned EPINEN5 :1;
        unsigned EPOUTEN5 :1;
        unsigned EPCONDIS5 :1;
        unsigned EPHSHK5 :1;
    };
} UEP5bits_t;
extern volatile UEP5bits_t UEP5bits __attribute__((address(0xF58)));
# 700 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEP6 __attribute__((address(0xF59)));

__asm("UEP6 equ 0F59h");


typedef union {
    struct {
        unsigned EPSTALL :1;
        unsigned EPINEN :1;
        unsigned EPOUTEN :1;
        unsigned EPCONDIS :1;
        unsigned EPHSHK :1;
    };
    struct {
        unsigned EP6STALL :1;
        unsigned EP6INEN :1;
        unsigned EP6OUTEN :1;
        unsigned EP6CONDIS :1;
        unsigned EP6HSHK :1;
    };
    struct {
        unsigned EPSTALL6 :1;
        unsigned EPINEN6 :1;
        unsigned EPOUTEN6 :1;
        unsigned EPCONDIS6 :1;
        unsigned EPHSHK6 :1;
    };
} UEP6bits_t;
extern volatile UEP6bits_t UEP6bits __attribute__((address(0xF59)));
# 808 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEP7 __attribute__((address(0xF5A)));

__asm("UEP7 equ 0F5Ah");


typedef union {
    struct {
        unsigned EPSTALL :1;
        unsigned EPINEN :1;
        unsigned EPOUTEN :1;
        unsigned EPCONDIS :1;
        unsigned EPHSHK :1;
    };
    struct {
        unsigned EP7STALL :1;
        unsigned EP7INEN :1;
        unsigned EP7OUTEN :1;
        unsigned EP7CONDIS :1;
        unsigned EP7HSHK :1;
    };
    struct {
        unsigned EPSTALL7 :1;
        unsigned EPINEN7 :1;
        unsigned EPOUTEN7 :1;
        unsigned EPCONDIS7 :1;
        unsigned EPHSHK7 :1;
    };
} UEP7bits_t;
extern volatile UEP7bits_t UEP7bits __attribute__((address(0xF5A)));
# 916 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEIE __attribute__((address(0xF5B)));

__asm("UEIE equ 0F5Bh");


typedef union {
    struct {
        unsigned PIDEE :1;
        unsigned CRC5EE :1;
        unsigned CRC16EE :1;
        unsigned DFN8EE :1;
        unsigned BTOEE :1;
        unsigned :2;
        unsigned BTSEE :1;
    };
} UEIEbits_t;
extern volatile UEIEbits_t UEIEbits __attribute__((address(0xF5B)));
# 967 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UADDR __attribute__((address(0xF5C)));

__asm("UADDR equ 0F5Ch");


typedef union {
    struct {
        unsigned ADDR :7;
    };
    struct {
        unsigned ADDR0 :1;
        unsigned ADDR1 :1;
        unsigned ADDR2 :1;
        unsigned ADDR3 :1;
        unsigned ADDR4 :1;
        unsigned ADDR5 :1;
        unsigned ADDR6 :1;
    };
} UADDRbits_t;
extern volatile UADDRbits_t UADDRbits __attribute__((address(0xF5C)));
# 1031 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UFRML __attribute__((address(0xF5D)));

__asm("UFRML equ 0F5Dh");


typedef union {
    struct {
        unsigned FRML :8;
    };
    struct {
        unsigned FRM0 :1;
        unsigned FRM1 :1;
        unsigned FRM2 :1;
        unsigned FRM3 :1;
        unsigned FRM4 :1;
        unsigned FRM5 :1;
        unsigned FRM6 :1;
        unsigned FRM7 :1;
    };
} UFRMLbits_t;
extern volatile UFRMLbits_t UFRMLbits __attribute__((address(0xF5D)));
# 1101 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UFRMH __attribute__((address(0xF5E)));

__asm("UFRMH equ 0F5Eh");


typedef union {
    struct {
        unsigned FRMH :3;
    };
    struct {
        unsigned FRM8 :1;
        unsigned FRM9 :1;
        unsigned FRM10 :1;
    };
} UFRMHbits_t;
extern volatile UFRMHbits_t UFRMHbits __attribute__((address(0xF5E)));
# 1141 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UEIR __attribute__((address(0xF5F)));

__asm("UEIR equ 0F5Fh");


typedef union {
    struct {
        unsigned PIDEF :1;
        unsigned CRC5EF :1;
        unsigned CRC16EF :1;
        unsigned DFN8EF :1;
        unsigned BTOEF :1;
        unsigned :2;
        unsigned BTSEF :1;
    };
} UEIRbits_t;
extern volatile UEIRbits_t UEIRbits __attribute__((address(0xF5F)));
# 1192 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UIE __attribute__((address(0xF60)));

__asm("UIE equ 0F60h");


typedef union {
    struct {
        unsigned URSTIE :1;
        unsigned UERRIE :1;
        unsigned ACTVIE :1;
        unsigned TRNIE :1;
        unsigned IDLEIE :1;
        unsigned STALLIE :1;
        unsigned SOFIE :1;
    };
} UIEbits_t;
extern volatile UIEbits_t UIEbits __attribute__((address(0xF60)));
# 1248 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UCFG __attribute__((address(0xF61)));

__asm("UCFG equ 0F61h");


typedef union {
    struct {
        unsigned PPB0 :1;
        unsigned PPB1 :1;
        unsigned FSEN :1;
        unsigned :1;
        unsigned UPUEN :1;
        unsigned :2;
        unsigned UTEYE :1;
    };
    struct {
        unsigned UPP0 :1;
        unsigned UPP1 :1;
    };
} UCFGbits_t;
extern volatile UCFGbits_t UCFGbits __attribute__((address(0xF61)));
# 1308 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UIR __attribute__((address(0xF62)));

__asm("UIR equ 0F62h");


typedef union {
    struct {
        unsigned URSTIF :1;
        unsigned UERRIF :1;
        unsigned ACTVIF :1;
        unsigned TRNIF :1;
        unsigned IDLEIF :1;
        unsigned STALLIF :1;
        unsigned SOFIF :1;
    };
} UIRbits_t;
extern volatile UIRbits_t UIRbits __attribute__((address(0xF62)));
# 1364 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char USTAT __attribute__((address(0xF63)));

__asm("USTAT equ 0F63h");


typedef union {
    struct {
        unsigned :1;
        unsigned PPBI :1;
        unsigned DIR :1;
        unsigned ENDP0 :1;
        unsigned ENDP1 :1;
        unsigned ENDP2 :1;
        unsigned ENDP3 :1;
    };
} USTATbits_t;
extern volatile USTATbits_t USTATbits __attribute__((address(0xF63)));
# 1415 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char UCON __attribute__((address(0xF64)));

__asm("UCON equ 0F64h");


typedef union {
    struct {
        unsigned :1;
        unsigned SUSPND :1;
        unsigned RESUME :1;
        unsigned USBEN :1;
        unsigned PKTDIS :1;
        unsigned SE0 :1;
        unsigned PPBRST :1;
    };
} UCONbits_t;
extern volatile UCONbits_t UCONbits __attribute__((address(0xF64)));
# 1466 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char SRCON0 __attribute__((address(0xF68)));

__asm("SRCON0 equ 0F68h");


typedef union {
    struct {
        unsigned SRPR :1;
        unsigned SRPS :1;
        unsigned SRNQEN :1;
        unsigned SRQEN :1;
        unsigned SRCLK :3;
        unsigned SRLEN :1;
    };
    struct {
        unsigned :4;
        unsigned SRCLK0 :1;
        unsigned SRCLK1 :1;
        unsigned SRCLK2 :1;
    };
} SRCON0bits_t;
extern volatile SRCON0bits_t SRCON0bits __attribute__((address(0xF68)));
# 1537 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char SRCON1 __attribute__((address(0xF69)));

__asm("SRCON1 equ 0F69h");


typedef union {
    struct {
        unsigned SRRC1E :1;
        unsigned SRRC2E :1;
        unsigned SRRCKE :1;
        unsigned SRRPE :1;
        unsigned SRSC1E :1;
        unsigned SRSC2E :1;
        unsigned SRSCKE :1;
        unsigned SRSPE :1;
    };
} SRCON1bits_t;
extern volatile SRCON1bits_t SRCON1bits __attribute__((address(0xF69)));
# 1599 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char CM2CON0 __attribute__((address(0xF6B)));

__asm("CM2CON0 equ 0F6Bh");


typedef union {
    struct {
        unsigned C2CH :2;
        unsigned C2R :1;
        unsigned C2SP :1;
        unsigned C2POL :1;
        unsigned C2OE :1;
        unsigned C2OUT :1;
        unsigned C2ON :1;
    };
    struct {
        unsigned C2CH0 :1;
        unsigned C2CH1 :1;
    };
} CM2CON0bits_t;
extern volatile CM2CON0bits_t CM2CON0bits __attribute__((address(0xF6B)));
# 1669 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char CM2CON1 __attribute__((address(0xF6C)));

__asm("CM2CON1 equ 0F6Ch");


typedef union {
    struct {
        unsigned C2SYNC :1;
        unsigned C1SYNC :1;
        unsigned C2HYS :1;
        unsigned C1HYS :1;
        unsigned C2RSEL :1;
        unsigned C1RSEL :1;
        unsigned MC2OUT :1;
        unsigned MC1OUT :1;
    };
} CM2CON1bits_t;
extern volatile CM2CON1bits_t CM2CON1bits __attribute__((address(0xF6C)));
# 1731 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char CM1CON0 __attribute__((address(0xF6D)));

__asm("CM1CON0 equ 0F6Dh");


typedef union {
    struct {
        unsigned C1CH :2;
        unsigned C1R :1;
        unsigned C1SP :1;
        unsigned C1POL :1;
        unsigned C1OE :1;
        unsigned C1OUT :1;
        unsigned C1ON :1;
    };
    struct {
        unsigned C1CH0 :1;
        unsigned C1CH1 :1;
    };
} CM1CON0bits_t;
extern volatile CM1CON0bits_t CM1CON0bits __attribute__((address(0xF6D)));
# 1801 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char SSPMSK __attribute__((address(0xF6F)));

__asm("SSPMSK equ 0F6Fh");


extern volatile unsigned char SSPMASK __attribute__((address(0xF6F)));

__asm("SSPMASK equ 0F6Fh");


typedef union {
    struct {
        unsigned MSK :8;
    };
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
} SSPMSKbits_t;
extern volatile SSPMSKbits_t SSPMSKbits __attribute__((address(0xF6F)));
# 1874 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned MSK :8;
    };
    struct {
        unsigned MSK0 :1;
        unsigned MSK1 :1;
        unsigned MSK2 :1;
        unsigned MSK3 :1;
        unsigned MSK4 :1;
        unsigned MSK5 :1;
        unsigned MSK6 :1;
        unsigned MSK7 :1;
    };
} SSPMASKbits_t;
extern volatile SSPMASKbits_t SSPMASKbits __attribute__((address(0xF6F)));
# 1939 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char SLRCON __attribute__((address(0xF76)));

__asm("SLRCON equ 0F76h");


typedef union {
    struct {
        unsigned SLRA :1;
        unsigned SLRB :1;
        unsigned SLRC :1;
    };
} SLRCONbits_t;
extern volatile SLRCONbits_t SLRCONbits __attribute__((address(0xF76)));
# 1971 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char WPUA __attribute__((address(0xF77)));

__asm("WPUA equ 0F77h");


typedef union {
    struct {
        unsigned :3;
        unsigned WPUA3 :1;
        unsigned WPUA4 :1;
        unsigned WPUA5 :1;
    };
} WPUAbits_t;
extern volatile WPUAbits_t WPUAbits __attribute__((address(0xF77)));
# 2004 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char WPUB __attribute__((address(0xF78)));

__asm("WPUB equ 0F78h");


typedef union {
    struct {
        unsigned :4;
        unsigned WPUB4 :1;
        unsigned WPUB5 :1;
        unsigned WPUB6 :1;
        unsigned WPUB7 :1;
    };
} WPUBbits_t;
extern volatile WPUBbits_t WPUBbits __attribute__((address(0xF78)));
# 2043 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char IOCA __attribute__((address(0xF79)));

__asm("IOCA equ 0F79h");


typedef union {
    struct {
        unsigned IOCA0 :1;
        unsigned IOCA1 :1;
        unsigned :1;
        unsigned IOCA3 :1;
        unsigned IOCA4 :1;
        unsigned IOCA5 :1;
    };
} IOCAbits_t;
extern volatile IOCAbits_t IOCAbits __attribute__((address(0xF79)));
# 2088 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char IOCB __attribute__((address(0xF7A)));

__asm("IOCB equ 0F7Ah");


typedef union {
    struct {
        unsigned :4;
        unsigned IOCB4 :1;
        unsigned IOCB5 :1;
        unsigned IOCB6 :1;
        unsigned IOCB7 :1;
    };
} IOCBbits_t;
extern volatile IOCBbits_t IOCBbits __attribute__((address(0xF7A)));
# 2127 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char ANSEL __attribute__((address(0xF7E)));

__asm("ANSEL equ 0F7Eh");


typedef union {
    struct {
        unsigned :3;
        unsigned ANS3 :1;
        unsigned ANS4 :1;
        unsigned ANS5 :1;
        unsigned ANS6 :1;
        unsigned ANS7 :1;
    };
} ANSELbits_t;
extern volatile ANSELbits_t ANSELbits __attribute__((address(0xF7E)));
# 2172 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char ANSELH __attribute__((address(0xF7F)));

__asm("ANSELH equ 0F7Fh");


typedef union {
    struct {
        unsigned ANS8 :1;
        unsigned ANS9 :1;
        unsigned ANS10 :1;
        unsigned ANS11 :1;
    };
} ANSELHbits_t;
extern volatile ANSELHbits_t ANSELHbits __attribute__((address(0xF7F)));
# 2210 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PORTA __attribute__((address(0xF80)));

__asm("PORTA equ 0F80h");


typedef union {
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
    };
    struct {
        unsigned :4;
        unsigned AN3 :1;
    };
    struct {
        unsigned :4;
        unsigned OSC2 :1;
        unsigned OSC1 :1;
    };
    struct {
        unsigned :4;
        unsigned CLKOUT :1;
        unsigned CLKIN :1;
    };
    struct {
        unsigned ULPWUIN :1;
        unsigned :4;
        unsigned LVDIN :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0xF80)));
# 2309 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PORTB __attribute__((address(0xF81)));

__asm("PORTB equ 0F81h");


typedef union {
    struct {
        unsigned :4;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
    struct {
        unsigned :4;
        unsigned SDI :1;
        unsigned RX :1;
        unsigned SCL :1;
        unsigned TX :1;
    };
    struct {
        unsigned :4;
        unsigned SDA :1;
        unsigned DT :1;
        unsigned SCK :1;
        unsigned CK :1;
    };
    struct {
        unsigned :4;
        unsigned AN10 :1;
        unsigned AN11 :1;
    };
} PORTBbits_t;
extern volatile PORTBbits_t PORTBbits __attribute__((address(0xF81)));
# 2417 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PORTC __attribute__((address(0xF82)));

__asm("PORTC equ 0F82h");


typedef union {
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
    struct {
        unsigned AN4 :1;
        unsigned AN5 :1;
        unsigned AN6 :1;
        unsigned AN7 :1;
        unsigned :2;
        unsigned AN8 :1;
        unsigned AN9 :1;
    };
    struct {
        unsigned :6;
        unsigned NOT_SS :1;
    };
    struct {
        unsigned C12INP :1;
        unsigned C12IN1M :1;
        unsigned C12IN2M :1;
        unsigned C12IN3M :1;
        unsigned C12OUT :1;
        unsigned CCP1 :1;
        unsigned nSS :1;
    };
    struct {
        unsigned INT0 :1;
        unsigned INT1 :1;
        unsigned INT2 :1;
        unsigned PGM :1;
        unsigned SRQ :1;
        unsigned T0CKI :1;
        unsigned T13CKI :1;
        unsigned T1OSCO :1;
    };
    struct {
        unsigned VREFP :1;
        unsigned VREFM :1;
        unsigned CVREF :1;
        unsigned :3;
        unsigned T1OSCI :1;
    };
    struct {
        unsigned :2;
        unsigned P1D :1;
        unsigned P1C :1;
        unsigned P1B :1;
        unsigned P1A :1;
        unsigned SS :1;
        unsigned SDO :1;
    };
    struct {
        unsigned :1;
        unsigned CCP2 :1;
        unsigned PA1 :1;
    };
    struct {
        unsigned :1;
        unsigned PA2 :1;
    };
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __attribute__((address(0xF82)));
# 2711 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char LATA __attribute__((address(0xF89)));

__asm("LATA equ 0F89h");


typedef union {
    struct {
        unsigned :4;
        unsigned LATA4 :1;
        unsigned LATA5 :1;
    };
    struct {
        unsigned :4;
        unsigned LA4 :1;
        unsigned LA5 :1;
    };
} LATAbits_t;
extern volatile LATAbits_t LATAbits __attribute__((address(0xF89)));
# 2753 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char LATB __attribute__((address(0xF8A)));

__asm("LATB equ 0F8Ah");


typedef union {
    struct {
        unsigned :4;
        unsigned LATB4 :1;
        unsigned LATB5 :1;
        unsigned LATB6 :1;
        unsigned LATB7 :1;
    };
    struct {
        unsigned :4;
        unsigned LB4 :1;
        unsigned LB5 :1;
        unsigned LB6 :1;
        unsigned LB7 :1;
    };
} LATBbits_t;
extern volatile LATBbits_t LATBbits __attribute__((address(0xF8A)));
# 2819 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char LATC __attribute__((address(0xF8B)));

__asm("LATC equ 0F8Bh");


typedef union {
    struct {
        unsigned LATC0 :1;
        unsigned LATC1 :1;
        unsigned LATC2 :1;
        unsigned LATC3 :1;
        unsigned LATC4 :1;
        unsigned LATC5 :1;
        unsigned LATC6 :1;
        unsigned LATC7 :1;
    };
    struct {
        unsigned LC0 :1;
        unsigned LC1 :1;
        unsigned LC2 :1;
        unsigned LC3 :1;
        unsigned LC4 :1;
        unsigned LC5 :1;
        unsigned LC6 :1;
        unsigned LC7 :1;
    };
} LATCbits_t;
extern volatile LATCbits_t LATCbits __attribute__((address(0xF8B)));
# 2931 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char TRISA __attribute__((address(0xF92)));

__asm("TRISA equ 0F92h");


extern volatile unsigned char DDRA __attribute__((address(0xF92)));

__asm("DDRA equ 0F92h");


typedef union {
    struct {
        unsigned :4;
        unsigned TRISA4 :1;
        unsigned TRISA5 :1;
    };
    struct {
        unsigned :4;
        unsigned RA4 :1;
        unsigned RA5 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0xF92)));
# 2976 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned :4;
        unsigned TRISA4 :1;
        unsigned TRISA5 :1;
    };
    struct {
        unsigned :4;
        unsigned RA4 :1;
        unsigned RA5 :1;
    };
} DDRAbits_t;
extern volatile DDRAbits_t DDRAbits __attribute__((address(0xF92)));
# 3013 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char TRISB __attribute__((address(0xF93)));

__asm("TRISB equ 0F93h");


extern volatile unsigned char DDRB __attribute__((address(0xF93)));

__asm("DDRB equ 0F93h");


typedef union {
    struct {
        unsigned :4;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned TRISB6 :1;
        unsigned TRISB7 :1;
    };
    struct {
        unsigned :4;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
} TRISBbits_t;
extern volatile TRISBbits_t TRISBbits __attribute__((address(0xF93)));
# 3082 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned :4;
        unsigned TRISB4 :1;
        unsigned TRISB5 :1;
        unsigned TRISB6 :1;
        unsigned TRISB7 :1;
    };
    struct {
        unsigned :4;
        unsigned RB4 :1;
        unsigned RB5 :1;
        unsigned RB6 :1;
        unsigned RB7 :1;
    };
} DDRBbits_t;
extern volatile DDRBbits_t DDRBbits __attribute__((address(0xF93)));
# 3143 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char TRISC __attribute__((address(0xF94)));

__asm("TRISC equ 0F94h");


extern volatile unsigned char DDRC __attribute__((address(0xF94)));

__asm("DDRC equ 0F94h");


typedef union {
    struct {
        unsigned TRISC0 :1;
        unsigned TRISC1 :1;
        unsigned TRISC2 :1;
        unsigned TRISC3 :1;
        unsigned TRISC4 :1;
        unsigned TRISC5 :1;
        unsigned TRISC6 :1;
        unsigned TRISC7 :1;
    };
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __attribute__((address(0xF94)));
# 3258 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned TRISC0 :1;
        unsigned TRISC1 :1;
        unsigned TRISC2 :1;
        unsigned TRISC3 :1;
        unsigned TRISC4 :1;
        unsigned TRISC5 :1;
        unsigned TRISC6 :1;
        unsigned TRISC7 :1;
    };
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
        unsigned RC6 :1;
        unsigned RC7 :1;
    };
} DDRCbits_t;
extern volatile DDRCbits_t DDRCbits __attribute__((address(0xF94)));
# 3365 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char OSCTUNE __attribute__((address(0xF9B)));

__asm("OSCTUNE equ 0F9Bh");


typedef union {
    struct {
        unsigned TUN :6;
        unsigned SPLLEN :1;
        unsigned INTSRC :1;
    };
    struct {
        unsigned TUN0 :1;
        unsigned TUN1 :1;
        unsigned TUN2 :1;
        unsigned TUN3 :1;
        unsigned TUN4 :1;
        unsigned TUN5 :1;
    };
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __attribute__((address(0xF9B)));
# 3435 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PIE1 __attribute__((address(0xF9D)));

__asm("PIE1 equ 0F9Dh");


typedef union {
    struct {
        unsigned TMR1IE :1;
        unsigned TMR2IE :1;
        unsigned CCP1IE :1;
        unsigned SSPIE :1;
        unsigned TXIE :1;
        unsigned RCIE :1;
        unsigned ADIE :1;
    };
    struct {
        unsigned :4;
        unsigned TX1IE :1;
        unsigned RC1IE :1;
    };
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __attribute__((address(0xF9D)));
# 3506 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PIR1 __attribute__((address(0xF9E)));

__asm("PIR1 equ 0F9Eh");


typedef union {
    struct {
        unsigned TMR1IF :1;
        unsigned TMR2IF :1;
        unsigned CCP1IF :1;
        unsigned SSPIF :1;
        unsigned TXIF :1;
        unsigned RCIF :1;
        unsigned ADIF :1;
    };
    struct {
        unsigned :4;
        unsigned TX1IF :1;
        unsigned RC1IF :1;
    };
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __attribute__((address(0xF9E)));
# 3577 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char IPR1 __attribute__((address(0xF9F)));

__asm("IPR1 equ 0F9Fh");


typedef union {
    struct {
        unsigned TMR1IP :1;
        unsigned TMR2IP :1;
        unsigned CCP1IP :1;
        unsigned SSPIP :1;
        unsigned TXIP :1;
        unsigned RCIP :1;
        unsigned ADIP :1;
    };
    struct {
        unsigned :4;
        unsigned TX1IP :1;
        unsigned RC1IP :1;
    };
} IPR1bits_t;
extern volatile IPR1bits_t IPR1bits __attribute__((address(0xF9F)));
# 3648 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PIE2 __attribute__((address(0xFA0)));

__asm("PIE2 equ 0FA0h");


typedef union {
    struct {
        unsigned :1;
        unsigned TMR3IE :1;
        unsigned USBIE :1;
        unsigned BCLIE :1;
        unsigned EEIE :1;
        unsigned C2IE :1;
        unsigned C1IE :1;
        unsigned OSCFIE :1;
    };
    struct {
        unsigned :6;
        unsigned CMIE :1;
    };
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __attribute__((address(0xFA0)));
# 3714 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PIR2 __attribute__((address(0xFA1)));

__asm("PIR2 equ 0FA1h");


typedef union {
    struct {
        unsigned :1;
        unsigned TMR3IF :1;
        unsigned USBIF :1;
        unsigned BCLIF :1;
        unsigned EEIF :1;
        unsigned C2IF :1;
        unsigned C1IF :1;
        unsigned OSCFIF :1;
    };
    struct {
        unsigned :6;
        unsigned CMIF :1;
    };
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __attribute__((address(0xFA1)));
# 3780 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char IPR2 __attribute__((address(0xFA2)));

__asm("IPR2 equ 0FA2h");


typedef union {
    struct {
        unsigned :1;
        unsigned TMR3IP :1;
        unsigned USBIP :1;
        unsigned BCLIP :1;
        unsigned EEIP :1;
        unsigned C2IP :1;
        unsigned C1IP :1;
        unsigned OSCFIP :1;
    };
    struct {
        unsigned :6;
        unsigned CMIP :1;
    };
} IPR2bits_t;
extern volatile IPR2bits_t IPR2bits __attribute__((address(0xFA2)));
# 3846 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char EECON1 __attribute__((address(0xFA6)));

__asm("EECON1 equ 0FA6h");


typedef union {
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned WREN :1;
        unsigned WRERR :1;
        unsigned FREE :1;
        unsigned :1;
        unsigned CFGS :1;
        unsigned EEPGD :1;
    };
    struct {
        unsigned :6;
        unsigned EEFS :1;
    };
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __attribute__((address(0xFA6)));
# 3912 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char EECON2 __attribute__((address(0xFA7)));

__asm("EECON2 equ 0FA7h");




extern volatile unsigned char EEDATA __attribute__((address(0xFA8)));

__asm("EEDATA equ 0FA8h");




extern volatile unsigned char EEADR __attribute__((address(0xFA9)));

__asm("EEADR equ 0FA9h");


typedef union {
    struct {
        unsigned EEADR0 :1;
        unsigned EEADR1 :1;
        unsigned EEADR2 :1;
        unsigned EEADR3 :1;
        unsigned EEADR4 :1;
        unsigned EEADR5 :1;
        unsigned EEADR6 :1;
        unsigned EEADR7 :1;
    };
} EEADRbits_t;
extern volatile EEADRbits_t EEADRbits __attribute__((address(0xFA9)));
# 3988 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char RCSTA __attribute__((address(0xFAB)));

__asm("RCSTA equ 0FABh");


extern volatile unsigned char RCSTA1 __attribute__((address(0xFAB)));

__asm("RCSTA1 equ 0FABh");


typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
    struct {
        unsigned :3;
        unsigned ADEN :1;
    };
    struct {
        unsigned :5;
        unsigned SRENA :1;
    };
    struct {
        unsigned :6;
        unsigned RC8_9 :1;
    };
    struct {
        unsigned :6;
        unsigned RC9 :1;
    };
    struct {
        unsigned RCD8 :1;
    };
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __attribute__((address(0xFAB)));
# 4097 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned RX9D :1;
        unsigned OERR :1;
        unsigned FERR :1;
        unsigned ADDEN :1;
        unsigned CREN :1;
        unsigned SREN :1;
        unsigned RX9 :1;
        unsigned SPEN :1;
    };
    struct {
        unsigned :3;
        unsigned ADEN :1;
    };
    struct {
        unsigned :5;
        unsigned SRENA :1;
    };
    struct {
        unsigned :6;
        unsigned RC8_9 :1;
    };
    struct {
        unsigned :6;
        unsigned RC9 :1;
    };
    struct {
        unsigned RCD8 :1;
    };
} RCSTA1bits_t;
extern volatile RCSTA1bits_t RCSTA1bits __attribute__((address(0xFAB)));
# 4198 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char TXSTA __attribute__((address(0xFAC)));

__asm("TXSTA equ 0FACh");


extern volatile unsigned char TXSTA1 __attribute__((address(0xFAC)));

__asm("TXSTA1 equ 0FACh");


typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
    struct {
        unsigned TX9D1 :1;
        unsigned TRMT1 :1;
        unsigned BRGH1 :1;
        unsigned SENDB1 :1;
        unsigned SYNC1 :1;
        unsigned TXEN1 :1;
        unsigned TX91 :1;
        unsigned CSRC1 :1;
    };
    struct {
        unsigned :6;
        unsigned TX8_9 :1;
    };
    struct {
        unsigned TXD8 :1;
    };
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __attribute__((address(0xFAC)));
# 4330 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned TX9D :1;
        unsigned TRMT :1;
        unsigned BRGH :1;
        unsigned SENDB :1;
        unsigned SYNC :1;
        unsigned TXEN :1;
        unsigned TX9 :1;
        unsigned CSRC :1;
    };
    struct {
        unsigned TX9D1 :1;
        unsigned TRMT1 :1;
        unsigned BRGH1 :1;
        unsigned SENDB1 :1;
        unsigned SYNC1 :1;
        unsigned TXEN1 :1;
        unsigned TX91 :1;
        unsigned CSRC1 :1;
    };
    struct {
        unsigned :6;
        unsigned TX8_9 :1;
    };
    struct {
        unsigned TXD8 :1;
    };
} TXSTA1bits_t;
extern volatile TXSTA1bits_t TXSTA1bits __attribute__((address(0xFAC)));
# 4454 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char TXREG __attribute__((address(0xFAD)));

__asm("TXREG equ 0FADh");


extern volatile unsigned char TXREG1 __attribute__((address(0xFAD)));

__asm("TXREG1 equ 0FADh");




extern volatile unsigned char RCREG __attribute__((address(0xFAE)));

__asm("RCREG equ 0FAEh");


extern volatile unsigned char RCREG1 __attribute__((address(0xFAE)));

__asm("RCREG1 equ 0FAEh");




extern volatile unsigned char SPBRG __attribute__((address(0xFAF)));

__asm("SPBRG equ 0FAFh");


extern volatile unsigned char SPBRG1 __attribute__((address(0xFAF)));

__asm("SPBRG1 equ 0FAFh");




extern volatile unsigned char SPBRGH __attribute__((address(0xFB0)));

__asm("SPBRGH equ 0FB0h");




extern volatile unsigned char T3CON __attribute__((address(0xFB1)));

__asm("T3CON equ 0FB1h");


typedef union {
    struct {
        unsigned :2;
        unsigned NOT_T3SYNC :1;
    };
    struct {
        unsigned TMR3ON :1;
        unsigned TMR3CS :1;
        unsigned nT3SYNC :1;
        unsigned T3CCP1 :1;
        unsigned T3CKPS :2;
        unsigned :1;
        unsigned RD16 :1;
    };
    struct {
        unsigned :2;
        unsigned T3SYNC :1;
        unsigned :1;
        unsigned T3CKPS0 :1;
        unsigned T3CKPS1 :1;
    };
    struct {
        unsigned :3;
        unsigned SOSCEN3 :1;
        unsigned :3;
        unsigned RD163 :1;
    };
    struct {
        unsigned :7;
        unsigned T3RD16 :1;
    };
} T3CONbits_t;
extern volatile T3CONbits_t T3CONbits __attribute__((address(0xFB1)));
# 4604 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned short TMR3 __attribute__((address(0xFB2)));

__asm("TMR3 equ 0FB2h");




extern volatile unsigned char TMR3L __attribute__((address(0xFB2)));

__asm("TMR3L equ 0FB2h");




extern volatile unsigned char TMR3H __attribute__((address(0xFB3)));

__asm("TMR3H equ 0FB3h");




extern volatile unsigned char ECCP1AS __attribute__((address(0xFB6)));

__asm("ECCP1AS equ 0FB6h");


typedef union {
    struct {
        unsigned PSSBD :2;
        unsigned PSSAC :2;
        unsigned ECCPAS :3;
        unsigned ECCPASE :1;
    };
    struct {
        unsigned PSSBD0 :1;
        unsigned PSSBD1 :1;
        unsigned PSSAC0 :1;
        unsigned PSSAC1 :1;
        unsigned ECCPAS0 :1;
        unsigned ECCPAS1 :1;
        unsigned ECCPAS2 :1;
    };
} ECCP1ASbits_t;
extern volatile ECCP1ASbits_t ECCP1ASbits __attribute__((address(0xFB6)));
# 4707 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PWM1CON __attribute__((address(0xFB7)));

__asm("PWM1CON equ 0FB7h");


typedef union {
    struct {
        unsigned PDC :7;
        unsigned PRSEN :1;
    };
    struct {
        unsigned PDC0 :1;
        unsigned PDC1 :1;
        unsigned PDC2 :1;
        unsigned PDC3 :1;
        unsigned PDC4 :1;
        unsigned PDC5 :1;
        unsigned PDC6 :1;
    };
} PWM1CONbits_t;
extern volatile PWM1CONbits_t PWM1CONbits __attribute__((address(0xFB7)));
# 4777 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char BAUDCON __attribute__((address(0xFB8)));

__asm("BAUDCON equ 0FB8h");


extern volatile unsigned char BAUDCTL __attribute__((address(0xFB8)));

__asm("BAUDCTL equ 0FB8h");


typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned CKTXP :1;
        unsigned DTRXP :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
    struct {
        unsigned :4;
        unsigned SCKP :1;
    };
    struct {
        unsigned :5;
        unsigned RXCKP :1;
    };
    struct {
        unsigned :1;
        unsigned W4E :1;
    };
} BAUDCONbits_t;
extern volatile BAUDCONbits_t BAUDCONbits __attribute__((address(0xFB8)));
# 4864 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned ABDEN :1;
        unsigned WUE :1;
        unsigned :1;
        unsigned BRG16 :1;
        unsigned CKTXP :1;
        unsigned DTRXP :1;
        unsigned RCIDL :1;
        unsigned ABDOVF :1;
    };
    struct {
        unsigned :4;
        unsigned SCKP :1;
    };
    struct {
        unsigned :5;
        unsigned RXCKP :1;
    };
    struct {
        unsigned :1;
        unsigned W4E :1;
    };
} BAUDCTLbits_t;
extern volatile BAUDCTLbits_t BAUDCTLbits __attribute__((address(0xFB8)));
# 4943 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PSTRCON __attribute__((address(0xFB9)));

__asm("PSTRCON equ 0FB9h");


typedef union {
    struct {
        unsigned STRA :1;
        unsigned STRB :1;
        unsigned STRC :1;
        unsigned STRD :1;
        unsigned STRSYNC :1;
    };
} PSTRCONbits_t;
extern volatile PSTRCONbits_t PSTRCONbits __attribute__((address(0xFB9)));
# 4987 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char REFCON0 __attribute__((address(0xFBA)));

__asm("REFCON0 equ 0FBAh");


extern volatile unsigned char VREFCON0 __attribute__((address(0xFBA)));

__asm("VREFCON0 equ 0FBAh");


typedef union {
    struct {
        unsigned :3;
        unsigned :1;
        unsigned FVR1S0 :1;
        unsigned FVR1S1 :1;
        unsigned FVR1ST :1;
        unsigned FVR1EN :1;
    };
    struct {
        unsigned :4;
        unsigned FVR1S :2;
    };
} REFCON0bits_t;
extern volatile REFCON0bits_t REFCON0bits __attribute__((address(0xFBA)));
# 5039 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned :3;
        unsigned :1;
        unsigned FVR1S0 :1;
        unsigned FVR1S1 :1;
        unsigned FVR1ST :1;
        unsigned FVR1EN :1;
    };
    struct {
        unsigned :4;
        unsigned FVR1S :2;
    };
} VREFCON0bits_t;
extern volatile VREFCON0bits_t VREFCON0bits __attribute__((address(0xFBA)));
# 5083 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char REFCON1 __attribute__((address(0xFBB)));

__asm("REFCON1 equ 0FBBh");


extern volatile unsigned char VREFCON1 __attribute__((address(0xFBB)));

__asm("VREFCON1 equ 0FBBh");


typedef union {
    struct {
        unsigned D1NSS :1;
        unsigned :1;
        unsigned D1PSS :2;
        unsigned :1;
        unsigned DAC1OE :1;
        unsigned D1LPS :1;
        unsigned D1EN :1;
    };
    struct {
        unsigned :2;
        unsigned D1PSS0 :1;
        unsigned D1PSS1 :1;
    };
    struct {
        unsigned D1NSS0 :1;
    };
} REFCON1bits_t;
extern volatile REFCON1bits_t REFCON1bits __attribute__((address(0xFBB)));
# 5155 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned D1NSS :1;
        unsigned :1;
        unsigned D1PSS :2;
        unsigned :1;
        unsigned DAC1OE :1;
        unsigned D1LPS :1;
        unsigned D1EN :1;
    };
    struct {
        unsigned :2;
        unsigned D1PSS0 :1;
        unsigned D1PSS1 :1;
    };
    struct {
        unsigned D1NSS0 :1;
    };
} VREFCON1bits_t;
extern volatile VREFCON1bits_t VREFCON1bits __attribute__((address(0xFBB)));
# 5219 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char REFCON2 __attribute__((address(0xFBC)));

__asm("REFCON2 equ 0FBCh");


extern volatile unsigned char VREFCON2 __attribute__((address(0xFBC)));

__asm("VREFCON2 equ 0FBCh");


typedef union {
    struct {
        unsigned DAC1R :5;
    };
    struct {
        unsigned DAC1R0 :1;
        unsigned DAC1R1 :1;
        unsigned DAC1R2 :1;
        unsigned DAC1R3 :1;
        unsigned DAC1R4 :1;
    };
} REFCON2bits_t;
extern volatile REFCON2bits_t REFCON2bits __attribute__((address(0xFBC)));
# 5274 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned DAC1R :5;
    };
    struct {
        unsigned DAC1R0 :1;
        unsigned DAC1R1 :1;
        unsigned DAC1R2 :1;
        unsigned DAC1R3 :1;
        unsigned DAC1R4 :1;
    };
} VREFCON2bits_t;
extern volatile VREFCON2bits_t VREFCON2bits __attribute__((address(0xFBC)));
# 5321 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char CCP1CON __attribute__((address(0xFBD)));

__asm("CCP1CON equ 0FBDh");


typedef union {
    struct {
        unsigned CCP1M :4;
        unsigned DC1B :2;
        unsigned P1M :2;
    };
    struct {
        unsigned CCP1M0 :1;
        unsigned CCP1M1 :1;
        unsigned CCP1M2 :1;
        unsigned CCP1M3 :1;
        unsigned DC1B0 :1;
        unsigned DC1B1 :1;
        unsigned P1M0 :1;
        unsigned P1M1 :1;
    };
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __attribute__((address(0xFBD)));
# 5403 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned short CCPR1 __attribute__((address(0xFBE)));

__asm("CCPR1 equ 0FBEh");




extern volatile unsigned char CCPR1L __attribute__((address(0xFBE)));

__asm("CCPR1L equ 0FBEh");




extern volatile unsigned char CCPR1H __attribute__((address(0xFBF)));

__asm("CCPR1H equ 0FBFh");




extern volatile unsigned char ADCON2 __attribute__((address(0xFC0)));

__asm("ADCON2 equ 0FC0h");


typedef union {
    struct {
        unsigned ADCS :3;
        unsigned ACQT :3;
        unsigned :1;
        unsigned ADFM :1;
    };
    struct {
        unsigned ADCS0 :1;
        unsigned ADCS1 :1;
        unsigned ADCS2 :1;
        unsigned ACQT0 :1;
        unsigned ACQT1 :1;
        unsigned ACQT2 :1;
    };
} ADCON2bits_t;
extern volatile ADCON2bits_t ADCON2bits __attribute__((address(0xFC0)));
# 5495 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char ADCON1 __attribute__((address(0xFC1)));

__asm("ADCON1 equ 0FC1h");


typedef union {
    struct {
        unsigned NVCFG0 :1;
        unsigned NVCFG1 :1;
        unsigned PVCFG0 :1;
        unsigned PVCFG1 :1;
    };
    struct {
        unsigned :3;
        unsigned CHSN3 :1;
    };
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __attribute__((address(0xFC1)));
# 5542 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char ADCON0 __attribute__((address(0xFC2)));

__asm("ADCON0 equ 0FC2h");


typedef union {
    struct {
        unsigned :1;
        unsigned GO_NOT_DONE :1;
    };
    struct {
        unsigned ADON :1;
        unsigned GO_nDONE :1;
        unsigned CHS :4;
    };
    struct {
        unsigned :1;
        unsigned GO :1;
        unsigned CHS0 :1;
        unsigned CHS1 :1;
        unsigned CHS2 :1;
        unsigned CHS3 :1;
    };
    struct {
        unsigned :1;
        unsigned DONE :1;
    };
    struct {
        unsigned :1;
        unsigned NOT_DONE :1;
    };
    struct {
        unsigned :1;
        unsigned nDONE :1;
    };
    struct {
        unsigned :1;
        unsigned GO_DONE :1;
    };
    struct {
        unsigned :1;
        unsigned GODONE :1;
    };
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __attribute__((address(0xFC2)));
# 5661 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned short ADRES __attribute__((address(0xFC3)));

__asm("ADRES equ 0FC3h");




extern volatile unsigned char ADRESL __attribute__((address(0xFC3)));

__asm("ADRESL equ 0FC3h");




extern volatile unsigned char ADRESH __attribute__((address(0xFC4)));

__asm("ADRESH equ 0FC4h");




extern volatile unsigned char SSPCON2 __attribute__((address(0xFC5)));

__asm("SSPCON2 equ 0FC5h");


typedef union {
    struct {
        unsigned SEN :1;
        unsigned RSEN :1;
        unsigned PEN :1;
        unsigned RCEN :1;
        unsigned ACKEN :1;
        unsigned ACKDT :1;
        unsigned ACKSTAT :1;
        unsigned GCEN :1;
    };
} SSPCON2bits_t;
extern volatile SSPCON2bits_t SSPCON2bits __attribute__((address(0xFC5)));
# 5744 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char SSPCON1 __attribute__((address(0xFC6)));

__asm("SSPCON1 equ 0FC6h");


typedef union {
    struct {
        unsigned SSPM :4;
        unsigned CKP :1;
        unsigned SSPEN :1;
        unsigned SSPOV :1;
        unsigned WCOL :1;
    };
    struct {
        unsigned SSPM0 :1;
        unsigned SSPM1 :1;
        unsigned SSPM2 :1;
        unsigned SSPM3 :1;
    };
} SSPCON1bits_t;
extern volatile SSPCON1bits_t SSPCON1bits __attribute__((address(0xFC6)));
# 5814 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char SSPSTAT __attribute__((address(0xFC7)));

__asm("SSPSTAT equ 0FC7h");


typedef union {
    struct {
        unsigned :2;
        unsigned R_NOT_W :1;
    };
    struct {
        unsigned :5;
        unsigned D_NOT_A :1;
    };
    struct {
        unsigned BF :1;
        unsigned UA :1;
        unsigned R_nW :1;
        unsigned S :1;
        unsigned P :1;
        unsigned D_nA :1;
        unsigned CKE :1;
        unsigned SMP :1;
    };
    struct {
        unsigned :2;
        unsigned R :1;
        unsigned :2;
        unsigned D :1;
    };
    struct {
        unsigned :2;
        unsigned W :1;
        unsigned :2;
        unsigned A :1;
    };
    struct {
        unsigned :2;
        unsigned nW :1;
        unsigned :2;
        unsigned nA :1;
    };
    struct {
        unsigned :2;
        unsigned R_W :1;
        unsigned :2;
        unsigned D_A :1;
    };
    struct {
        unsigned :2;
        unsigned NOT_WRITE :1;
    };
    struct {
        unsigned :5;
        unsigned NOT_ADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned nWRITE :1;
        unsigned :2;
        unsigned nADDRESS :1;
    };
    struct {
        unsigned :2;
        unsigned RW :1;
        unsigned START :1;
        unsigned STOP :1;
        unsigned DA :1;
    };
    struct {
        unsigned :2;
        unsigned NOT_W :1;
        unsigned :2;
        unsigned NOT_A :1;
    };
} SSPSTATbits_t;
extern volatile SSPSTATbits_t SSPSTATbits __attribute__((address(0xFC7)));
# 6035 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char SSPADD __attribute__((address(0xFC8)));

__asm("SSPADD equ 0FC8h");




extern volatile unsigned char SSPBUF __attribute__((address(0xFC9)));

__asm("SSPBUF equ 0FC9h");




extern volatile unsigned char T2CON __attribute__((address(0xFCA)));

__asm("T2CON equ 0FCAh");


typedef union {
    struct {
        unsigned T2CKPS :2;
        unsigned TMR2ON :1;
        unsigned T2OUTPS :4;
    };
    struct {
        unsigned T2CKPS0 :1;
        unsigned T2CKPS1 :1;
        unsigned :1;
        unsigned T2OUTPS0 :1;
        unsigned T2OUTPS1 :1;
        unsigned T2OUTPS2 :1;
        unsigned T2OUTPS3 :1;
    };
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __attribute__((address(0xFCA)));
# 6120 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char PR2 __attribute__((address(0xFCB)));

__asm("PR2 equ 0FCBh");


extern volatile unsigned char MEMCON __attribute__((address(0xFCB)));

__asm("MEMCON equ 0FCBh");


typedef union {
    struct {
        unsigned :7;
        unsigned EBDIS :1;
    };
    struct {
        unsigned :4;
        unsigned WAIT0 :1;
    };
    struct {
        unsigned :5;
        unsigned WAIT1 :1;
    };
    struct {
        unsigned WM0 :1;
    };
    struct {
        unsigned :1;
        unsigned WM1 :1;
    };
} PR2bits_t;
extern volatile PR2bits_t PR2bits __attribute__((address(0xFCB)));
# 6179 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
typedef union {
    struct {
        unsigned :7;
        unsigned EBDIS :1;
    };
    struct {
        unsigned :4;
        unsigned WAIT0 :1;
    };
    struct {
        unsigned :5;
        unsigned WAIT1 :1;
    };
    struct {
        unsigned WM0 :1;
    };
    struct {
        unsigned :1;
        unsigned WM1 :1;
    };
} MEMCONbits_t;
extern volatile MEMCONbits_t MEMCONbits __attribute__((address(0xFCB)));
# 6230 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char TMR2 __attribute__((address(0xFCC)));

__asm("TMR2 equ 0FCCh");




extern volatile unsigned char T1CON __attribute__((address(0xFCD)));

__asm("T1CON equ 0FCDh");


typedef union {
    struct {
        unsigned :2;
        unsigned NOT_T1SYNC :1;
    };
    struct {
        unsigned TMR1ON :1;
        unsigned TMR1CS :1;
        unsigned nT1SYNC :1;
        unsigned T1OSCEN :1;
        unsigned T1CKPS :2;
        unsigned T1RUN :1;
        unsigned RD16 :1;
    };
    struct {
        unsigned :2;
        unsigned T1SYNC :1;
        unsigned :1;
        unsigned T1CKPS0 :1;
        unsigned T1CKPS1 :1;
    };
    struct {
        unsigned :3;
        unsigned SOSCEN :1;
        unsigned :3;
        unsigned T1RD16 :1;
    };
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __attribute__((address(0xFCD)));
# 6340 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned short TMR1 __attribute__((address(0xFCE)));

__asm("TMR1 equ 0FCEh");




extern volatile unsigned char TMR1L __attribute__((address(0xFCE)));

__asm("TMR1L equ 0FCEh");




extern volatile unsigned char TMR1H __attribute__((address(0xFCF)));

__asm("TMR1H equ 0FCFh");




extern volatile unsigned char RCON __attribute__((address(0xFD0)));

__asm("RCON equ 0FD0h");


typedef union {
    struct {
        unsigned NOT_BOR :1;
    };
    struct {
        unsigned :1;
        unsigned NOT_POR :1;
    };
    struct {
        unsigned :2;
        unsigned NOT_PD :1;
    };
    struct {
        unsigned :3;
        unsigned NOT_TO :1;
    };
    struct {
        unsigned :4;
        unsigned NOT_RI :1;
    };
    struct {
        unsigned nBOR :1;
        unsigned nPOR :1;
        unsigned nPD :1;
        unsigned nTO :1;
        unsigned nRI :1;
        unsigned :1;
        unsigned SBOREN :1;
        unsigned IPEN :1;
    };
    struct {
        unsigned BOR :1;
        unsigned POR :1;
        unsigned PD :1;
        unsigned TO :1;
        unsigned RI :1;
    };
} RCONbits_t;
extern volatile RCONbits_t RCONbits __attribute__((address(0xFD0)));
# 6494 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char WDTCON __attribute__((address(0xFD1)));

__asm("WDTCON equ 0FD1h");


typedef union {
    struct {
        unsigned SWDTEN :1;
    };
    struct {
        unsigned SWDTE :1;
    };
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __attribute__((address(0xFD1)));
# 6522 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char OSCCON2 __attribute__((address(0xFD2)));

__asm("OSCCON2 equ 0FD2h");


typedef union {
    struct {
        unsigned LFIOFS :1;
        unsigned HFIOFL :1;
        unsigned PRI_SD :1;
    };
} OSCCON2bits_t;
extern volatile OSCCON2bits_t OSCCON2bits __attribute__((address(0xFD2)));
# 6554 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char OSCCON __attribute__((address(0xFD3)));

__asm("OSCCON equ 0FD3h");


typedef union {
    struct {
        unsigned SCS0 :1;
        unsigned SCS1 :1;
        unsigned IOFS :1;
        unsigned OSTS :1;
        unsigned IRCF :3;
        unsigned IDLEN :1;
    };
    struct {
        unsigned :4;
        unsigned IRCF0 :1;
        unsigned IRCF1 :1;
        unsigned IRCF2 :1;
    };
    struct {
        unsigned SCS :1;
    };
} OSCCONbits_t;
extern volatile OSCCONbits_t OSCCONbits __attribute__((address(0xFD3)));
# 6633 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char T0CON __attribute__((address(0xFD5)));

__asm("T0CON equ 0FD5h");


typedef union {
    struct {
        unsigned T0PS :3;
        unsigned PSA :1;
        unsigned T0SE :1;
        unsigned T0CS :1;
        unsigned T08BIT :1;
        unsigned TMR0ON :1;
    };
    struct {
        unsigned T0PS0 :1;
        unsigned T0PS1 :1;
        unsigned T0PS2 :1;
    };
} T0CONbits_t;
extern volatile T0CONbits_t T0CONbits __attribute__((address(0xFD5)));
# 6703 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned short TMR0 __attribute__((address(0xFD6)));

__asm("TMR0 equ 0FD6h");




extern volatile unsigned char TMR0L __attribute__((address(0xFD6)));

__asm("TMR0L equ 0FD6h");




extern volatile unsigned char TMR0H __attribute__((address(0xFD7)));

__asm("TMR0H equ 0FD7h");




extern volatile unsigned char STATUS __attribute__((address(0xFD8)));

__asm("STATUS equ 0FD8h");


typedef union {
    struct {
        unsigned C :1;
        unsigned DC :1;
        unsigned Z :1;
        unsigned OV :1;
        unsigned N :1;
    };
    struct {
        unsigned CARRY :1;
        unsigned :1;
        unsigned ZERO :1;
        unsigned OVERFLOW :1;
        unsigned NEGATIVE :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0xFD8)));
# 6795 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned short FSR2 __attribute__((address(0xFD9)));

__asm("FSR2 equ 0FD9h");




extern volatile unsigned char FSR2L __attribute__((address(0xFD9)));

__asm("FSR2L equ 0FD9h");




extern volatile unsigned char FSR2H __attribute__((address(0xFDA)));

__asm("FSR2H equ 0FDAh");




extern volatile unsigned char PLUSW2 __attribute__((address(0xFDB)));

__asm("PLUSW2 equ 0FDBh");




extern volatile unsigned char PREINC2 __attribute__((address(0xFDC)));

__asm("PREINC2 equ 0FDCh");




extern volatile unsigned char POSTDEC2 __attribute__((address(0xFDD)));

__asm("POSTDEC2 equ 0FDDh");




extern volatile unsigned char POSTINC2 __attribute__((address(0xFDE)));

__asm("POSTINC2 equ 0FDEh");




extern volatile unsigned char INDF2 __attribute__((address(0xFDF)));

__asm("INDF2 equ 0FDFh");




extern volatile unsigned char BSR __attribute__((address(0xFE0)));

__asm("BSR equ 0FE0h");




extern volatile unsigned short FSR1 __attribute__((address(0xFE1)));

__asm("FSR1 equ 0FE1h");




extern volatile unsigned char FSR1L __attribute__((address(0xFE1)));

__asm("FSR1L equ 0FE1h");




extern volatile unsigned char FSR1H __attribute__((address(0xFE2)));

__asm("FSR1H equ 0FE2h");




extern volatile unsigned char PLUSW1 __attribute__((address(0xFE3)));

__asm("PLUSW1 equ 0FE3h");




extern volatile unsigned char PREINC1 __attribute__((address(0xFE4)));

__asm("PREINC1 equ 0FE4h");




extern volatile unsigned char POSTDEC1 __attribute__((address(0xFE5)));

__asm("POSTDEC1 equ 0FE5h");




extern volatile unsigned char POSTINC1 __attribute__((address(0xFE6)));

__asm("POSTINC1 equ 0FE6h");




extern volatile unsigned char INDF1 __attribute__((address(0xFE7)));

__asm("INDF1 equ 0FE7h");




extern volatile unsigned char WREG __attribute__((address(0xFE8)));

__asm("WREG equ 0FE8h");
# 6926 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned short FSR0 __attribute__((address(0xFE9)));

__asm("FSR0 equ 0FE9h");




extern volatile unsigned char FSR0L __attribute__((address(0xFE9)));

__asm("FSR0L equ 0FE9h");




extern volatile unsigned char FSR0H __attribute__((address(0xFEA)));

__asm("FSR0H equ 0FEAh");




extern volatile unsigned char PLUSW0 __attribute__((address(0xFEB)));

__asm("PLUSW0 equ 0FEBh");




extern volatile unsigned char PREINC0 __attribute__((address(0xFEC)));

__asm("PREINC0 equ 0FECh");




extern volatile unsigned char POSTDEC0 __attribute__((address(0xFED)));

__asm("POSTDEC0 equ 0FEDh");




extern volatile unsigned char POSTINC0 __attribute__((address(0xFEE)));

__asm("POSTINC0 equ 0FEEh");




extern volatile unsigned char INDF0 __attribute__((address(0xFEF)));

__asm("INDF0 equ 0FEFh");




extern volatile unsigned char INTCON3 __attribute__((address(0xFF0)));

__asm("INTCON3 equ 0FF0h");


typedef union {
    struct {
        unsigned INT1IF :1;
        unsigned INT2IF :1;
        unsigned :1;
        unsigned INT1IE :1;
        unsigned INT2IE :1;
        unsigned :1;
        unsigned INT1IP :1;
        unsigned INT2IP :1;
    };
    struct {
        unsigned INT1F :1;
        unsigned INT2F :1;
        unsigned :1;
        unsigned INT1E :1;
        unsigned INT2E :1;
        unsigned :1;
        unsigned INT1P :1;
        unsigned INT2P :1;
    };
} INTCON3bits_t;
extern volatile INTCON3bits_t INTCON3bits __attribute__((address(0xFF0)));
# 7074 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char INTCON2 __attribute__((address(0xFF1)));

__asm("INTCON2 equ 0FF1h");


typedef union {
    struct {
        unsigned :7;
        unsigned NOT_RABPU :1;
    };
    struct {
        unsigned RABIP :1;
        unsigned :1;
        unsigned TMR0IP :1;
        unsigned :1;
        unsigned INTEDG2 :1;
        unsigned INTEDG1 :1;
        unsigned INTEDG0 :1;
        unsigned nRABPU :1;
    };
    struct {
        unsigned :7;
        unsigned RABPU :1;
    };
} INTCON2bits_t;
extern volatile INTCON2bits_t INTCON2bits __attribute__((address(0xFF1)));
# 7144 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned char INTCON __attribute__((address(0xFF2)));

__asm("INTCON equ 0FF2h");


typedef union {
    struct {
        unsigned RABIF :1;
        unsigned INT0IF :1;
        unsigned TMR0IF :1;
        unsigned RABIE :1;
        unsigned INT0IE :1;
        unsigned TMR0IE :1;
        unsigned PEIE_GIEL :1;
        unsigned GIE_GIEH :1;
    };
    struct {
        unsigned :1;
        unsigned INT0F :1;
        unsigned T0IF :1;
        unsigned :1;
        unsigned INT0E :1;
        unsigned T0IE :1;
        unsigned PEIE :1;
        unsigned GIE :1;
    };
    struct {
        unsigned :6;
        unsigned GIEL :1;
        unsigned GIEH :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0xFF2)));
# 7261 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile unsigned short PROD __attribute__((address(0xFF3)));

__asm("PROD equ 0FF3h");




extern volatile unsigned char PRODL __attribute__((address(0xFF3)));

__asm("PRODL equ 0FF3h");




extern volatile unsigned char PRODH __attribute__((address(0xFF4)));

__asm("PRODH equ 0FF4h");




extern volatile unsigned char TABLAT __attribute__((address(0xFF5)));

__asm("TABLAT equ 0FF5h");





extern volatile __uint24 TBLPTR __attribute__((address(0xFF6)));


__asm("TBLPTR equ 0FF6h");




extern volatile unsigned char TBLPTRL __attribute__((address(0xFF6)));

__asm("TBLPTRL equ 0FF6h");




extern volatile unsigned char TBLPTRH __attribute__((address(0xFF7)));

__asm("TBLPTRH equ 0FF7h");




extern volatile unsigned char TBLPTRU __attribute__((address(0xFF8)));

__asm("TBLPTRU equ 0FF8h");





extern volatile __uint24 PCLAT __attribute__((address(0xFF9)));


__asm("PCLAT equ 0FF9h");



extern volatile __uint24 PC __attribute__((address(0xFF9)));


__asm("PC equ 0FF9h");




extern volatile unsigned char PCL __attribute__((address(0xFF9)));

__asm("PCL equ 0FF9h");




extern volatile unsigned char PCLATH __attribute__((address(0xFFA)));

__asm("PCLATH equ 0FFAh");




extern volatile unsigned char PCLATU __attribute__((address(0xFFB)));

__asm("PCLATU equ 0FFBh");




extern volatile unsigned char STKPTR __attribute__((address(0xFFC)));

__asm("STKPTR equ 0FFCh");


typedef union {
    struct {
        unsigned STKPTR :5;
        unsigned :1;
        unsigned STKUNF :1;
        unsigned STKFUL :1;
    };
    struct {
        unsigned SP0 :1;
        unsigned SP1 :1;
        unsigned SP2 :1;
        unsigned SP3 :1;
        unsigned SP4 :1;
        unsigned :2;
        unsigned STKOVF :1;
    };
} STKPTRbits_t;
extern volatile STKPTRbits_t STKPTRbits __attribute__((address(0xFFC)));
# 7429 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile __uint24 TOS __attribute__((address(0xFFD)));


__asm("TOS equ 0FFDh");




extern volatile unsigned char TOSL __attribute__((address(0xFFD)));

__asm("TOSL equ 0FFDh");




extern volatile unsigned char TOSH __attribute__((address(0xFFE)));

__asm("TOSH equ 0FFEh");




extern volatile unsigned char TOSU __attribute__((address(0xFFF)));

__asm("TOSU equ 0FFFh");
# 7468 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/proc/pic18f14k50.h" 3
extern volatile __bit ABDEN __attribute__((address(0x7DC0)));


extern volatile __bit ABDOVF __attribute__((address(0x7DC7)));


extern volatile __bit ACKDT __attribute__((address(0x7E2D)));


extern volatile __bit ACKEN __attribute__((address(0x7E2C)));


extern volatile __bit ACKSTAT __attribute__((address(0x7E2E)));


extern volatile __bit ACQT0 __attribute__((address(0x7E03)));


extern volatile __bit ACQT1 __attribute__((address(0x7E04)));


extern volatile __bit ACQT2 __attribute__((address(0x7E05)));


extern volatile __bit ACTVIE __attribute__((address(0x7B02)));


extern volatile __bit ACTVIF __attribute__((address(0x7B12)));


extern volatile __bit ADCS0 __attribute__((address(0x7E00)));


extern volatile __bit ADCS1 __attribute__((address(0x7E01)));


extern volatile __bit ADCS2 __attribute__((address(0x7E02)));


extern volatile __bit ADDEN __attribute__((address(0x7D5B)));


extern volatile __bit ADDR0 __attribute__((address(0x7AE0)));


extern volatile __bit ADDR1 __attribute__((address(0x7AE1)));


extern volatile __bit ADDR2 __attribute__((address(0x7AE2)));


extern volatile __bit ADDR3 __attribute__((address(0x7AE3)));


extern volatile __bit ADDR4 __attribute__((address(0x7AE4)));


extern volatile __bit ADDR5 __attribute__((address(0x7AE5)));


extern volatile __bit ADDR6 __attribute__((address(0x7AE6)));


extern volatile __bit ADEN __attribute__((address(0x7D5B)));


extern volatile __bit ADFM __attribute__((address(0x7E07)));


extern volatile __bit ADIE __attribute__((address(0x7CEE)));


extern volatile __bit ADIF __attribute__((address(0x7CF6)));


extern volatile __bit ADIP __attribute__((address(0x7CFE)));


extern volatile __bit ADON __attribute__((address(0x7E10)));


extern volatile __bit AN10 __attribute__((address(0x7C0C)));


extern volatile __bit AN11 __attribute__((address(0x7C0D)));


extern volatile __bit AN3 __attribute__((address(0x7C04)));


extern volatile __bit AN4 __attribute__((address(0x7C10)));


extern volatile __bit AN5 __attribute__((address(0x7C11)));


extern volatile __bit AN6 __attribute__((address(0x7C12)));


extern volatile __bit AN7 __attribute__((address(0x7C13)));


extern volatile __bit AN8 __attribute__((address(0x7C16)));


extern volatile __bit AN9 __attribute__((address(0x7C17)));


extern volatile __bit ANS10 __attribute__((address(0x7BFA)));


extern volatile __bit ANS11 __attribute__((address(0x7BFB)));


extern volatile __bit ANS3 __attribute__((address(0x7BF3)));


extern volatile __bit ANS4 __attribute__((address(0x7BF4)));


extern volatile __bit ANS5 __attribute__((address(0x7BF5)));


extern volatile __bit ANS6 __attribute__((address(0x7BF6)));


extern volatile __bit ANS7 __attribute__((address(0x7BF7)));


extern volatile __bit ANS8 __attribute__((address(0x7BF8)));


extern volatile __bit ANS9 __attribute__((address(0x7BF9)));


extern volatile __bit BCLIE __attribute__((address(0x7D03)));


extern volatile __bit BCLIF __attribute__((address(0x7D0B)));


extern volatile __bit BCLIP __attribute__((address(0x7D13)));


extern volatile __bit BF __attribute__((address(0x7E38)));


extern volatile __bit BOR __attribute__((address(0x7E80)));


extern volatile __bit BRG16 __attribute__((address(0x7DC3)));


extern volatile __bit BRGH __attribute__((address(0x7D62)));


extern volatile __bit BRGH1 __attribute__((address(0x7D62)));


extern volatile __bit BTOEE __attribute__((address(0x7ADC)));


extern volatile __bit BTOEF __attribute__((address(0x7AFC)));


extern volatile __bit BTSEE __attribute__((address(0x7ADF)));


extern volatile __bit BTSEF __attribute__((address(0x7AFF)));


extern volatile __bit C12IN1M __attribute__((address(0x7C11)));


extern volatile __bit C12IN2M __attribute__((address(0x7C12)));


extern volatile __bit C12IN3M __attribute__((address(0x7C13)));


extern volatile __bit C12INP __attribute__((address(0x7C10)));


extern volatile __bit C12OUT __attribute__((address(0x7C14)));


extern volatile __bit C1CH0 __attribute__((address(0x7B68)));


extern volatile __bit C1CH1 __attribute__((address(0x7B69)));


extern volatile __bit C1HYS __attribute__((address(0x7B63)));


extern volatile __bit C1IE __attribute__((address(0x7D06)));


extern volatile __bit C1IF __attribute__((address(0x7D0E)));


extern volatile __bit C1IP __attribute__((address(0x7D16)));


extern volatile __bit C1OE __attribute__((address(0x7B6D)));


extern volatile __bit C1ON __attribute__((address(0x7B6F)));


extern volatile __bit C1OUT __attribute__((address(0x7B6E)));


extern volatile __bit C1POL __attribute__((address(0x7B6C)));


extern volatile __bit C1R __attribute__((address(0x7B6A)));


extern volatile __bit C1RSEL __attribute__((address(0x7B65)));


extern volatile __bit C1SP __attribute__((address(0x7B6B)));


extern volatile __bit C1SYNC __attribute__((address(0x7B61)));


extern volatile __bit C2CH0 __attribute__((address(0x7B58)));


extern volatile __bit C2CH1 __attribute__((address(0x7B59)));


extern volatile __bit C2HYS __attribute__((address(0x7B62)));


extern volatile __bit C2IE __attribute__((address(0x7D05)));


extern volatile __bit C2IF __attribute__((address(0x7D0D)));


extern volatile __bit C2IP __attribute__((address(0x7D15)));


extern volatile __bit C2OE __attribute__((address(0x7B5D)));


extern volatile __bit C2ON __attribute__((address(0x7B5F)));


extern volatile __bit C2OUT __attribute__((address(0x7B5E)));


extern volatile __bit C2POL __attribute__((address(0x7B5C)));


extern volatile __bit C2R __attribute__((address(0x7B5A)));


extern volatile __bit C2RSEL __attribute__((address(0x7B64)));


extern volatile __bit C2SP __attribute__((address(0x7B5B)));


extern volatile __bit C2SYNC __attribute__((address(0x7B60)));


extern volatile __bit CARRY __attribute__((address(0x7EC0)));


extern volatile __bit CCP1 __attribute__((address(0x7C15)));


extern volatile __bit CCP1IE __attribute__((address(0x7CEA)));


extern volatile __bit CCP1IF __attribute__((address(0x7CF2)));


extern volatile __bit CCP1IP __attribute__((address(0x7CFA)));


extern volatile __bit CCP1M0 __attribute__((address(0x7DE8)));


extern volatile __bit CCP1M1 __attribute__((address(0x7DE9)));


extern volatile __bit CCP1M2 __attribute__((address(0x7DEA)));


extern volatile __bit CCP1M3 __attribute__((address(0x7DEB)));


extern volatile __bit CCP2 __attribute__((address(0x7C11)));


extern volatile __bit CFGS __attribute__((address(0x7D36)));


extern volatile __bit CHS0 __attribute__((address(0x7E12)));


extern volatile __bit CHS1 __attribute__((address(0x7E13)));


extern volatile __bit CHS2 __attribute__((address(0x7E14)));


extern volatile __bit CHS3 __attribute__((address(0x7E15)));


extern volatile __bit CHSN3 __attribute__((address(0x7E0B)));


extern volatile __bit CK __attribute__((address(0x7C0F)));


extern volatile __bit CKE __attribute__((address(0x7E3E)));


extern volatile __bit CKP __attribute__((address(0x7E34)));


extern volatile __bit CKTXP __attribute__((address(0x7DC4)));


extern volatile __bit CLKIN __attribute__((address(0x7C05)));


extern volatile __bit CLKOUT __attribute__((address(0x7C04)));


extern volatile __bit CMIE __attribute__((address(0x7D06)));


extern volatile __bit CMIF __attribute__((address(0x7D0E)));


extern volatile __bit CMIP __attribute__((address(0x7D16)));


extern volatile __bit CRC16EE __attribute__((address(0x7ADA)));


extern volatile __bit CRC16EF __attribute__((address(0x7AFA)));


extern volatile __bit CRC5EE __attribute__((address(0x7AD9)));


extern volatile __bit CRC5EF __attribute__((address(0x7AF9)));


extern volatile __bit CREN __attribute__((address(0x7D5C)));


extern volatile __bit CSRC __attribute__((address(0x7D67)));


extern volatile __bit CSRC1 __attribute__((address(0x7D67)));


extern volatile __bit CVREF __attribute__((address(0x7C12)));


extern volatile __bit D1EN __attribute__((address(0x7DDF)));


extern volatile __bit D1LPS __attribute__((address(0x7DDE)));


extern volatile __bit D1NSS __attribute__((address(0x7DD8)));


extern volatile __bit D1NSS0 __attribute__((address(0x7DD8)));


extern volatile __bit D1PSS0 __attribute__((address(0x7DDA)));


extern volatile __bit D1PSS1 __attribute__((address(0x7DDB)));


extern volatile __bit DA __attribute__((address(0x7E3D)));


extern volatile __bit DAC1OE __attribute__((address(0x7DDD)));


extern volatile __bit DAC1R0 __attribute__((address(0x7DE0)));


extern volatile __bit DAC1R1 __attribute__((address(0x7DE1)));


extern volatile __bit DAC1R2 __attribute__((address(0x7DE2)));


extern volatile __bit DAC1R3 __attribute__((address(0x7DE3)));


extern volatile __bit DAC1R4 __attribute__((address(0x7DE4)));


extern volatile __bit DC __attribute__((address(0x7EC1)));


extern volatile __bit DC1B0 __attribute__((address(0x7DEC)));


extern volatile __bit DC1B1 __attribute__((address(0x7DED)));


extern volatile __bit DFN8EE __attribute__((address(0x7ADB)));


extern volatile __bit DFN8EF __attribute__((address(0x7AFB)));


extern volatile __bit DIR __attribute__((address(0x7B1A)));


extern volatile __bit DONE __attribute__((address(0x7E11)));


extern volatile __bit DT __attribute__((address(0x7C0D)));


extern volatile __bit DTRXP __attribute__((address(0x7DC5)));


extern volatile __bit D_A __attribute__((address(0x7E3D)));


extern volatile __bit D_NOT_A __attribute__((address(0x7E3D)));


extern volatile __bit D_nA __attribute__((address(0x7E3D)));


extern volatile __bit EBDIS __attribute__((address(0x7E5F)));


extern volatile __bit ECCPAS0 __attribute__((address(0x7DB4)));


extern volatile __bit ECCPAS1 __attribute__((address(0x7DB5)));


extern volatile __bit ECCPAS2 __attribute__((address(0x7DB6)));


extern volatile __bit ECCPASE __attribute__((address(0x7DB7)));


extern volatile __bit EEADR0 __attribute__((address(0x7D48)));


extern volatile __bit EEADR1 __attribute__((address(0x7D49)));


extern volatile __bit EEADR2 __attribute__((address(0x7D4A)));


extern volatile __bit EEADR3 __attribute__((address(0x7D4B)));


extern volatile __bit EEADR4 __attribute__((address(0x7D4C)));


extern volatile __bit EEADR5 __attribute__((address(0x7D4D)));


extern volatile __bit EEADR6 __attribute__((address(0x7D4E)));


extern volatile __bit EEADR7 __attribute__((address(0x7D4F)));


extern volatile __bit EEFS __attribute__((address(0x7D36)));


extern volatile __bit EEIE __attribute__((address(0x7D04)));


extern volatile __bit EEIF __attribute__((address(0x7D0C)));


extern volatile __bit EEIP __attribute__((address(0x7D14)));


extern volatile __bit EEPGD __attribute__((address(0x7D37)));


extern volatile __bit ENDP0 __attribute__((address(0x7B1B)));


extern volatile __bit ENDP1 __attribute__((address(0x7B1C)));


extern volatile __bit ENDP2 __attribute__((address(0x7B1D)));


extern volatile __bit ENDP3 __attribute__((address(0x7B1E)));


extern volatile __bit EP0CONDIS __attribute__((address(0x7A9B)));


extern volatile __bit EP0HSHK __attribute__((address(0x7A9C)));


extern volatile __bit EP0INEN __attribute__((address(0x7A99)));


extern volatile __bit EP0OUTEN __attribute__((address(0x7A9A)));


extern volatile __bit EP0STALL __attribute__((address(0x7A98)));


extern volatile __bit EP1CONDIS __attribute__((address(0x7AA3)));


extern volatile __bit EP1HSHK __attribute__((address(0x7AA4)));


extern volatile __bit EP1INEN __attribute__((address(0x7AA1)));


extern volatile __bit EP1OUTEN __attribute__((address(0x7AA2)));


extern volatile __bit EP1STALL __attribute__((address(0x7AA0)));


extern volatile __bit EP2CONDIS __attribute__((address(0x7AAB)));


extern volatile __bit EP2HSHK __attribute__((address(0x7AAC)));


extern volatile __bit EP2INEN __attribute__((address(0x7AA9)));


extern volatile __bit EP2OUTEN __attribute__((address(0x7AAA)));


extern volatile __bit EP2STALL __attribute__((address(0x7AA8)));


extern volatile __bit EP3CONDIS __attribute__((address(0x7AB3)));


extern volatile __bit EP3HSHK __attribute__((address(0x7AB4)));


extern volatile __bit EP3INEN __attribute__((address(0x7AB1)));


extern volatile __bit EP3OUTEN __attribute__((address(0x7AB2)));


extern volatile __bit EP3STALL __attribute__((address(0x7AB0)));


extern volatile __bit EP4CONDIS __attribute__((address(0x7ABB)));


extern volatile __bit EP4HSHK __attribute__((address(0x7ABC)));


extern volatile __bit EP4INEN __attribute__((address(0x7AB9)));


extern volatile __bit EP4OUTEN __attribute__((address(0x7ABA)));


extern volatile __bit EP4STALL __attribute__((address(0x7AB8)));


extern volatile __bit EP5CONDIS __attribute__((address(0x7AC3)));


extern volatile __bit EP5HSHK __attribute__((address(0x7AC4)));


extern volatile __bit EP5INEN __attribute__((address(0x7AC1)));


extern volatile __bit EP5OUTEN __attribute__((address(0x7AC2)));


extern volatile __bit EP5STALL __attribute__((address(0x7AC0)));


extern volatile __bit EP6CONDIS __attribute__((address(0x7ACB)));


extern volatile __bit EP6HSHK __attribute__((address(0x7ACC)));


extern volatile __bit EP6INEN __attribute__((address(0x7AC9)));


extern volatile __bit EP6OUTEN __attribute__((address(0x7ACA)));


extern volatile __bit EP6STALL __attribute__((address(0x7AC8)));


extern volatile __bit EP7CONDIS __attribute__((address(0x7AD3)));


extern volatile __bit EP7HSHK __attribute__((address(0x7AD4)));


extern volatile __bit EP7INEN __attribute__((address(0x7AD1)));


extern volatile __bit EP7OUTEN __attribute__((address(0x7AD2)));


extern volatile __bit EP7STALL __attribute__((address(0x7AD0)));


extern volatile __bit __attribute__((__deprecated__)) EPCONDIS __attribute__((address(0x7A9B)));


extern volatile __bit EPCONDIS0 __attribute__((address(0x7A9B)));


extern volatile __bit EPCONDIS1 __attribute__((address(0x7AA3)));


extern volatile __bit EPCONDIS2 __attribute__((address(0x7AAB)));


extern volatile __bit EPCONDIS3 __attribute__((address(0x7AB3)));


extern volatile __bit EPCONDIS4 __attribute__((address(0x7ABB)));


extern volatile __bit EPCONDIS5 __attribute__((address(0x7AC3)));


extern volatile __bit EPCONDIS6 __attribute__((address(0x7ACB)));


extern volatile __bit EPCONDIS7 __attribute__((address(0x7AD3)));


extern volatile __bit __attribute__((__deprecated__)) EPHSHK __attribute__((address(0x7A9C)));


extern volatile __bit EPHSHK0 __attribute__((address(0x7A9C)));


extern volatile __bit EPHSHK1 __attribute__((address(0x7AA4)));


extern volatile __bit EPHSHK2 __attribute__((address(0x7AAC)));


extern volatile __bit EPHSHK3 __attribute__((address(0x7AB4)));


extern volatile __bit EPHSHK4 __attribute__((address(0x7ABC)));


extern volatile __bit EPHSHK5 __attribute__((address(0x7AC4)));


extern volatile __bit EPHSHK6 __attribute__((address(0x7ACC)));


extern volatile __bit EPHSHK7 __attribute__((address(0x7AD4)));


extern volatile __bit __attribute__((__deprecated__)) EPINEN __attribute__((address(0x7A99)));


extern volatile __bit EPINEN0 __attribute__((address(0x7A99)));


extern volatile __bit EPINEN1 __attribute__((address(0x7AA1)));


extern volatile __bit EPINEN2 __attribute__((address(0x7AA9)));


extern volatile __bit EPINEN3 __attribute__((address(0x7AB1)));


extern volatile __bit EPINEN4 __attribute__((address(0x7AB9)));


extern volatile __bit EPINEN5 __attribute__((address(0x7AC1)));


extern volatile __bit EPINEN6 __attribute__((address(0x7AC9)));


extern volatile __bit EPINEN7 __attribute__((address(0x7AD1)));


extern volatile __bit __attribute__((__deprecated__)) EPOUTEN __attribute__((address(0x7A9A)));


extern volatile __bit EPOUTEN0 __attribute__((address(0x7A9A)));


extern volatile __bit EPOUTEN1 __attribute__((address(0x7AA2)));


extern volatile __bit EPOUTEN2 __attribute__((address(0x7AAA)));


extern volatile __bit EPOUTEN3 __attribute__((address(0x7AB2)));


extern volatile __bit EPOUTEN4 __attribute__((address(0x7ABA)));


extern volatile __bit EPOUTEN5 __attribute__((address(0x7AC2)));


extern volatile __bit EPOUTEN6 __attribute__((address(0x7ACA)));


extern volatile __bit EPOUTEN7 __attribute__((address(0x7AD2)));


extern volatile __bit __attribute__((__deprecated__)) EPSTALL __attribute__((address(0x7A98)));


extern volatile __bit EPSTALL0 __attribute__((address(0x7A98)));


extern volatile __bit EPSTALL1 __attribute__((address(0x7AA0)));


extern volatile __bit EPSTALL2 __attribute__((address(0x7AA8)));


extern volatile __bit EPSTALL3 __attribute__((address(0x7AB0)));


extern volatile __bit EPSTALL4 __attribute__((address(0x7AB8)));


extern volatile __bit EPSTALL5 __attribute__((address(0x7AC0)));


extern volatile __bit EPSTALL6 __attribute__((address(0x7AC8)));


extern volatile __bit EPSTALL7 __attribute__((address(0x7AD0)));


extern volatile __bit FERR __attribute__((address(0x7D5A)));


extern volatile __bit FREE __attribute__((address(0x7D34)));


extern volatile __bit FRM0 __attribute__((address(0x7AE8)));


extern volatile __bit FRM1 __attribute__((address(0x7AE9)));


extern volatile __bit FRM10 __attribute__((address(0x7AF2)));


extern volatile __bit FRM2 __attribute__((address(0x7AEA)));


extern volatile __bit FRM3 __attribute__((address(0x7AEB)));


extern volatile __bit FRM4 __attribute__((address(0x7AEC)));


extern volatile __bit FRM5 __attribute__((address(0x7AED)));


extern volatile __bit FRM6 __attribute__((address(0x7AEE)));


extern volatile __bit FRM7 __attribute__((address(0x7AEF)));


extern volatile __bit FRM8 __attribute__((address(0x7AF0)));


extern volatile __bit FRM9 __attribute__((address(0x7AF1)));


extern volatile __bit FSEN __attribute__((address(0x7B0A)));


extern volatile __bit FVR1EN __attribute__((address(0x7DD7)));


extern volatile __bit FVR1S0 __attribute__((address(0x7DD4)));


extern volatile __bit FVR1S1 __attribute__((address(0x7DD5)));


extern volatile __bit FVR1ST __attribute__((address(0x7DD6)));


extern volatile __bit GCEN __attribute__((address(0x7E2F)));


extern volatile __bit GIE __attribute__((address(0x7F97)));


extern volatile __bit GIEH __attribute__((address(0x7F97)));


extern volatile __bit GIEL __attribute__((address(0x7F96)));


extern volatile __bit GIE_GIEH __attribute__((address(0x7F97)));


extern volatile __bit GO __attribute__((address(0x7E11)));


extern volatile __bit GODONE __attribute__((address(0x7E11)));


extern volatile __bit GO_DONE __attribute__((address(0x7E11)));


extern volatile __bit GO_NOT_DONE __attribute__((address(0x7E11)));


extern volatile __bit GO_nDONE __attribute__((address(0x7E11)));


extern volatile __bit HFIOFL __attribute__((address(0x7E91)));


extern volatile __bit IDLEIE __attribute__((address(0x7B04)));


extern volatile __bit IDLEIF __attribute__((address(0x7B14)));


extern volatile __bit IDLEN __attribute__((address(0x7E9F)));


extern volatile __bit INT0 __attribute__((address(0x7C10)));


extern volatile __bit INT0E __attribute__((address(0x7F94)));


extern volatile __bit INT0F __attribute__((address(0x7F91)));


extern volatile __bit INT0IE __attribute__((address(0x7F94)));


extern volatile __bit INT0IF __attribute__((address(0x7F91)));


extern volatile __bit INT1 __attribute__((address(0x7C11)));


extern volatile __bit INT1E __attribute__((address(0x7F83)));


extern volatile __bit INT1F __attribute__((address(0x7F80)));


extern volatile __bit INT1IE __attribute__((address(0x7F83)));


extern volatile __bit INT1IF __attribute__((address(0x7F80)));


extern volatile __bit INT1IP __attribute__((address(0x7F86)));


extern volatile __bit INT1P __attribute__((address(0x7F86)));


extern volatile __bit INT2 __attribute__((address(0x7C12)));


extern volatile __bit INT2E __attribute__((address(0x7F84)));


extern volatile __bit INT2F __attribute__((address(0x7F81)));


extern volatile __bit INT2IE __attribute__((address(0x7F84)));


extern volatile __bit INT2IF __attribute__((address(0x7F81)));


extern volatile __bit INT2IP __attribute__((address(0x7F87)));


extern volatile __bit INT2P __attribute__((address(0x7F87)));


extern volatile __bit INTEDG0 __attribute__((address(0x7F8E)));


extern volatile __bit INTEDG1 __attribute__((address(0x7F8D)));


extern volatile __bit INTEDG2 __attribute__((address(0x7F8C)));


extern volatile __bit INTSRC __attribute__((address(0x7CDF)));


extern volatile __bit IOCA0 __attribute__((address(0x7BC8)));


extern volatile __bit IOCA1 __attribute__((address(0x7BC9)));


extern volatile __bit IOCA3 __attribute__((address(0x7BCB)));


extern volatile __bit IOCA4 __attribute__((address(0x7BCC)));


extern volatile __bit IOCA5 __attribute__((address(0x7BCD)));


extern volatile __bit IOCB4 __attribute__((address(0x7BD4)));


extern volatile __bit IOCB5 __attribute__((address(0x7BD5)));


extern volatile __bit IOCB6 __attribute__((address(0x7BD6)));


extern volatile __bit IOCB7 __attribute__((address(0x7BD7)));


extern volatile __bit IOFS __attribute__((address(0x7E9A)));


extern volatile __bit IPEN __attribute__((address(0x7E87)));


extern volatile __bit IRCF0 __attribute__((address(0x7E9C)));


extern volatile __bit IRCF1 __attribute__((address(0x7E9D)));


extern volatile __bit IRCF2 __attribute__((address(0x7E9E)));


extern volatile __bit LA4 __attribute__((address(0x7C4C)));


extern volatile __bit LA5 __attribute__((address(0x7C4D)));


extern volatile __bit LATA4 __attribute__((address(0x7C4C)));


extern volatile __bit LATA5 __attribute__((address(0x7C4D)));


extern volatile __bit LATB4 __attribute__((address(0x7C54)));


extern volatile __bit LATB5 __attribute__((address(0x7C55)));


extern volatile __bit LATB6 __attribute__((address(0x7C56)));


extern volatile __bit LATB7 __attribute__((address(0x7C57)));


extern volatile __bit LATC0 __attribute__((address(0x7C58)));


extern volatile __bit LATC1 __attribute__((address(0x7C59)));


extern volatile __bit LATC2 __attribute__((address(0x7C5A)));


extern volatile __bit LATC3 __attribute__((address(0x7C5B)));


extern volatile __bit LATC4 __attribute__((address(0x7C5C)));


extern volatile __bit LATC5 __attribute__((address(0x7C5D)));


extern volatile __bit LATC6 __attribute__((address(0x7C5E)));


extern volatile __bit LATC7 __attribute__((address(0x7C5F)));


extern volatile __bit LB4 __attribute__((address(0x7C54)));


extern volatile __bit LB5 __attribute__((address(0x7C55)));


extern volatile __bit LB6 __attribute__((address(0x7C56)));


extern volatile __bit LB7 __attribute__((address(0x7C57)));


extern volatile __bit LC0 __attribute__((address(0x7C58)));


extern volatile __bit LC1 __attribute__((address(0x7C59)));


extern volatile __bit LC2 __attribute__((address(0x7C5A)));


extern volatile __bit LC3 __attribute__((address(0x7C5B)));


extern volatile __bit LC4 __attribute__((address(0x7C5C)));


extern volatile __bit LC5 __attribute__((address(0x7C5D)));


extern volatile __bit LC6 __attribute__((address(0x7C5E)));


extern volatile __bit LC7 __attribute__((address(0x7C5F)));


extern volatile __bit LFIOFS __attribute__((address(0x7E90)));


extern volatile __bit LVDIN __attribute__((address(0x7C05)));


extern volatile __bit MC1OUT __attribute__((address(0x7B67)));


extern volatile __bit MC2OUT __attribute__((address(0x7B66)));


extern volatile __bit MSK0 __attribute__((address(0x7B78)));


extern volatile __bit MSK1 __attribute__((address(0x7B79)));


extern volatile __bit MSK2 __attribute__((address(0x7B7A)));


extern volatile __bit MSK3 __attribute__((address(0x7B7B)));


extern volatile __bit MSK4 __attribute__((address(0x7B7C)));


extern volatile __bit MSK5 __attribute__((address(0x7B7D)));


extern volatile __bit MSK6 __attribute__((address(0x7B7E)));


extern volatile __bit MSK7 __attribute__((address(0x7B7F)));


extern volatile __bit NEGATIVE __attribute__((address(0x7EC4)));


extern volatile __bit NOT_A __attribute__((address(0x7E3D)));


extern volatile __bit NOT_ADDRESS __attribute__((address(0x7E3D)));


extern volatile __bit NOT_BOR __attribute__((address(0x7E80)));


extern volatile __bit NOT_DONE __attribute__((address(0x7E11)));


extern volatile __bit NOT_PD __attribute__((address(0x7E82)));


extern volatile __bit NOT_POR __attribute__((address(0x7E81)));


extern volatile __bit NOT_RABPU __attribute__((address(0x7F8F)));


extern volatile __bit NOT_RI __attribute__((address(0x7E84)));


extern volatile __bit NOT_SS __attribute__((address(0x7C16)));


extern volatile __bit NOT_T1SYNC __attribute__((address(0x7E6A)));


extern volatile __bit NOT_T3SYNC __attribute__((address(0x7D8A)));


extern volatile __bit NOT_TO __attribute__((address(0x7E83)));


extern volatile __bit NOT_W __attribute__((address(0x7E3A)));


extern volatile __bit NOT_WRITE __attribute__((address(0x7E3A)));


extern volatile __bit NVCFG0 __attribute__((address(0x7E08)));


extern volatile __bit NVCFG1 __attribute__((address(0x7E09)));


extern volatile __bit OERR __attribute__((address(0x7D59)));


extern volatile __bit OSC1 __attribute__((address(0x7C05)));


extern volatile __bit OSC2 __attribute__((address(0x7C04)));


extern volatile __bit OSCFIE __attribute__((address(0x7D07)));


extern volatile __bit OSCFIF __attribute__((address(0x7D0F)));


extern volatile __bit OSCFIP __attribute__((address(0x7D17)));


extern volatile __bit OSTS __attribute__((address(0x7E9B)));


extern volatile __bit OV __attribute__((address(0x7EC3)));


extern volatile __bit OVERFLOW __attribute__((address(0x7EC3)));


extern volatile __bit P1A __attribute__((address(0x7C15)));


extern volatile __bit P1B __attribute__((address(0x7C14)));


extern volatile __bit P1C __attribute__((address(0x7C13)));


extern volatile __bit P1D __attribute__((address(0x7C12)));


extern volatile __bit P1M0 __attribute__((address(0x7DEE)));


extern volatile __bit P1M1 __attribute__((address(0x7DEF)));


extern volatile __bit PA1 __attribute__((address(0x7C12)));


extern volatile __bit PA2 __attribute__((address(0x7C11)));


extern volatile __bit PD __attribute__((address(0x7E82)));


extern volatile __bit PDC0 __attribute__((address(0x7DB8)));


extern volatile __bit PDC1 __attribute__((address(0x7DB9)));


extern volatile __bit PDC2 __attribute__((address(0x7DBA)));


extern volatile __bit PDC3 __attribute__((address(0x7DBB)));


extern volatile __bit PDC4 __attribute__((address(0x7DBC)));


extern volatile __bit PDC5 __attribute__((address(0x7DBD)));


extern volatile __bit PDC6 __attribute__((address(0x7DBE)));


extern volatile __bit PEIE __attribute__((address(0x7F96)));


extern volatile __bit PEIE_GIEL __attribute__((address(0x7F96)));


extern volatile __bit PEN __attribute__((address(0x7E2A)));


extern volatile __bit PGM __attribute__((address(0x7C13)));


extern volatile __bit PIDEE __attribute__((address(0x7AD8)));


extern volatile __bit PIDEF __attribute__((address(0x7AF8)));


extern volatile __bit PKTDIS __attribute__((address(0x7B24)));


extern volatile __bit POR __attribute__((address(0x7E81)));


extern volatile __bit PPB0 __attribute__((address(0x7B08)));


extern volatile __bit PPB1 __attribute__((address(0x7B09)));


extern volatile __bit PPBI __attribute__((address(0x7B19)));


extern volatile __bit PPBRST __attribute__((address(0x7B26)));


extern volatile __bit PRI_SD __attribute__((address(0x7E92)));


extern volatile __bit PRSEN __attribute__((address(0x7DBF)));


extern volatile __bit PSA __attribute__((address(0x7EAB)));


extern volatile __bit PSSAC0 __attribute__((address(0x7DB2)));


extern volatile __bit PSSAC1 __attribute__((address(0x7DB3)));


extern volatile __bit PSSBD0 __attribute__((address(0x7DB0)));


extern volatile __bit PSSBD1 __attribute__((address(0x7DB1)));


extern volatile __bit PVCFG0 __attribute__((address(0x7E0A)));


extern volatile __bit PVCFG1 __attribute__((address(0x7E0B)));


extern volatile __bit RA0 __attribute__((address(0x7C00)));


extern volatile __bit RA1 __attribute__((address(0x7C01)));


extern volatile __bit RA3 __attribute__((address(0x7C03)));


extern volatile __bit __attribute__((__deprecated__)) RA4 __attribute__((address(0x7C04)));


extern volatile __bit __attribute__((__deprecated__)) RA5 __attribute__((address(0x7C05)));


extern volatile __bit RABIE __attribute__((address(0x7F93)));


extern volatile __bit RABIF __attribute__((address(0x7F90)));


extern volatile __bit RABIP __attribute__((address(0x7F88)));


extern volatile __bit RABPU __attribute__((address(0x7F8F)));


extern volatile __bit __attribute__((__deprecated__)) RB4 __attribute__((address(0x7C0C)));


extern volatile __bit __attribute__((__deprecated__)) RB5 __attribute__((address(0x7C0D)));


extern volatile __bit __attribute__((__deprecated__)) RB6 __attribute__((address(0x7C0E)));


extern volatile __bit __attribute__((__deprecated__)) RB7 __attribute__((address(0x7C0F)));


extern volatile __bit __attribute__((__deprecated__)) RC0 __attribute__((address(0x7C10)));


extern volatile __bit __attribute__((__deprecated__)) RC1 __attribute__((address(0x7C11)));


extern volatile __bit RC1IE __attribute__((address(0x7CED)));


extern volatile __bit RC1IF __attribute__((address(0x7CF5)));


extern volatile __bit RC1IP __attribute__((address(0x7CFD)));


extern volatile __bit __attribute__((__deprecated__)) RC2 __attribute__((address(0x7C12)));


extern volatile __bit __attribute__((__deprecated__)) RC3 __attribute__((address(0x7C13)));


extern volatile __bit __attribute__((__deprecated__)) RC4 __attribute__((address(0x7C14)));


extern volatile __bit __attribute__((__deprecated__)) RC5 __attribute__((address(0x7C15)));


extern volatile __bit __attribute__((__deprecated__)) RC6 __attribute__((address(0x7C16)));


extern volatile __bit __attribute__((__deprecated__)) RC7 __attribute__((address(0x7C17)));


extern volatile __bit RC8_9 __attribute__((address(0x7D5E)));


extern volatile __bit RC9 __attribute__((address(0x7D5E)));


extern volatile __bit RCD8 __attribute__((address(0x7D58)));


extern volatile __bit RCEN __attribute__((address(0x7E2B)));


extern volatile __bit RCIDL __attribute__((address(0x7DC6)));


extern volatile __bit RCIE __attribute__((address(0x7CED)));


extern volatile __bit RCIF __attribute__((address(0x7CF5)));


extern volatile __bit RCIP __attribute__((address(0x7CFD)));


extern volatile __bit RD __attribute__((address(0x7D30)));


extern volatile __bit RD163 __attribute__((address(0x7D8F)));


extern volatile __bit RESUME __attribute__((address(0x7B22)));


extern volatile __bit RI __attribute__((address(0x7E84)));


extern volatile __bit RSEN __attribute__((address(0x7E29)));


extern volatile __bit RW __attribute__((address(0x7E3A)));


extern volatile __bit RX __attribute__((address(0x7C0D)));


extern volatile __bit RX9 __attribute__((address(0x7D5E)));


extern volatile __bit RX9D __attribute__((address(0x7D58)));


extern volatile __bit RXCKP __attribute__((address(0x7DC5)));


extern volatile __bit R_NOT_W __attribute__((address(0x7E3A)));


extern volatile __bit R_W __attribute__((address(0x7E3A)));


extern volatile __bit R_nW __attribute__((address(0x7E3A)));


extern volatile __bit SBOREN __attribute__((address(0x7E86)));


extern volatile __bit SCK __attribute__((address(0x7C0E)));


extern volatile __bit SCKP __attribute__((address(0x7DC4)));


extern volatile __bit SCL __attribute__((address(0x7C0E)));


extern volatile __bit SCS __attribute__((address(0x7E98)));


extern volatile __bit SCS0 __attribute__((address(0x7E98)));


extern volatile __bit SCS1 __attribute__((address(0x7E99)));


extern volatile __bit SDA __attribute__((address(0x7C0C)));


extern volatile __bit SDI __attribute__((address(0x7C0C)));


extern volatile __bit SDO __attribute__((address(0x7C17)));


extern volatile __bit SE0 __attribute__((address(0x7B25)));


extern volatile __bit SEN __attribute__((address(0x7E28)));


extern volatile __bit SENDB __attribute__((address(0x7D63)));


extern volatile __bit SENDB1 __attribute__((address(0x7D63)));


extern volatile __bit SLRA __attribute__((address(0x7BB0)));


extern volatile __bit SLRB __attribute__((address(0x7BB1)));


extern volatile __bit SLRC __attribute__((address(0x7BB2)));


extern volatile __bit SMP __attribute__((address(0x7E3F)));


extern volatile __bit SOFIE __attribute__((address(0x7B06)));


extern volatile __bit SOFIF __attribute__((address(0x7B16)));


extern volatile __bit SOSCEN __attribute__((address(0x7E6B)));


extern volatile __bit SOSCEN3 __attribute__((address(0x7D8B)));


extern volatile __bit SP0 __attribute__((address(0x7FE0)));


extern volatile __bit SP1 __attribute__((address(0x7FE1)));


extern volatile __bit SP2 __attribute__((address(0x7FE2)));


extern volatile __bit SP3 __attribute__((address(0x7FE3)));


extern volatile __bit SP4 __attribute__((address(0x7FE4)));


extern volatile __bit SPEN __attribute__((address(0x7D5F)));


extern volatile __bit SPLLEN __attribute__((address(0x7CDE)));


extern volatile __bit SRCLK0 __attribute__((address(0x7B44)));


extern volatile __bit SRCLK1 __attribute__((address(0x7B45)));


extern volatile __bit SRCLK2 __attribute__((address(0x7B46)));


extern volatile __bit SREN __attribute__((address(0x7D5D)));


extern volatile __bit SRENA __attribute__((address(0x7D5D)));


extern volatile __bit SRLEN __attribute__((address(0x7B47)));


extern volatile __bit SRNQEN __attribute__((address(0x7B42)));


extern volatile __bit SRPR __attribute__((address(0x7B40)));


extern volatile __bit SRPS __attribute__((address(0x7B41)));


extern volatile __bit SRQ __attribute__((address(0x7C14)));


extern volatile __bit SRQEN __attribute__((address(0x7B43)));


extern volatile __bit SRRC1E __attribute__((address(0x7B48)));


extern volatile __bit SRRC2E __attribute__((address(0x7B49)));


extern volatile __bit SRRCKE __attribute__((address(0x7B4A)));


extern volatile __bit SRRPE __attribute__((address(0x7B4B)));


extern volatile __bit SRSC1E __attribute__((address(0x7B4C)));


extern volatile __bit SRSC2E __attribute__((address(0x7B4D)));


extern volatile __bit SRSCKE __attribute__((address(0x7B4E)));


extern volatile __bit SRSPE __attribute__((address(0x7B4F)));


extern volatile __bit SS __attribute__((address(0x7C16)));


extern volatile __bit SSPEN __attribute__((address(0x7E35)));


extern volatile __bit SSPIE __attribute__((address(0x7CEB)));


extern volatile __bit SSPIF __attribute__((address(0x7CF3)));


extern volatile __bit SSPIP __attribute__((address(0x7CFB)));


extern volatile __bit SSPM0 __attribute__((address(0x7E30)));


extern volatile __bit SSPM1 __attribute__((address(0x7E31)));


extern volatile __bit SSPM2 __attribute__((address(0x7E32)));


extern volatile __bit SSPM3 __attribute__((address(0x7E33)));


extern volatile __bit SSPOV __attribute__((address(0x7E36)));


extern volatile __bit STALLIE __attribute__((address(0x7B05)));


extern volatile __bit STALLIF __attribute__((address(0x7B15)));


extern volatile __bit START __attribute__((address(0x7E3B)));


extern volatile __bit STKFUL __attribute__((address(0x7FE7)));


extern volatile __bit STKOVF __attribute__((address(0x7FE7)));


extern volatile __bit STKUNF __attribute__((address(0x7FE6)));


extern volatile __bit STOP __attribute__((address(0x7E3C)));


extern volatile __bit STRA __attribute__((address(0x7DC8)));


extern volatile __bit STRB __attribute__((address(0x7DC9)));


extern volatile __bit STRC __attribute__((address(0x7DCA)));


extern volatile __bit STRD __attribute__((address(0x7DCB)));


extern volatile __bit STRSYNC __attribute__((address(0x7DCC)));


extern volatile __bit SUSPND __attribute__((address(0x7B21)));


extern volatile __bit SWDTE __attribute__((address(0x7E88)));


extern volatile __bit SWDTEN __attribute__((address(0x7E88)));


extern volatile __bit SYNC __attribute__((address(0x7D64)));


extern volatile __bit SYNC1 __attribute__((address(0x7D64)));


extern volatile __bit T08BIT __attribute__((address(0x7EAE)));


extern volatile __bit T0CKI __attribute__((address(0x7C15)));


extern volatile __bit T0CS __attribute__((address(0x7EAD)));


extern volatile __bit T0IE __attribute__((address(0x7F95)));


extern volatile __bit T0IF __attribute__((address(0x7F92)));


extern volatile __bit T0PS0 __attribute__((address(0x7EA8)));


extern volatile __bit T0PS1 __attribute__((address(0x7EA9)));


extern volatile __bit T0PS2 __attribute__((address(0x7EAA)));


extern volatile __bit T0SE __attribute__((address(0x7EAC)));


extern volatile __bit T13CKI __attribute__((address(0x7C16)));


extern volatile __bit T1CKPS0 __attribute__((address(0x7E6C)));


extern volatile __bit T1CKPS1 __attribute__((address(0x7E6D)));


extern volatile __bit T1OSCEN __attribute__((address(0x7E6B)));


extern volatile __bit T1OSCI __attribute__((address(0x7C16)));


extern volatile __bit T1OSCO __attribute__((address(0x7C17)));


extern volatile __bit T1RD16 __attribute__((address(0x7E6F)));


extern volatile __bit T1RUN __attribute__((address(0x7E6E)));


extern volatile __bit T1SYNC __attribute__((address(0x7E6A)));


extern volatile __bit T2CKPS0 __attribute__((address(0x7E50)));


extern volatile __bit T2CKPS1 __attribute__((address(0x7E51)));


extern volatile __bit T2OUTPS0 __attribute__((address(0x7E53)));


extern volatile __bit T2OUTPS1 __attribute__((address(0x7E54)));


extern volatile __bit T2OUTPS2 __attribute__((address(0x7E55)));


extern volatile __bit T2OUTPS3 __attribute__((address(0x7E56)));


extern volatile __bit T3CCP1 __attribute__((address(0x7D8B)));


extern volatile __bit T3CKPS0 __attribute__((address(0x7D8C)));


extern volatile __bit T3CKPS1 __attribute__((address(0x7D8D)));


extern volatile __bit T3RD16 __attribute__((address(0x7D8F)));


extern volatile __bit T3SYNC __attribute__((address(0x7D8A)));


extern volatile __bit TMR0IE __attribute__((address(0x7F95)));


extern volatile __bit TMR0IF __attribute__((address(0x7F92)));


extern volatile __bit TMR0IP __attribute__((address(0x7F8A)));


extern volatile __bit TMR0ON __attribute__((address(0x7EAF)));


extern volatile __bit TMR1CS __attribute__((address(0x7E69)));


extern volatile __bit TMR1IE __attribute__((address(0x7CE8)));


extern volatile __bit TMR1IF __attribute__((address(0x7CF0)));


extern volatile __bit TMR1IP __attribute__((address(0x7CF8)));


extern volatile __bit TMR1ON __attribute__((address(0x7E68)));


extern volatile __bit TMR2IE __attribute__((address(0x7CE9)));


extern volatile __bit TMR2IF __attribute__((address(0x7CF1)));


extern volatile __bit TMR2IP __attribute__((address(0x7CF9)));


extern volatile __bit TMR2ON __attribute__((address(0x7E52)));


extern volatile __bit TMR3CS __attribute__((address(0x7D89)));


extern volatile __bit TMR3IE __attribute__((address(0x7D01)));


extern volatile __bit TMR3IF __attribute__((address(0x7D09)));


extern volatile __bit TMR3IP __attribute__((address(0x7D11)));


extern volatile __bit TMR3ON __attribute__((address(0x7D88)));


extern volatile __bit TO __attribute__((address(0x7E83)));


extern volatile __bit TRISA4 __attribute__((address(0x7C94)));


extern volatile __bit TRISA5 __attribute__((address(0x7C95)));


extern volatile __bit TRISB4 __attribute__((address(0x7C9C)));


extern volatile __bit TRISB5 __attribute__((address(0x7C9D)));


extern volatile __bit TRISB6 __attribute__((address(0x7C9E)));


extern volatile __bit TRISB7 __attribute__((address(0x7C9F)));


extern volatile __bit TRISC0 __attribute__((address(0x7CA0)));


extern volatile __bit TRISC1 __attribute__((address(0x7CA1)));


extern volatile __bit TRISC2 __attribute__((address(0x7CA2)));


extern volatile __bit TRISC3 __attribute__((address(0x7CA3)));


extern volatile __bit TRISC4 __attribute__((address(0x7CA4)));


extern volatile __bit TRISC5 __attribute__((address(0x7CA5)));


extern volatile __bit TRISC6 __attribute__((address(0x7CA6)));


extern volatile __bit TRISC7 __attribute__((address(0x7CA7)));


extern volatile __bit TRMT __attribute__((address(0x7D61)));


extern volatile __bit TRMT1 __attribute__((address(0x7D61)));


extern volatile __bit TRNIE __attribute__((address(0x7B03)));


extern volatile __bit TRNIF __attribute__((address(0x7B13)));


extern volatile __bit TUN0 __attribute__((address(0x7CD8)));


extern volatile __bit TUN1 __attribute__((address(0x7CD9)));


extern volatile __bit TUN2 __attribute__((address(0x7CDA)));


extern volatile __bit TUN3 __attribute__((address(0x7CDB)));


extern volatile __bit TUN4 __attribute__((address(0x7CDC)));


extern volatile __bit TUN5 __attribute__((address(0x7CDD)));


extern volatile __bit TX __attribute__((address(0x7C0F)));


extern volatile __bit TX1IE __attribute__((address(0x7CEC)));


extern volatile __bit TX1IF __attribute__((address(0x7CF4)));


extern volatile __bit TX1IP __attribute__((address(0x7CFC)));


extern volatile __bit TX8_9 __attribute__((address(0x7D66)));


extern volatile __bit TX9 __attribute__((address(0x7D66)));


extern volatile __bit TX91 __attribute__((address(0x7D66)));


extern volatile __bit TX9D __attribute__((address(0x7D60)));


extern volatile __bit TX9D1 __attribute__((address(0x7D60)));


extern volatile __bit TXD8 __attribute__((address(0x7D60)));


extern volatile __bit TXEN __attribute__((address(0x7D65)));


extern volatile __bit TXEN1 __attribute__((address(0x7D65)));


extern volatile __bit TXIE __attribute__((address(0x7CEC)));


extern volatile __bit TXIF __attribute__((address(0x7CF4)));


extern volatile __bit TXIP __attribute__((address(0x7CFC)));


extern volatile __bit UA __attribute__((address(0x7E39)));


extern volatile __bit UERRIE __attribute__((address(0x7B01)));


extern volatile __bit UERRIF __attribute__((address(0x7B11)));


extern volatile __bit ULPWUIN __attribute__((address(0x7C00)));


extern volatile __bit UPP0 __attribute__((address(0x7B08)));


extern volatile __bit UPP1 __attribute__((address(0x7B09)));


extern volatile __bit UPUEN __attribute__((address(0x7B0C)));


extern volatile __bit URSTIE __attribute__((address(0x7B00)));


extern volatile __bit URSTIF __attribute__((address(0x7B10)));


extern volatile __bit USBEN __attribute__((address(0x7B23)));


extern volatile __bit USBIE __attribute__((address(0x7D02)));


extern volatile __bit USBIF __attribute__((address(0x7D0A)));


extern volatile __bit USBIP __attribute__((address(0x7D12)));


extern volatile __bit UTEYE __attribute__((address(0x7B0F)));


extern volatile __bit VREFM __attribute__((address(0x7C11)));


extern volatile __bit VREFP __attribute__((address(0x7C10)));


extern volatile __bit W4E __attribute__((address(0x7DC1)));


extern volatile __bit WAIT0 __attribute__((address(0x7E5C)));


extern volatile __bit WAIT1 __attribute__((address(0x7E5D)));


extern volatile __bit WCOL __attribute__((address(0x7E37)));


extern volatile __bit WM0 __attribute__((address(0x7E58)));


extern volatile __bit WM1 __attribute__((address(0x7E59)));


extern volatile __bit WPUA3 __attribute__((address(0x7BBB)));


extern volatile __bit WPUA4 __attribute__((address(0x7BBC)));


extern volatile __bit WPUA5 __attribute__((address(0x7BBD)));


extern volatile __bit WPUB4 __attribute__((address(0x7BC4)));


extern volatile __bit WPUB5 __attribute__((address(0x7BC5)));


extern volatile __bit WPUB6 __attribute__((address(0x7BC6)));


extern volatile __bit WPUB7 __attribute__((address(0x7BC7)));


extern volatile __bit WR __attribute__((address(0x7D31)));


extern volatile __bit WREN __attribute__((address(0x7D32)));


extern volatile __bit WRERR __attribute__((address(0x7D33)));


extern volatile __bit WUE __attribute__((address(0x7DC1)));


extern volatile __bit ZERO __attribute__((address(0x7EC2)));


extern volatile __bit nA __attribute__((address(0x7E3D)));


extern volatile __bit nADDRESS __attribute__((address(0x7E3D)));


extern volatile __bit nBOR __attribute__((address(0x7E80)));


extern volatile __bit nDONE __attribute__((address(0x7E11)));


extern volatile __bit nPD __attribute__((address(0x7E82)));


extern volatile __bit nPOR __attribute__((address(0x7E81)));


extern volatile __bit nRABPU __attribute__((address(0x7F8F)));


extern volatile __bit nRI __attribute__((address(0x7E84)));


extern volatile __bit nSS __attribute__((address(0x7C16)));


extern volatile __bit nT1SYNC __attribute__((address(0x7E6A)));


extern volatile __bit nT3SYNC __attribute__((address(0x7D8A)));


extern volatile __bit nTO __attribute__((address(0x7E83)));


extern volatile __bit nW __attribute__((address(0x7E3A)));


extern volatile __bit nWRITE __attribute__((address(0x7E3A)));
# 60 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18_chip_select.h" 2 3
# 9 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18.h" 2 3
# 19 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18.h" 3
__attribute__((__unsupported__("The " "flash_write" " routine is no longer supported. Please use the MPLAB X MCC."))) void flash_write(const unsigned char *, unsigned int, __far unsigned char *);
__attribute__((__unsupported__("The " "EraseFlash" " routine is no longer supported. Please use the MPLAB X MCC."))) void EraseFlash(unsigned long startaddr, unsigned long endaddr);







# 1 "/opt/microchip/xc8/v2.50/pic/include/errata.h" 1 3
# 28 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18.h" 2 3
# 156 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18.h" 3
__attribute__((__unsupported__("The " "Read_b_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) unsigned char Read_b_eep(unsigned int badd);
__attribute__((__unsupported__("The " "Busy_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) void Busy_eep(void);
__attribute__((__unsupported__("The " "Write_b_eep" " routine is no longer supported. Please use the MPLAB X MCC."))) void Write_b_eep(unsigned int badd, unsigned char bdat);
# 192 "/opt/microchip/mplabx/v6.20/packs/Microchip/PIC18F-K_DFP/1.13.292/xc8/pic/include/pic18.h" 3
unsigned char __t1rd16on(void);
unsigned char __t3rd16on(void);
# 34 "/opt/microchip/xc8/v2.50/pic/include/xc.h" 2 3
# 70 "main.c" 2

# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/stdio.h" 1 3
# 24 "/opt/microchip/xc8/v2.50/pic/include/c99/stdio.h" 3
# 1 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 1 3
# 12 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef void * va_list[1];




typedef void * __isoc_va_list[1];
# 143 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef __int24 ssize_t;
# 255 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef long long off_t;
# 409 "/opt/microchip/xc8/v2.50/pic/include/c99/bits/alltypes.h" 3
typedef struct _IO_FILE FILE;
# 25 "/opt/microchip/xc8/v2.50/pic/include/c99/stdio.h" 2 3
# 52 "/opt/microchip/xc8/v2.50/pic/include/c99/stdio.h" 3
typedef union _G_fpos64_t {
 char __opaque[16];
 double __align;
} fpos_t;

extern FILE *const stdin;
extern FILE *const stdout;
extern FILE *const stderr;





FILE *fopen(const char *restrict, const char *restrict);
FILE *freopen(const char *restrict, const char *restrict, FILE *restrict);
int fclose(FILE *);

int remove(const char *);
int rename(const char *, const char *);

int feof(FILE *);
int ferror(FILE *);
int fflush(FILE *);
void clearerr(FILE *);

int fseek(FILE *, long, int);
long ftell(FILE *);
void rewind(FILE *);

int fgetpos(FILE *restrict, fpos_t *restrict);
int fsetpos(FILE *, const fpos_t *);

size_t fread(void *restrict, size_t, size_t, FILE *restrict);
size_t fwrite(const void *restrict, size_t, size_t, FILE *restrict);

int fgetc(FILE *);
int getc(FILE *);
int getchar(void);





int ungetc(int, FILE *);
int getch(void);

int fputc(int, FILE *);
int putc(int, FILE *);
int putchar(int);





void putch(char);

char *fgets(char *restrict, int, FILE *restrict);

char *gets(char *);


int fputs(const char *restrict, FILE *restrict);
int puts(const char *);

__attribute__((__format__(__printf__, 1, 2)))
int printf(const char *restrict, ...);
__attribute__((__format__(__printf__, 2, 3)))
int fprintf(FILE *restrict, const char *restrict, ...);
__attribute__((__format__(__printf__, 2, 3)))
int sprintf(char *restrict, const char *restrict, ...);
__attribute__((__format__(__printf__, 3, 4)))
int snprintf(char *restrict, size_t, const char *restrict, ...);

__attribute__((__format__(__printf__, 1, 0)))
int vprintf(const char *restrict, __isoc_va_list);
int vfprintf(FILE *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__printf__, 2, 0)))
int vsprintf(char *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__printf__, 3, 0)))
int vsnprintf(char *restrict, size_t, const char *restrict, __isoc_va_list);

__attribute__((__format__(__scanf__, 1, 2)))
int scanf(const char *restrict, ...);
__attribute__((__format__(__scanf__, 2, 3)))
int fscanf(FILE *restrict, const char *restrict, ...);
__attribute__((__format__(__scanf__, 2, 3)))
int sscanf(const char *restrict, const char *restrict, ...);

__attribute__((__format__(__scanf__, 1, 0)))
int vscanf(const char *restrict, __isoc_va_list);
int vfscanf(FILE *restrict, const char *restrict, __isoc_va_list);
__attribute__((__format__(__scanf__, 2, 0)))
int vsscanf(const char *restrict, const char *restrict, __isoc_va_list);

void perror(const char *);

int setvbuf(FILE *restrict, char *restrict, int, size_t);
void setbuf(FILE *restrict, char *restrict);

char *tmpnam(char *);
FILE *tmpfile(void);




FILE *fmemopen(void *restrict, size_t, const char *restrict);
FILE *open_memstream(char **, size_t *);
FILE *fdopen(int, const char *);
FILE *popen(const char *, const char *);
int pclose(FILE *);
int fileno(FILE *);
int fseeko(FILE *, off_t, int);
off_t ftello(FILE *);
int dprintf(int, const char *restrict, ...);
int vdprintf(int, const char *restrict, __isoc_va_list);
void flockfile(FILE *);
int ftrylockfile(FILE *);
void funlockfile(FILE *);
int getc_unlocked(FILE *);
int getchar_unlocked(void);
int putc_unlocked(int, FILE *);
int putchar_unlocked(int);
ssize_t getdelim(char **restrict, size_t *restrict, int, FILE *restrict);
ssize_t getline(char **restrict, size_t *restrict, FILE *restrict);
int renameat(int, const char *, int, const char *);
char *ctermid(char *);







char *tempnam(const char *, const char *);
# 72 "main.c" 2
# 88 "main.c"
uint16_t adcResult = 0;

uint8_t overflowCount = 0;
uint8_t sampling = 0;

uint8_t eeprom_data_lsb = 0;
uint8_t eeprom_data_msb = 0;
uint8_t eeprom_data_ctl = 0;
uint16_t eeprom_data = 0;

uint8_t received_data = 0;

uint8_t address_low_byte = 0;
uint8_t address_high_byte = 0;
uint16_t address_index = 0;

uint8_t segPattern0;
uint8_t segPattern1;
uint8_t segPattern2;

uint8_t displayDigit0;
uint8_t displayDigit1;
uint8_t displayDigit2;

uint8_t turnOnDisplay = 0;
uint8_t displayAddress = 0;
uint8_t displaySample = 0;

uint8_t fsm_state = 0;

double adc_step_size = 0.0048875855;
double adc_voltage = 0;

unsigned int c_target = 0;
unsigned int c_remainder = 0;
unsigned int c_number = 0;

char uart_buffer[10];
unsigned int button_counter = 0;




uint8_t getLowByte(uint16_t number){
    return ( number & 0xFF);
}

uint8_t getHighByte(uint16_t number){
    return ((number >> 8) & 0xFF);
}

void adcSampling(){

    sampling = 1;


    ADCON0bits.GO = 1;

    while (ADCON0bits.GO);

    adcResult = ((uint16_t)ADRESH << 8) | ADRESL;

}




void setupADC(void);
void setupTimer1(void);
void setupI2C(void);

void setButtonInterrupt(void);
uint8_t get7SegmentPattern(uint8_t);
void displayDigits( uint8_t, uint8_t, uint8_t );
void show7Segment(void);
void setDisplayDigits(void);

void I2CStart(void);
void I2CStop(void);
void I2CRestart(void);
void I2CWrite( uint8_t );
uint8_t I2CRead(uint8_t );
void EEPROMWrite(uint16_t , uint8_t );
uint8_t EEPROMRead(uint16_t );

void setupEUSART(void);
void EUSART_TransmitChar(char);
void EUSART_TransmitString( char *);

void __attribute__((picinterrupt(("")))) ISR(void) {

    if ( PIR1bits.TMR1IF) {
        PIR1bits.TMR1IF = 0;

        TMR1 = 0;

        if (fsm_state == 1 || fsm_state == 2 ) {

            overflowCount++;

            if (overflowCount >= 57) {
                overflowCount = 0;


                adcSampling();

            }
        }
        else if( fsm_state == 7 ){
            LATAbits.LA5 ^= 1;
        }
    }


    if ( INTCON3bits.INT1IF || INTCON3bits.INT2IF ) {

        button_counter++;

        if( INTCON3bits.INT2IF && button_counter > 800 ){

            INTCON3bits.INT2IF = 0;

            displayAddress = 1;

            displaySample = 1;
            button_counter = 0;

            switch(fsm_state){
                case 0 : {
                    fsm_state = 1;
                    address_index = 0;
                    if( turnOnDisplay == 0 ){
                        adcSampling();
                    }
                    break;
                }
                case 1 : {
                    fsm_state = 2;
                    address_index = 0;
                    break;
                }
                case 2 : {
                    fsm_state = 3;
                    address_index = 0;
                    EUSART_TransmitString("START \r\n");
                    break;
                }
                case 3 : {
                    fsm_state = 4;
                    break;
                }
                case 4 : {
                    fsm_state = 0;
                    break;
                }
                case 5 : {
                    fsm_state = 0;
                    break;
                }
                case 6 : {
                    fsm_state = 0;
                    break;
                }
                case 7 : {
                    fsm_state = 0;
                    break;
                }
                default : break;
            }

        }else if( INTCON3bits.INT1IF && button_counter > 800 ){

            INTCON3bits.INT1IF = 0;

            displayAddress = 0;
            displaySample = 1;
            button_counter = 0;


            switch(fsm_state){
                case 4 : {
                    fsm_state = 5;
                    break;
                }
                case 5 : {
                    fsm_state = 6;
                    break;
                }
                case 6 : {
                    fsm_state = 7;
                    break;
                }

                default : break;
            }
        }

        if( turnOnDisplay == 1 && button_counter < 10 ){
            setDisplayDigits();
            show7Segment();
        }else{
            turnOnDisplay = 1;
        }
    }
}


void main(void) {


    OSCCONbits.SCS0 = 1;
    OSCCONbits.SCS1 = 1;

    OSCCONbits.OSTS = 0;


    OSCCONbits.IRCF0 = 0;
    OSCCONbits.IRCF1 = 0;
    OSCCONbits.IRCF2 = 1;

    _delay((unsigned long)((500)*(2000000/4000.0)));


    ANSEL = 0b00001000;
    TRISA = 0b00010000;

    TRISCbits.RC0 = 0;
    ANSELbits.ANS4 = 0;

    setupADC();

    _delay((unsigned long)((100)*(2000000/4000000.0)));


    setupTimer1();

    setupI2C();



    TRISCbits.TRISC5 = 0;
    TRISCbits.TRISC4 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;

    LATCbits.LATC5 = 0;
    LATCbits.LATC4 = 0;
    LATCbits.LATC3 = 0;
    LATCbits.LATC6 = 0;
    LATCbits.LATC7 = 0;


    TRISCbits.RC1 = 1;
    TRISCbits.RC2 = 1;
    setButtonInterrupt();


    setupEUSART();

    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    adcSampling();

    while(1){

        if( sampling == 1 && fsm_state == 1 ){

            LATAbits.LA5 = 1;

            adc_voltage = adc_step_size * adcResult;


            address_index++;
            EEPROMWrite(address_index, getLowByte( adcResult) );
            address_index++;
            EEPROMWrite(address_index, getHighByte( adcResult) );
            address_index++;
            EEPROMWrite(address_index, 254 );

            if( address_index >= 8100 ){




                fsm_state = 0;
                address_index = 0;
            }

            _delay((unsigned long)((1000)*(2000000/4000.0)));

            sampling = 0;
            LATAbits.LA5 = 0;

        }else if( sampling == 1 && fsm_state == 2 ){
            LATAbits.LA5 = 1;
            adc_voltage = adc_step_size * adcResult;



            sprintf(uart_buffer, "%.2f", adc_voltage);

            EUSART_TransmitString(uart_buffer);
            EUSART_TransmitString("\r\n");

            _delay((unsigned long)((1000)*(2000000/4000.0)));

            sampling = 0;
            LATAbits.LA5 = 0;

        }

        if( fsm_state == 3 ){

            LATAbits.LA5 = 1;

            if( address_index == 0){
                EUSART_TransmitString("START \r\n");
                address_index++;

            }else if(address_index >= 7200 ){
                EUSART_TransmitString("END \r\n");
                address_index = 0;

            }else {
                eeprom_data_lsb = EEPROMRead(address_index);
                address_index++;
                eeprom_data_msb = EEPROMRead(address_index);
                address_index++;
                eeprom_data_ctl = EEPROMRead(address_index);
                address_index++;

                if( eeprom_data_ctl == 254 ){


                    eeprom_data = ((uint16_t) eeprom_data_msb << 8) | eeprom_data_lsb;

                    adc_voltage = adc_step_size * eeprom_data;


                    sprintf(uart_buffer, "%.2f", adc_voltage);

                    EUSART_TransmitString(uart_buffer);
                    EUSART_TransmitString("\r\n");
                }else{
                    address_index = 7200;
                }
            }
        }else{

            if( fsm_state < 7){
                LATAbits.LA5 = 0;
            }
        }

        if( turnOnDisplay == 1 ){

            PORTCbits.RC0 = 0;

            setDisplayDigits();
            show7Segment();

            _delay((unsigned long)((10000)*(2000000/4000.0)));

            turnOnDisplay = 0;

        }else{

            PORTCbits.RC0 = 1;

             _delay((unsigned long)((1000)*(2000000/4000.0)));
        }
    }

    return;
}


void setupADC() {
    ADCON1 = 0b00000000;
    ADCON0 = 0b00001101;
    ADCON2 = 0b10101111;
}

void setupTimer1(void) {
    T1CON = 0b00110101;
    TMR1 = 0;


    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;





    TMR1 = 0;
}

void setupI2C() {
    TRISBbits.RB4 = 1;
    TRISBbits.RB6 = 1;
    SSPCON1 = 0b00101000;
    SSPADD = ((2000000 / 4) / 100000) - 1;
    SSPSTAT = 0x00;

    SSPCON2 = 0b00000000;
}


void setButtonInterrupt(){

    INTCON2bits.INTEDG1 = 1;
    INTCON2bits.INTEDG2 = 1;

    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT2IE = 1;

    INTCON3bits.INT1IP = 0;
    INTCON3bits.INT2IP = 0;


    INTCON3bits.INT1IF = 0;
    INTCON3bits.INT2IF = 0;

}

uint8_t get7SegmentPattern(uint8_t digit) {


    uint8_t pattern;

    switch (digit) {
        case 0: pattern = 0b00111111; break;
        case 1: pattern = 0b00000110; break;
        case 2: pattern = 0b01011011; break;
        case 3: pattern = 0b01001111; break;
        case 4: pattern = 0b01100110; break;
        case 5: pattern = 0b01101101; break;
        case 6: pattern = 0b01111101; break;
        case 7: pattern = 0b00000111; break;
        case 8: pattern = 0b01111111; break;
        case 9: pattern = 0b01101111; break;
        case 89: pattern = 0b01111001; break;
        case 'A': pattern = 0b01110111; break;
        case 'C': pattern = 0b00111001; break;
        case 'S': pattern = 0b01101101; break;
        case 'L': pattern = 0b00111000; break;
        case 'r': pattern = 0b00110001; break;
        default: pattern = 0b00000000; break;
    }

    return pattern;
}

void displayDigits( uint8_t digit0, uint8_t digit1, uint8_t digit2 ){
    int i;
    for ( i = 0 ; i < 8 ; i++ ){

        PORTCbits.RC5 = (digit0 >> i) & (0x01);
        PORTCbits.RC4 = (digit1 >> i) & (0x01);
        PORTCbits.RC3 = (digit2 >> i) & (0x01);


        PORTCbits.RC6 = 1;
        _delay((unsigned long)((500)*(2000000/4000000.0)));
        PORTCbits.RC6 = 0;
        _delay((unsigned long)((500)*(2000000/4000000.0)));
    }


    PORTCbits.RC7 = 1;
    _delay((unsigned long)((500)*(2000000/4000000.0)));
    PORTCbits.RC7 = 0;

}

void I2CStart() {
    SSPCON2bits.SEN = 1;
    while (SSPCON2bits.SEN);
}

void I2CStop() {
    SSPCON2bits.PEN = 1;
    while (SSPCON2bits.PEN);
}

void I2CRestart() {
    SSPCON2bits.RSEN = 1;
    while (SSPCON2bits.RSEN);
}

void I2CWrite( uint8_t data) {
    SSPBUF = data;
    while (SSPSTATbits.BF);
    while (SSPCON2bits.ACKSTAT);
}

uint8_t I2CRead(uint8_t ack) {
    SSPCON2bits.RCEN = 1;
    while (!SSPSTATbits.BF);
    received_data = SSPBUF;

    SSPCON2bits.ACKDT = ack;
    SSPCON2bits.ACKEN = 1;
    while (SSPCON2bits.ACKEN);
    return received_data;
}

void EEPROMWrite(uint16_t address, uint8_t data) {
    I2CStart();
    I2CWrite(0b10100000);

    address_low_byte = getLowByte(address);
    address_high_byte = getHighByte(address);

    I2CWrite(address_low_byte);
    I2CWrite(address_high_byte);
    I2CWrite(data);
    I2CStop();

    _delay((unsigned long)((5)*(2000000/4000.0)));
}

uint8_t EEPROMRead(uint16_t address ) {
    uint8_t data;

    address_low_byte = getLowByte(address);
    address_high_byte = getHighByte(address);

    I2CStart();
    I2CWrite(0b10100000);
    I2CWrite(address_low_byte);
    I2CWrite(address_high_byte);
    I2CRestart();
    I2CWrite(0b10100000 | 1);
    data = I2CRead(1);
    I2CStop();
    return data;
}

void setupEUSART() {



    BAUDCONbits.BRG16 = 0;
    BAUDCONbits.ABDEN = 0;
    BAUDCONbits.CKTXP = 0;
    BAUDCONbits.ABDOVF = 0;


    SPBRG = 12;

    TXSTAbits.BRGH = 0;

    TXSTAbits.SYNC = 0;
    TXSTAbits.TX9 = 0;
    RCSTAbits.SPEN = 1;
    TXSTAbits.TXEN = 1;
    RCSTAbits.CREN = 0;

    TXSTAbits.SENDB = 0;


    TRISBbits.TRISB7 = 1;
    TRISBbits.TRISB5 = 1;
}


void EUSART_TransmitChar(char data) {
    while (!TXSTAbits.TRMT);
    TXREG = data;
}

void EUSART_TransmitString( char *a){
    int i;
    for( i= 0; a[i] !='\0'; i++ ){
       EUSART_TransmitChar(a[i]);
    }
}

void setDisplayDigits() {
    if (fsm_state == 0) {

        displayDigit0 = 11;
        displayDigit1 = 'A';
        displayDigit2 = 11;

    } else if (fsm_state == 1) {

        c_target = (unsigned int) (adc_voltage * 100);
        c_remainder = c_target % 100;
        c_number = c_target / 100;

        displayDigit0 = (uint8_t) c_number;
        displayDigit1 = (uint8_t) (c_remainder / 10);
        displayDigit2 = c_remainder % 10;

    } else if (fsm_state == 2) {

        c_target = (unsigned int) (adc_voltage * 100);
        c_remainder = c_target % 100;
        c_number = c_target / 100;

        displayDigit0 = (uint8_t) c_number;
        displayDigit1 = (uint8_t) (c_remainder / 10);
        displayDigit2 = 'C';

    } else if (fsm_state == 3) {

        displayDigit0 = 11;
        displayDigit1 = 'C';
        displayDigit2 = 11;

    } else if (fsm_state == 4) {

        displayDigit0 = 'S';
        displayDigit1 = 0;
        displayDigit2 = 11;

    } else if (fsm_state == 5) {

        displayDigit0 = 'S';
        displayDigit1 = 1;
        displayDigit2 = 11;

    } else if (fsm_state == 6) {

        displayDigit0 = 'S';
        displayDigit1 = 2;
        displayDigit2 = 11;

    } else if (fsm_state == 7) {

        displayDigit0 = 'C';
        displayDigit1 = 'L';
        displayDigit2 = 'r';

    }
    else {
        __nop();
    }
}

void show7Segment(){

    segPattern0 = get7SegmentPattern( displayDigit0 );

    if( displaySample == 1 && ( fsm_state == 1 || fsm_state == 2 ) ){
        segPattern0 = segPattern0 | 0b10000000;
    }

    segPattern1 = get7SegmentPattern( displayDigit1 );
    segPattern2 = get7SegmentPattern( displayDigit2 );

    displayDigits( segPattern0, segPattern1, segPattern2 );
}
