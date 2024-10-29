//----------------------------------------------------------------------------

#ifndef TYPES_H
#define TYPES_H

//----------------------------------------------------------------------------
			// datatypes
//----------------------------------------------------------------------------
typedef unsigned char   uint8_t;
typedef unsigned short  uint16_t;
typedef unsigned int    uint32_t;
typedef unsigned long   ulong_t;
typedef signed char     int8_t;
typedef signed short    int16_t;
typedef signed int      int32_t;
typedef signed long     long_t;

typedef enum 
{
	false = 0,
	true = 1
}bool_e; 

#endif

//-----------------------------------------------------------------------------
// \file    evmc6748_pll.h
// \brief   C6748 phase locked loop controller registers & bit definitions.
//
//-----------------------------------------------------------------------------

#ifndef EVMC6748_PLL_H
#define EVMC6748_PLL_H

//-----------------------------------------------------------------------------
// Register Structure & Defines
//-----------------------------------------------------------------------------
typedef struct
{
   volatile uint32_t REVID;         // 0x0000
   volatile uint32_t RSVD0[56];     // 0x0004
   volatile uint32_t RSTYPE;        // 0x00E4
   volatile uint32_t RSVD1[6];      // 0x00E8
   volatile uint32_t PLLCTL;        // 0x0100
   volatile uint32_t OCSEL;         // 0x0104
   volatile uint32_t RSVD2[2];      // 0x0108
   volatile uint32_t PLLM;          // 0x0110
   volatile uint32_t PREDIV;        // 0x0114
   volatile uint32_t PLLDIV1;       // 0x0118
   volatile uint32_t PLLDIV2;       // 0x011C
   volatile uint32_t PLLDIV3;       // 0x0120
   volatile uint32_t OSCDIV;        // 0x0124
   volatile uint32_t POSTDIV;       // 0x0128
   volatile uint32_t RSVD3[3];      // 0x012C
   volatile uint32_t PLLCMD;        // 0x0138
   volatile uint32_t PLLSTAT;       // 0x013C
   volatile uint32_t ALNCTL;        // 0x0140
   volatile uint32_t DCHANGE;       // 0x0144
   volatile uint32_t CKEN;          // 0x0148
   volatile uint32_t CKSTAT;        // 0x014C
   volatile uint32_t SYSTAT;        // 0x0150
   volatile uint32_t RSVD4[3];      // 0x0154
   volatile uint32_t PLLDIV4;       // 0x0160
   volatile uint32_t PLLDIV5;       // 0x0164
   volatile uint32_t PLLDIV6;       // 0x0168
   volatile uint32_t PLLDIV7;       // 0x016C
} pll_regs_t;

#define PLL0         ((pll_regs_t*)PLL0_REG_BASE)
#define PLL1         ((pll_regs_t*)PLL1_REG_BASE)

// bitmask defines for PLLCTL.
#define EXTCLKSRC       (0x00000200)   // bit 9
#define CLKMODE         (0x00000100)   // bit 8
#define PLLENSRC        (0x00000020)   // bit 5
#define PLLDIS          (0x00000010)   // bit 4
#define PLLRST          (0x00000008)   // bit 3
#define PLLPWRDN        (0x00000002)   // bit 1
#define PLLEN           (0x00000001)   // bit 0
#define CLKMODE_SHIFT   (8)

// bitmask defines for PLLCMD and PLLSTAT.
#define GOSET        (0x00000001)
#define GOSTAT       (0x00000001)

// defines for divisors.
#define DIV_ENABLE         (0x00008000)
#define MULTIPLIER         (24)
#define POSTDIV_RATIO      (1)
#define PLLDIV2_RATIO      (2)
#define PLLDIV3_RATIO      (12)
#define PLLDIV4_RATIO      (4)
#define PLLDIV5_RATIO      (5)
#define PLLDIV6_RATIO      (1)
#define PLLDIV7_RATIO      (6)
#define PLL_LOCK_CYCLES          (2400)
#define PLL_STABILIZATION_TIME   (2000)
#define PLL_RESET_TIME_CNT       (200)

// system clock frequency defines.
#define AUXCLOCK_HZ        (24)
#define SYSCLOCK1_HZ       (300000000)
#define SYSCLOCK2_HZ       (SYSCLOCK1_HZ / PLLDIV2_RATIO)
#define SYSCLOCK3_HZ       (SYSCLOCK1_HZ / PLLDIV3_RATIO)
#define SYSCLOCK4_HZ       (SYSCLOCK1_HZ / PLLDIV4_RATIO)
#define SYSCLOCK5_HZ       (SYSCLOCK1_HZ / PLLDIV5_RATIO)
#define SYSCLOCK6_HZ       (SYSCLOCK1_HZ / PLLDIV6_RATIO)
#define SYSCLOCK7_HZ       (SYSCLOCK1_HZ / PLLDIV7_RATIO)

// bitmask defines for OCSEL (OBSCLK pin).
#define OBSCLK_CLKIN       (0x14)
#define OBSCLK_SYSCLK1     (0x17)
#define OBSCLK_SYSCLK2     (0x18)
#define OBSCLK_SYSCLK3     (0x19)
#define OBSCLK_SYSCLK4     (0x1A)
#define OBSCLK_SYSCLK5     (0x1B)
#define OBSCLK_SYSCLK6     (0x1C)
#define OBSCLK_SYSCLK7     (0x1D)

#endif

//-----------------------------------------------------------------------------
// \file    evmc6748_psc.h
// \brief   C6748 power and sleep config registers & bit definitions.
//
//-----------------------------------------------------------------------------

#ifndef EVMC6748_PSC_H
#define EVMC6748_PSC_H

//-----------------------------------------------------------------------------
// Register Structure & Defines
//-----------------------------------------------------------------------------
typedef struct
{
  volatile uint32_t REVID;          // 0x0000
  volatile uint32_t RSVD0[5];       // 0x0004
  volatile uint32_t INTEVAL;        // 0x0018
  volatile uint32_t RSVD1[9];       // 0x001C
  volatile uint32_t MERRPR0;        // 0x0040
  volatile uint32_t RSVD2[3];       // 0x0044
  volatile uint32_t MERRCR0;        // 0x0050
  volatile uint32_t RSVD3[3];       // 0x0054
  volatile uint32_t PERRPR;         // 0x0060
  volatile uint32_t RSVD4;          // 0x0064
  volatile uint32_t PERRCR;         // 0x0068
  volatile uint32_t RSVD5[45];      // 0x006C
  volatile uint32_t PTCMD;          // 0x0120
  volatile uint32_t RSVD6;          // 0x0124
  volatile uint32_t PTSTAT;         // 0x0128
  volatile uint32_t RSVD7[53];      // 0x012C
  volatile uint32_t PDSTAT0;        // 0x0200
  volatile uint32_t PDSTAT1;        // 0x0204
  volatile uint32_t RSVD8[62];      // 0x0208
  volatile uint32_t PDCTL0;         // 0x0300
  volatile uint32_t PDCTL1;         // 0x0304
  volatile uint32_t RSVD9[62];      // 0x0308
  volatile uint32_t PDCFG0;         // 0x0400
  volatile uint32_t PDCFG1;         // 0x0404
  volatile uint32_t RSVD10[254];    // 0x0408
  volatile uint32_t MDSTAT[32];     // 0x0800
  volatile uint32_t RSVD11[96];     // 0x0880
  volatile uint32_t MDCTL[32];      // 0x0A00
} psc_regs_t;

// define the power and sleep config modules.
#define PSC0            ((psc_regs_t *)PSC0_REG_BASE)
#define PSC1            ((psc_regs_t *)PSC1_REG_BASE)

// domain defines.
#define DOMAIN0         (0x00000001)
#define DOMAIN1         (0x00000002)

// defines for ptcmd.
#define GO_0            (0x00000001)
#define GO_1            (0x00000002)

// defines for ptstat.
#define GOSTAT_0        (0x00000001)
#define GOSTAT_1        (0x00000002)

// psc0 lpsc defines.
#define LPSC_TPCC          (0)
#define LPSC_TPTC0         (1)
#define LPSC_TPTC1         (2)
#define LPSC_EMIFA         (3)
#define LPSC_SPI0          (4)
#define LPSC_MMCSD0        (5)
#define LPSC_AINTC         (6)
#define LPSC_ARM_RAMROM    (7)
// 8 not used.
#define LPSC_UART0         (9)
#define LPSC_SCR0          (10)
#define LPSC_SCR1          (11)
#define LPSC_SCR2          (12)
#define LPSC_DMAX          (13)
#define LPSC_ARM           (14)
#define LPSC_DSP           (15)

// psc1 lpsc defines.
// 0 not used.
#define LPSC_USB0                (1)
#define LPSC_USB1                (2)
#define LPSC_GPIO                (3)
#define LPSC_HPI                 (4)
#define LPSC_EMAC                (5)
#define LPSC_EMIF3A              (6)
#define LPSC_MCASP0              (7)
#define LPSC_SATA                (8)
#define LPSC_VPIF                (9)
#define LPSC_SPI1                (10)
#define LPSC_I2C1                (11)
#define LPSC_UART1               (12)
#define LPSC_UART2               (13)
#define LPSC_MCBSP0              (14)
#define LPSC_MCBSP1              (15)
#define LPSC_LCDC                (16)
#define LPSC_PWM                 (17)
#define LPSC_MMCSD1              (18)
#define LPSC_RPI                 (19)
#define LPSC_ECAP                (20)
#define LPSC_TPTC2               (21)
// 22-23 not used.
#define LPSC_SCR8                (24)
#define LPSC_SCR7                (25)
#define LPSC_SCR12               (26)
// 27-30 not used.
#define LPSC_SHRAM               (31)

// psc module status register defines.
#define MASK_STATE               (0x0000003F)

// psc module control register defines.
#define FORCE                    (0x80000000)   // bit 31.
#define EMUIHBIE                 (0x00000400)   // bit 10.
#define EMURSTIE                 (0x00000200)   // bit 9.
#define LRST                     (0x00000100)   // bit 8.
#define NEXT                     (0x00000007)   // bits 0-2.

// psc module next states.
#define PSC_ENABLE               (0x00000003)
#define PSC_DISABLE              (0x00000002)
#define PSC_SYNCRESET            (0x00000001)
#define PSC_SWRSTDISABLE         (0x00000000)

#endif

//-----------------------------------------------------------------------------
// \file    evmc6748_ddr.h
// \brief   C6748 ddr registers & bit definitions.
//
//-----------------------------------------------------------------------------

#ifndef EVMC6748_DDR_H
#define EVMC6748_DDR_H

//-----------------------------------------------------------------------------
// Register Structure & Defines
//-----------------------------------------------------------------------------
typedef struct
{
  volatile uint32_t REVID;          // 0x0000
  volatile uint32_t SDRSTAT;        // 0x0004
  volatile uint32_t SDCR;           // 0x0008
  volatile uint32_t SDRCR;          // 0x000C
  volatile uint32_t SDTIMR1;        // 0x0010
  volatile uint32_t SDTIMR2;        // 0x0014
  volatile uint32_t RSVD1;          // 0x0018
  volatile uint32_t SDCR2;          // 0x001C
  volatile uint32_t PBBPR;          // 0x0020
  volatile uint32_t RSVD2;          // 0x0024
  volatile uint32_t VBUSMCFG1;      // 0x0028
  volatile uint32_t VBUSMCFG2;      // 0x002C
  volatile uint32_t RSVD3[36];      // 0x0030
  volatile uint32_t IRR;            // 0x00C0
  volatile uint32_t IMR;            // 0x00C4
  volatile uint32_t IMSR;           // 0x00C8
  volatile uint32_t IMCR;           // 0x00CC
  volatile uint32_t RSVD4[4];       // 0x00D0
  volatile uint32_t DDRPHYREV;      // 0x00E0
  volatile uint32_t DDRPHYCTL1;     // 0x00E4
  volatile uint32_t DDRPHYCTL2;     // 0x00E8
} ddr_regs_t;

#define DDR          ((ddr_regs_t*)DDR_REG_BASE)

#define VTPIO_CTL    (*(uint32_t *)(0x1E2C0000))

#endif

//-----------------------------------------------------------------------------
// \file    evmc6748_sysconfig.h
// \brief   C6748 system config registers & bit definitions.
//
//-----------------------------------------------------------------------------

#ifndef EVMC6748_SYSCONFIG_H
#define EVMC6748_SYSCONFIG_H

//-----------------------------------------------------------------------------
// Register Structure & Defines
//-----------------------------------------------------------------------------
typedef struct
{
   volatile uint32_t REVID;            // 0x0000
   volatile uint32_t RSVD0;            // 0x0004
   volatile uint32_t DIEIDR[4];        // 0x0008
   volatile uint32_t RSVD1[2];         // 0x0018
   volatile uint32_t BOOTCFG;          // 0x0020
   volatile uint32_t RSVD2[5];         // 0x0024
   volatile uint32_t KICKR[2];         // 0x0038
   volatile uint32_t HOST0CFG;         // 0x0040
   volatile uint32_t HOST1CFG;         // 0x0044
   volatile uint32_t RSVD3[38];        // 0x0048
   volatile uint32_t IRAWSTAT;         // 0x00E0
   volatile uint32_t IENSTAT;          // 0x00E4
   volatile uint32_t IENSET;           // 0x00E8
   volatile uint32_t IENCLR;           // 0x00EC
   volatile uint32_t EOI;              // 0x00F0
   volatile uint32_t FLTADDRR;         // 0x00F4
   volatile uint32_t FLTSTAT;          // 0x00F8
   volatile uint32_t RSVD4[5];         // 0x00FC
   volatile uint32_t MSTPRI[3];        // 0x0110
   volatile uint32_t RSVD5;            // 0x011C
   volatile uint32_t PINMUX[20];       // 0x0120
   volatile uint32_t SUSPSRC;          // 0x0170
   volatile uint32_t CHIPSIG;          // 0x0174
   volatile uint32_t CHIPSIG_CLR;      // 0x0178
   volatile uint32_t CFGCHIP[5];       // 0x017C
} sysconfig_regs_t;

// define the one and only system config module.
#define SYSCONFIG          ((sysconfig_regs_t *)SYSCONFIG_REG_BASE)

// unlock/lock kick registers defines.
#define KICK0R_UNLOCK      (0x83E70B13)
#define KICK1R_UNLOCK      (0x95A4F1E0)
#define KICK0R_LOCK        (0x00000000)
#define KICK1R_LOCK        (0x00000000)

// bitmask defines for cfgchip[0].
#define PLL0_MASTER_LOCK   (0x00000010)   // bit 4

// bitmask defines for cfgchip[3].
#define CLK2XSRC           (0x00000080)   // bit 7
#define PLL1_MASTER_LOCK   (0x00000020)   // bit 5
#define DIV4P5ENA          (0x00000004)   // bit 2
#define EMA_CLKSRC         (0x00000002)   // bit 1

// defines for hostcfg.
#define BOOTRDY            (0x00000001)   // bit 0

// defines for pinmux0.
#define MASK_EMB_WE        (0xF0000000) // bits 31-28
#define MASK_EMB_RAS       (0x0F000000) // bits 27-24

#endif

//-----------------------------------------------------------------------------
// \file    evmc6748_timer.h
// \brief   C6748 timer registers, bit definitions, and
//          function prototypes.
//
//-----------------------------------------------------------------------------

#ifndef EVMC6748_TIMER_H
#define EVMC6748_TIMER_H

//-----------------------------------------------------------------------------
// Register Structure & Defines
//-----------------------------------------------------------------------------
typedef struct
{
    volatile uint32_t REV;             // 0x0000
    volatile uint32_t EMUMGT;          // 0x0004
    volatile uint32_t GPINT_GPEN;      // 0x0008
    volatile uint32_t GPDATA_GPDIR;    // 0x000C
    volatile uint32_t TIM12;           // 0x0010
    volatile uint32_t TIM34;           // 0x0014
    volatile uint32_t PRD12;           // 0x0018
    volatile uint32_t PRD34;           // 0x001C
    volatile uint32_t TCR;             // 0x0020
    volatile uint32_t TGCR;            // 0x0024
    volatile uint32_t WDTCR;           // 0x0028
    volatile uint32_t RSVD0[2];        // 0x002C
    volatile uint32_t REL12;           // 0x0034
    volatile uint32_t REL34;           // 0x0038
    volatile uint32_t CAP12;           // 0x003C
    volatile uint32_t CAP34;           // 0x0040
    volatile uint32_t INTCTLSTAT;      // 0x0044
    volatile uint32_t RSVD1[6];        // 0x0048
    volatile uint32_t CMP0;            // 0x0060
    volatile uint32_t CMP1;            // 0x0064
    volatile uint32_t CMP2;            // 0x0068
    volatile uint32_t CMP3;            // 0x006C
    volatile uint32_t CMP4;            // 0x0070
    volatile uint32_t CMP5;            // 0x0074
    volatile uint32_t CMP6;            // 0x0078
    volatile uint32_t CMP7;            // 0x007C
} timer_regs_t;

// define all the available timer peripherals for the processor.
#define TMR0            ((timer_regs_t *)TIMER0_REG_BASE)
#define TMR1            ((timer_regs_t *)TIMER1_REG_BASE)

// bitmask defines for GPINT_GPEN.
#define GPENO34         (0x02000000)   // bit 25
#define GPENI34         (0x01000000)   // bit 24
#define GPENO12         (0x00020000)   // bit 17
#define GPENI12         (0x00010000)   // bit 16
#define GPINT34INVO     (0x00002000)   // bit 13
#define GPINT34INVI     (0x00001000)   // bit 12
#define GPINT34ENO      (0x00000200)   // bit 9
#define GPINT34ENI      (0x00000100)   // bit 8
#define GPINT12INVO     (0x00000020)   // bit 5
#define GPINT12INVI     (0x00000010)   // bit 4
#define GPINT12ENO      (0x00000002)   // bit 1
#define GPINT12ENI      (0x00000001)   // bit 0

// bitmask defines for GPDATA_GPDIR.
#define GPDIRO34        (0x02000000)   // bit 25
#define GPDIRI34        (0x01000000)   // bit 24
#define GPDIRO12        (0x00020000)   // bit 17
#define GPDIRI12        (0x00010000)   // bit 16
#define GPDATAO34       (0x00000200)   // bit 9
#define GPDATAI34       (0x00000100)   // bit 8
#define GPDATAO12       (0x00000002)   // bit 1
#define GPDATAI12       (0x00000001)   // bit 0

// bitmask defines for TCR.
#define ENAMODE34                (0x00C00000)   // bit 22,23
#define ENAMODE34_ONETIME        (0x00400000)   // bit 22
#define ENAMODE34_CONT           (0x00800000)   // bit 23
#define ENAMODE34_CONT_RELOAD    (0x00C00000)   // bit 22,23
#define ENAMODE12                (0x000000C0)   // bit 6,7
#define ENAMODE12_ONETIME        (0x00000040)   // bit 6
#define ENAMODE12_CONT           (0x00000080)   // bit 7
#define ENAMODE12_CONT_RELOAD    (0x000000C0)   // bit 6,7

// bitmask defines for TGCR.
#define PRESCALER(n)             ((n) << 8)
#define PLUSEN                   (0x00000010)   // bit 4
#define TIMMODE_64BIT            (0x0000000C)   // bit 2,3
#define TIMMODE_32BIT_UNCHAINED  (0x00000004)   // bit 2
#define TIMMODE_64BIT_WDOG       (0x00000008)   // bit 3
#define TIMMODE_32BIT_CHAINED    (0x0000000C)   // bit 2,3
#define TIM34RS                  (0x00000002)   // bit 1
#define TIM12RS                  (0x00000001)   // bit 0

// bitmask defines for INTCTLSTAT.
#define PRDINTSTAT34             (0x00020000)   // bit 17
#define PRDINTEN34               (0x00010000)   // bit 16
#define PRDINTSTAT12             (0x00000002)   // bit 1
#define PRDINTEN12               (0x00000001)   // bit 0

#define DELAY_10TH_SEC     (100000)   // in us
#define DELAY_QUARTER_SEC  (250000)    // in us
#define DELAY_HALF_SEC     (500000)    // in us
#define DELAY_1_SEC        (1000000)   // in us

//-----------------------------------------------------------------------------
// Public Function Prototypes
//-----------------------------------------------------------------------------
uint32_t USTIMER_init(void);
void USTIMER_delay(uint32_t in_delay);
void USTIMER_reset(void);
uint32_t USTIMER_get(void);
void USTIMER_set(uint32_t in_time);

#endif


//-----------------------------------------------------------------------------
// \file    evmc6748.h
// \brief   C6748 internal registers & bit definitions.
//
//-----------------------------------------------------------------------------

#ifndef EVMC6748_H
#define EVMC6748_H

//-----------------------------------------------------------------------------
// bit manipulation macros and bitval lookup table declarations.
//-----------------------------------------------------------------------------
#define SETBIT(dest,mask)     (dest |= mask)
#define CLRBIT(dest,mask)     (dest &= ~mask)
#define TGLBIT(dest,mask)     (dest ^= mask)
#define CHKBIT(dest,mask)     (dest & mask)
extern const uint32_t bitval_u32[32];

//-----------------------------------------------------------------------------
// bitmask defines for EMUMGT.
//-----------------------------------------------------------------------------
#define SOFT                  (0x00000002)   // bit 1
#define FREE                  (0x00000001)   // bit 0

//-----------------------------------------------------------------------------
// Return Error Defines
//-----------------------------------------------------------------------------
#define ERR_NO_ERROR             (0)
#define ERR_FAIL                 (1)
#define ERR_INIT_FAIL            (2)
#define ERR_NO_UI_BOARD          (3)
#define ERR_INVALID_PARAMETER    (4)
#define ERR_TIMEOUT              (5)
#define ERR_UART_RX_FIFO_EMPTY   (6)
#define ERR_SPI_BUS_ERROR        (7)
#define ERR_ENET_LINK_INACTIVE   (8)
#define ERR_ENET_PKT_TOO_LARGE   (9)
#define ERR_ENET_Q_EMPTY         (10)
#define ERR_ENET_RX_ERROR        (11)
#define ERR_FLASH_VERIFY         (12)
#define ERR_MMCSD_TIMEOUT        (13)
#define ERR_NAND_BAD_BLOCK       (14)
#define ERR_UIDB_NUM_DEVICES     (15)    //for UI databus muxing control 
#define ERR_UIDB_INCOMPAT_DEV    (16)
#define ERR_UIDB_INVALID_DEVICE  (17)
//-----------------------------------------------------------------------------
// Utility Print Defines
//-----------------------------------------------------------------------------
#define PRINT_NONE                  (0)
#define PRINT_DURING_VERIFY         (1)
#define PRINT_HEADER                (0)
#define PRINT_CONTINUE              (1)

//-----------------------------------------------------------------------------
// power and sleep config registers
//-----------------------------------------------------------------------------
#define PSC0_REG_BASE         (0x01C10000)
#define PSC1_REG_BASE         (0x01E27000)

//-----------------------------------------------------------------------------
// system config registers
//-----------------------------------------------------------------------------
#define SYSCONFIG_REG_BASE    (0x01C14000)

//-----------------------------------------------------------------------------
// PLL registers
//-----------------------------------------------------------------------------
#define PLL0_REG_BASE         (0x01C11000)
#define PLL1_REG_BASE         (0x01E1A000)

//-----------------------------------------------------------------------------
// DDR registers
//-----------------------------------------------------------------------------
#define DDR_REG_BASE          (0xB0000000)
#define DDR_MEM_BASE          (0xC0000000)
#define DDR_MEM_SIZE          (0x08000000)	// 128MB

//-----------------------------------------------------------------------------
// EMIFA registers
//-----------------------------------------------------------------------------
#define EMIFA_REG_BASE        (0x68000000)

//-----------------------------------------------------------------------------
// GPIO registers
//-----------------------------------------------------------------------------
#define GPIO_REG_BASE         (0x01E26000)
#define GPIO_BANK_OFFSET      (0x28)

#define GPIO_REV              (GPIO_REG_BASE)
#define GPIO_BINTEN           (GPIO_REG_BASE + 0x08)

#define GPIO_BANK01_BASE      (GPIO_REG_BASE + 0x10)
#define GPIO_BANK23_BASE      (GPIO_BANK01_BASE + GPIO_BANK_OFFSET)
#define GPIO_BANK45_BASE      (GPIO_BANK23_BASE + GPIO_BANK_OFFSET)
#define GPIO_BANK67_BASE      (GPIO_BANK45_BASE + GPIO_BANK_OFFSET)
#define GPIO_BANK8_BASE       (GPIO_BANK67_BASE + GPIO_BANK_OFFSET)

#define GPIO_BUFF_OE_BANK        (2)
#define GPIO_BUFF_OE_PIN         (6)
#define PINMUX_GPIO_BUFF_OE_REG  (6)
#define PINMUX_GPIO_BUFF_OE_MASK (0x000000F0)
#define PINMUX_GPIO_BUFF_OE_VAL  (0x00000080)

//-----------------------------------------------------------------------------
// Timer registers
//-----------------------------------------------------------------------------
#define TIMER0_REG_BASE       (0x01C20000)
#define TIMER1_REG_BASE       (0x01C21000)

//-----------------------------------------------------------------------------
// UART registers
//-----------------------------------------------------------------------------
#define UART0_REG_BASE        (0x01C42000)
#define UART1_REG_BASE        (0x01D0C000)
#define UART2_REG_BASE        (0x01D0D000)

//-----------------------------------------------------------------------------
// SPI registers
//-----------------------------------------------------------------------------
#define SPI0_REG_BASE         (0x01C41000)
#define SPI1_REG_BASE         (0x01F0E000)

//-----------------------------------------------------------------------------
// I2C registers
//-----------------------------------------------------------------------------
#define I2C0_REG_BASE         (0x01C22000)
#define I2C1_REG_BASE         (0x01E28000)

//-----------------------------------------------------------------------------
// EMAC registers
//-----------------------------------------------------------------------------
#define EMAC_RAM_BASE         (0x01E20000)
#define EMAC_CTRL_REG_BASE    (0x01E22000)
#define EMAC_REG_BASE         (0x01E23000)
#define MDIO_REG_BASE         (0x01E24000)

//-----------------------------------------------------------------------------
// MMCSD registers
//-----------------------------------------------------------------------------
#define MMCSD0_REG_BASE       (0x01C40000)
#define MMCSD1_REG_BASE       (0x01E1B000)

//-----------------------------------------------------------------------------
// MCASP registers
//-----------------------------------------------------------------------------
#define MCASP_REG_BASE        (0x01D00000)

//-----------------------------------------------------------------------------
// USB registers
//-----------------------------------------------------------------------------
#define USB_OTG_REG_BASE      (0x01E00000)
#define USB_HOST_REG_BASE     (0x01E25000)

//-----------------------------------------------------------------------------
// VPIF registers
//-----------------------------------------------------------------------------
#define VPIF_REG_BASE            (0x01E17000)
#define VPIF_CAP_CH0_REG_BASE    (0x01E17040)
#define VPIF_CAP_CH1_REG_BASE    (0x01E17080)
#define VPIF_DISP_CH2_REG_BASE   (0x01E170C0)
#define VPIF_DISP_CH3_REG_BASE   (0x01E17140)

//-----------------------------------------------------------------------------
// LCDC registers
//-----------------------------------------------------------------------------
#define LCDC_REG_BASE         (0x01E13000)

//-----------------------------------------------------------------------------
// uPP registers
//-----------------------------------------------------------------------------
#define UPP_REG_BASE          (0x01E16000)

//-----------------------------------------------------------------------------
// RTC registers
//-----------------------------------------------------------------------------
#define RTC_REG_BASE          (0x01C23000)

//-----------------------------------------------------------------------------
// Public Function Prototypes
//-----------------------------------------------------------------------------
uint32_t EVMC6748_init(void);
uint32_t EVMC6748_initRAM(void);
void EVMC6748_enableDsp(void);
void EVMC6748_pinmuxConfig(uint32_t in_reg, uint32_t in_mask, uint32_t in_val);
void EVMC6748_lpscTransition(psc_regs_t *psc_reg, uint32_t in_domain, uint8_t in_module, uint8_t in_next_state);
void UTIL_printMem(uint32_t begin_addr, uint8_t *buffer, uint32_t length, uint8_t continuation);
uint8_t UTIL_isUIBoardAttached(void);
uint32_t config_pll0(uint32_t clkmode, uint32_t pllm, uint32_t postdiv, uint32_t plldiv1, uint32_t plldiv2, uint32_t plldiv3, uint32_t plldiv7);
uint32_t config_pll1(uint32_t pllm, uint32_t postdiv, uint32_t plldiv1, uint32_t plldiv2, uint32_t plldiv3);

#endif


//-----------------------------------------------------------------------------
// \file    evmc6748_spi.h
// \brief   C6748 spi registers, bit definitions, and
//          function prototypes.
//
//-----------------------------------------------------------------------------

#ifndef EVMC6748_SPI_H
#define EVMC6748_SPI_H

//-----------------------------------------------------------------------------
// Register Structure & Defines
//-----------------------------------------------------------------------------
typedef struct
{
  volatile uint32_t SPIGCR0;        // 0x0000
  volatile uint32_t SPIGCR1;        // 0x0004
  volatile uint32_t SPIINT;         // 0x0008
  volatile uint32_t SPILVL;         // 0x000C
  volatile uint32_t SPIFLG;         // 0x0010
  volatile uint32_t SPIPC0;         // 0x0014
  volatile uint32_t SPIPC1;         // 0x0018
  volatile uint32_t SPIPC2;         // 0x001C
  volatile uint32_t SPIPC3;         // 0x0020
  volatile uint32_t SPIPC4;         // 0x0024
  volatile uint32_t SPIPC5;         // 0x0028
  volatile uint32_t RSVD0[3];       // 0x002C
  volatile uint32_t SPIDAT0;        // 0x0038
  volatile uint32_t SPIDAT1;        // 0x003C
  volatile uint32_t SPIBUF;         // 0x0040
  volatile uint32_t SPIEMU;         // 0x0044
  volatile uint32_t SPIDELAY;       // 0x0048
  volatile uint32_t SPIDEF;         // 0x004C
  volatile uint32_t SPIFMT0;        // 0x0050
  volatile uint32_t SPIFMT1;        // 0x0054
  volatile uint32_t SPIFMT2;        // 0x0058
  volatile uint32_t SPIFMT3;        // 0x005C
  volatile uint32_t INTVEC0;        // 0x0060
  volatile uint32_t INTVEC1;        // 0x0064
} spi_regs_t;

#define SPI0         ((spi_regs_t *)SPI0_REG_BASE)
#define SPI1         ((spi_regs_t *)SPI1_REG_BASE)

// defines for SPIGCR0.
#define RESET        (0x00000001)   // bit 0

// defines for SPIGCR1.
#define ENABLE       (0x01000000)   // bit 24
#define LOOPBACK     (0x00010000)   // bit 16
#define POWERDOWN    (0x00000100)   // bit 8
#define CLKMOD       (0x00000002)   // bit 1
#define MASTER       (0x00000001)   // bit 0

// defines for SPIPC0, 1, 2, 3, 4, 5.
#define SOMI         (0x00000800)   // bit 11
#define SIMO         (0x00000400)   // bit 10
#define CLK          (0x00000200)   // bit 9
#define ENA          (0x00000100)   // bit 8
#define SCS0         (0x00000001)   // bit 0

// defines for SPIDAT1.
#define CSHOLD       (0x10000000)   // bit 28
#define CSNR         (0x00010000)   // bit 16
#define MASK_TXDATA  (0x0000FFFF)

// defines for SPIBUF.
#define RXEMPTY      (0x80000000)   // bit 31
#define TXFULL       (0x20000000)   // bit 29

// defines for SPIDEF.
#define CSDEF0       (0x00000001)   // bit 0

// defines for SPIFMT0.
#define SHIFTDIR           (0x00100000)   // bit 20
#define POLARITY           (0x00020000)   // bit 17
#define PHASE              (0x00010000)   // bit 16
#define SHIFT_PRESCALE     (8)   // bit 8
#define SHIFT_CHARLEN      (0)   // bit 0

//-----------------------------------------------------------------------------
// Public Typedefs
//-----------------------------------------------------------------------------

typedef enum
{
   SPI_MODE_SLAVE = 0,
   SPI_MODE_MASTER
} spi_mode_e;

typedef enum
{
   SPI_3PIN = 0,
   SPI_4PIN_CS,
   SPI_4PIN_EN,
   SPI_5PIN
} spi_pin_option_e;

typedef enum
{
   SPI_CS_ACTIVE_LOW = 0,
   SPI_CS_ACTIVE_HIGH
} spi_cs_active_e;

typedef enum
{
   SPI_SHIFT_MSB = 0,
   SPI_SHIFT_LSB
} spi_shift_dir_e;

typedef enum
{
   SPI_HOLD_NONE = 0,
   SPI_HOLD_ACTIVE,
   SPI_HOLD_CLR
} spi_cs_hold_e;

typedef struct
{
   spi_mode_e mode;
   spi_pin_option_e pin_option;
   spi_cs_active_e cs_active;
   spi_shift_dir_e shift_dir;
   uint8_t polarity;
   uint8_t phase;
   uint32_t freq;
} spi_config_t;

//-----------------------------------------------------------------------------
// Public Function Prototypes
//-----------------------------------------------------------------------------

uint32_t SPI_init(spi_regs_t *spi, spi_config_t *in_config);
uint32_t SPI_xfer(spi_regs_t *spi, uint8_t *src_buffer, uint8_t *dest_buffer, uint32_t in_length, spi_cs_hold_e in_cs_hold);

#endif

