/**  ====================== CC2640.c =================================
 *  This board file is made for the 4x4 mm QFN package, to convert this board
 *  file to use for other smaller or larger device packages please refer to the table
 *  below which lists the max IOID values supported by each package. All other
 *  unused pins should be set to IOID_UNUSED.
 *
 *  Furthermore the board file is also used
 *  to define a symbol that configures the RF front end and bias.
 *  See the comments below for more information.
 *  For an in depth tutorial on how to create a custom board file, please refer
 *  to the section "Running the SDK on Custom Boards" with in the Software
 *  Developer's Guide.
 *
 *  Refer to the datasheet for all the package options and IO descriptions:
 *  http://www.ti.com/lit/ds/symlink/cc2640r2f.pdf
 *
 *  +-----------------------+------------------+-----------------------+
 *  |     Package Option    |  Total GPIO Pins |   MAX IOID            |
 *  +=======================+==================+=======================+
 *  |     7x7 mm QFN        |     31           |   IOID_30             |
 *  +-----------------------+------------------+-----------------------+
 *  |     5x5 mm QFN        |     15           |   IOID_14             |
 *  +-----------------------+------------------+-----------------------+
 *  |     4x4 mm QFN        |     10           |   IOID_9              |
 *  +-----------------------+------------------+-----------------------+
 *  |     2.7 x 2.7 mm WCSP |     14           |   IOID_13             |
 *  +-----------------------+------------------+-----------------------+
 *  ============================================================================
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <ti/devices/cc26x0r2/driverlib/ioc.h>
#include <ti/devices/cc26x0r2/driverlib/udma.h>
#include <ti/devices/cc26x0r2/inc/hw_ints.h>
#include <ti/devices/cc26x0r2/inc/hw_memmap.h>

#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

#include "CC2640.h"

/*
 *  =============================== Crypto ===============================
 */
#include <ti/drivers/crypto/CryptoCC26XX.h>

CryptoCC26XX_Object cryptoCC26XXObjects[CC2640_CRYPTOCOUNT];

const CryptoCC26XX_HWAttrs cryptoCC26XXHWAttrs[CC2640_CRYPTOCOUNT] = {
    {
        .baseAddr       = CRYPTO_BASE,
        .powerMngrId    = PowerCC26XX_PERIPH_CRYPTO,
        .intNum         = INT_CRYPTO_RESULT_AVAIL_IRQ,
        .intPriority    = ~0,
    }
};

const CryptoCC26XX_Config CryptoCC26XX_config[CC2640_CRYPTOCOUNT] = {
    {
         .object  = &cryptoCC26XXObjects[CC2640_CRYPTO0],
         .hwAttrs = &cryptoCC26XXHWAttrs[CC2640_CRYPTO0]
    },
};

/*
 *  =============================== AESCCM ===============================
 */
#include <ti/drivers/AESCCM.h>
#include <ti/drivers/aesccm/AESCCMCC26XX.h>

AESCCMCC26XX_Object aesccmCC26XXObjects[CC2640_AESCCMCOUNT];

const AESCCMCC26XX_HWAttrs aesccmCC26XXHWAttrs[CC2640_AESCCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCCM_Config AESCCM_config[CC2640_AESCCMCOUNT] = {
    {
         .object  = &aesccmCC26XXObjects[CC2640_AESCCM0],
         .hwAttrs = &aesccmCC26XXHWAttrs[CC2640_AESCCM0]
    },
};

const uint_least8_t AESCCM_count = CC2640_AESCCMCOUNT;


/*
 *  =============================== AESGCM ===============================
 */
#include <ti/drivers/AESGCM.h>
#include <ti/drivers/aesgcm/AESGCMCC26XX.h>

AESGCMCC26XX_Object aesgcmCC26XXObjects[CC2640_AESGCMCOUNT];

const AESGCMCC26XX_HWAttrs aesgcmCC26XXHWAttrs[CC2640_AESGCMCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESGCM_Config AESGCM_config[CC2640_AESGCMCOUNT] = {
    {
         .object  = &aesgcmCC26XXObjects[CC2640_AESGCM0],
         .hwAttrs = &aesgcmCC26XXHWAttrs[CC2640_AESGCM0]
    },
};

const uint_least8_t AESGCM_count = CC2640_AESGCMCOUNT;

/*
 *  =============================== AESCBC ===============================
 */
#include <ti/drivers/AESCBC.h>
#include <ti/drivers/aescbc/AESCBCCC26XX.h>

AESCBCCC26XX_Object aescbcCC26XXObjects[CC2640_AESCBCCOUNT];

const AESCBCCC26XX_HWAttrs aescbcCC26XXHWAttrs[CC2640_AESCBCCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCBC_Config AESCBC_config[CC2640_AESCBCCOUNT] = {
    {
         .object  = &aescbcCC26XXObjects[CC2640_AESCBC0],
         .hwAttrs = &aescbcCC26XXHWAttrs[CC2640_AESCBC0]
    },
};

const uint_least8_t AESCBC_count = CC2640_AESCBCCOUNT;

/*
 *  =============================== AESCTR ===============================
 */
#include <ti/drivers/AESCTR.h>
#include <ti/drivers/aesctr/AESCTRCC26XX.h>

AESCTRCC26XX_Object aesctrCC26XXObjects[CC2640_AESCTRCOUNT];

const AESCTRCC26XX_HWAttrs aesctrCC26XXHWAttrs[CC2640_AESCTRCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESCTR_Config AESCTR_config[CC2640_AESCTRCOUNT] = {
    {
         .object  = &aesctrCC26XXObjects[CC2640_AESCTR0],
         .hwAttrs = &aesctrCC26XXHWAttrs[CC2640_AESCTR0]
    },
};

const uint_least8_t AESCTR_count = CC2640_AESCTRCOUNT;

/*
 *  =============================== AESECB ===============================
 */
#include <ti/drivers/AESECB.h>
#include <ti/drivers/aesecb/AESECBCC26XX.h>

AESECBCC26XX_Object aesecbCC26XXObjects[CC2640_AESECBCOUNT];

const AESECBCC26XX_HWAttrs aesecbCC26XXHWAttrs[CC2640_AESECBCOUNT] = {
    {
        .intPriority       = ~0,
    }
};

const AESECB_Config AESECB_config[CC2640_AESECBCOUNT] = {
    {
         .object  = &aesecbCC26XXObjects[CC2640_AESECB0],
         .hwAttrs = &aesecbCC26XXHWAttrs[CC2640_AESECB0]
    },
};

const uint_least8_t AESECB_count = CC2640_AESECBCOUNT;

/*
 *  =============================== AESCTRDRBG ===============================
 */
#include <ti/drivers/AESCTRDRBG.h>
#include <ti/drivers/aesctrdrbg/AESCTRDRBGXX.h>

AESCTRDRBGXX_Object aesctrdrbgXXObjects[CC2640_AESCTRDRBGCOUNT];

const AESCTRDRBGXX_HWAttrs aesctrdrbgXXHWAttrs[CC2640_AESCTRDRBGCOUNT] = {
    {
        .aesctrIndex       = CC2640_AESCTR0,
    }
};

const AESCTRDRBG_Config AESCTRDRBG_config[CC2640_AESCTRDRBGCOUNT] = {
    {
         .object  = &aesctrdrbgXXObjects[CC2640_AESCTRDRBG0],
         .hwAttrs = &aesctrdrbgXXHWAttrs[CC2640_AESCTRDRBG0]
    },
};

const uint_least8_t AESCTRDRBG_count = CC2640_AESCTRDRBGCOUNT;

/*
 *  =============================== GPTimer ===============================
 *  Remove unused entries to reduce flash usage both in Board.c and Board.h
 */
#include <ti/drivers/timer/GPTimerCC26XX.h>

GPTimerCC26XX_Object gptimerCC26XXObjects[CC2640_GPTIMERCOUNT];

const GPTimerCC26XX_HWAttrs gptimerCC26xxHWAttrs[CC2640_GPTIMERPARTSCOUNT] = {
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0A, },
    { .baseAddr = GPT0_BASE, .intNum = INT_GPT0B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT0, .pinMux = GPT_PIN_0B, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1A, },
    { .baseAddr = GPT1_BASE, .intNum = INT_GPT1B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT1, .pinMux = GPT_PIN_1B, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2A, },
    { .baseAddr = GPT2_BASE, .intNum = INT_GPT2B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT2, .pinMux = GPT_PIN_2B, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3A, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3A, },
    { .baseAddr = GPT3_BASE, .intNum = INT_GPT3B, .intPriority = (~0), .powerMngrId = PowerCC26XX_PERIPH_GPT3, .pinMux = GPT_PIN_3B, },
};

const GPTimerCC26XX_Config GPTimerCC26XX_config[CC2640_GPTIMERPARTSCOUNT] = {
    { &gptimerCC26XXObjects[CC2640_GPTIMER0], &gptimerCC26xxHWAttrs[CC2640_GPTIMER0A], GPT_A },
    { &gptimerCC26XXObjects[CC2640_GPTIMER0], &gptimerCC26xxHWAttrs[CC2640_GPTIMER0B], GPT_B },
    { &gptimerCC26XXObjects[CC2640_GPTIMER1], &gptimerCC26xxHWAttrs[CC2640_GPTIMER1A], GPT_A },
    { &gptimerCC26XXObjects[CC2640_GPTIMER1], &gptimerCC26xxHWAttrs[CC2640_GPTIMER1B], GPT_B },
    { &gptimerCC26XXObjects[CC2640_GPTIMER2], &gptimerCC26xxHWAttrs[CC2640_GPTIMER2A], GPT_A },
    { &gptimerCC26XXObjects[CC2640_GPTIMER2], &gptimerCC26xxHWAttrs[CC2640_GPTIMER2B], GPT_B },
    { &gptimerCC26XXObjects[CC2640_GPTIMER3], &gptimerCC26xxHWAttrs[CC2640_GPTIMER3A], GPT_A },
    { &gptimerCC26XXObjects[CC2640_GPTIMER3], &gptimerCC26xxHWAttrs[CC2640_GPTIMER3B], GPT_B },
};

/*
 *  =============================== I2C ===============================
*/
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

I2CCC26XX_Object i2cCC26xxObjects[CC2640_I2CCOUNT];

const I2CCC26XX_HWAttrsV1 i2cCC26xxHWAttrs[CC2640_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_I2C0,
        .intNum      = INT_I2C_IRQ,
        .intPriority = ~0,
        .swiPriority = 0,
        .sdaPin      = CC2640_I2C0_SDA0,
        .sclPin      = CC2640_I2C0_SCL0,
    }
};

const I2C_Config I2C_config[CC2640_I2CCOUNT] = {
    {
        .fxnTablePtr = &I2CCC26XX_fxnTable,
        .object      = &i2cCC26xxObjects[CC2640_I2C0],
        .hwAttrs     = &i2cCC26xxHWAttrs[CC2640_I2C0]
    },
};

const uint_least8_t I2C_count = CC2640_I2CCOUNT;

/*
 *  =============================== PIN ===============================
 */
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>

const PIN_Config BoardGpioInitTable[] = {
    CC2640_PIN_CD | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL,                          /* CD is initially 0 */
    CC2640_PIN_CHARGE | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_DIS,                               /* Charge is just an input */
    CC2640_PIN_ALERT | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_NEGEDGE | PIN_HYSTERESIS,           /* ALERT is active low */
    CC2640_PIN_INTX | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_POSEDGE | PIN_HYSTERESIS,            /* INTX is active high */
    CC2640_PIN_INTY | PIN_INPUT_EN | PIN_NOPULL | PIN_IRQ_POSEDGE | PIN_HYSTERESIS,            /* INTY is active high */

    PIN_TERMINATE
};

const PINCC26XX_HWAttrs PINCC26XX_hwAttrs = {
    .intPriority = ~0,
    .swiPriority = 0
};

/*
 *  =============================== Power ===============================
 */

const PowerCC26XX_Config PowerCC26XX_config = {
    .policyInitFxn      = NULL,
    .policyFxn          = &PowerCC26XX_standbyPolicy,
    .calibrateFxn       = &PowerCC26XX_calibrate,
    .enablePolicy       = true,
    .calibrateRCOSC_LF  = true,
    .calibrateRCOSC_HF  = true,
};

/*
 *  =============================== RF Driver ===============================
 */
#include <ti/drivers/rf/RF.h>

const RFCC26XX_HWAttrsV2 RFCC26XX_hwAttrs = {
    .hwiPriority        = ~0,       /* Lowest HWI priority */
    .swiPriority        = 0,        /* Lowest SWI priority */
    .xoscHfAlwaysNeeded = true,     /* Keep XOSC dependency while in standby */
    .globalCallback     = NULL,     /* No board specific callback */
    .globalEventMask    = 0         /* No events subscribed to */
};

/*
 *  =============================== UDMA ===============================
 */
#include <ti/drivers/dma/UDMACC26XX.h>

UDMACC26XX_Object udmaObjects[CC2640_UDMACOUNT];

const UDMACC26XX_HWAttrs udmaHWAttrs[CC2640_UDMACOUNT] = {
    {
        .baseAddr    = UDMA0_BASE,
        .powerMngrId = PowerCC26XX_PERIPH_UDMA,
        .intNum      = INT_DMA_ERR,
        .intPriority = ~0
    }
};

const UDMACC26XX_Config UDMACC26XX_config[CC2640_UDMACOUNT] = {
    {
         .object  = &udmaObjects[CC2640_UDMA0],
         .hwAttrs = &udmaHWAttrs[CC2640_UDMA0]
    },
};

/*
 *  =============================== Watchdog ===============================
 */
#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogCC26XX.h>

WatchdogCC26XX_Object watchdogCC26XXObjects[CC2640_WATCHDOGCOUNT];

const WatchdogCC26XX_HWAttrs watchdogCC26XXHWAttrs[CC2640_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WDT_BASE,
        .reloadValue = 1000 /* Reload value in milliseconds */
    },
};

const Watchdog_Config Watchdog_config[CC2640_WATCHDOGCOUNT] = {
    {
        .fxnTablePtr = &WatchdogCC26XX_fxnTable,
        .object      = &watchdogCC26XXObjects[CC2640_WATCHDOG0],
        .hwAttrs     = &watchdogCC26XXHWAttrs[CC2640_WATCHDOG0]
    },
};

const uint_least8_t Watchdog_count = CC2640_WATCHDOGCOUNT;

/*
 *  ========================= TRNG begin ====================================
 */
#include <TRNGCC26XX.h>

/* TRNG objects */
TRNGCC26XX_Object trngCC26XXObjects[CC2640_TRNGCOUNT];

/* TRNG configuration structure, describing which pins are to be used */
const TRNGCC26XX_HWAttrs TRNGCC26XXHWAttrs[CC2640_TRNGCOUNT] = {
    {
        .powerMngrId    = PowerCC26XX_PERIPH_TRNG,
    }
};

/* TRNG configuration structure */
const TRNGCC26XX_Config TRNGCC26XX_config[] = {
    {
         .object  = &trngCC26XXObjects[0],
         .hwAttrs = &TRNGCC26XXHWAttrs[0]
    },
    {NULL, NULL}
};

/*
 *  ========================= TRNG end ====================================
 */


/*
 *  ======== CC2640_initGeneral ========
 */
void CC2640_initGeneral(void)
{
    Power_init();

    if (PIN_init(BoardGpioInitTable) != PIN_SUCCESS) {
        /* Error with PIN_init */
        while (1);
    }
}

/*
 *  ======== Board_init ========
 */
void Board_init(void)
{
    CC2640_initGeneral();
}
