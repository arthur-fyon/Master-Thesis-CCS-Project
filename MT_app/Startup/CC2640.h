/** ============================================================================
 *  @file       CC2640.h
 *
 *  @brief      CC2640 Board Specific header file.
 *
 *  This file is responsible for setting up the board specific items for the
 *  CC2640 custom board.
 *
 *  This board file is made for the 7x7 mm QFN package, to convert this board
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
#ifndef __CC2640_BOARD_H__
#define __CC2640_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <ti/drivers/PIN.h>
#include <ti/devices/cc26x0r2/driverlib/ioc.h>

/* Externs */
extern const PIN_Config BoardGpioInitTable[];

/* Defines */
#ifndef CC2640
  #define CC2640
#endif /* CC2640 */

/*
 *  ============================================================================
 *  RF Front End and Bias configuration symbols for TI reference designs and
 *  kits. This symbol sets the RF Front End configuration in ble_user_config.h
 *  and selects the appropriate PA table in ble_user_config.c.
 *  Other configurations can be used by editing these files.
 *
 *  Define only one symbol:
 *  CC2650EM_7ID    - Differential RF and internal biasing
                      (default for CC2640R2 LaunchPad)
 *  CC2650EM_5XD    � Differential RF and external biasing
 *  CC2650EM_4XS    � Single-ended RF on RF-P and external biasing
 *  CC2640R2DK_CXS  - WCSP: Single-ended RF on RF-N and external biasing
 *                    (Note that the WCSP is only tested and characterized for
 *                     single ended configuration, and it has a WCSP-specific
 *                     PA table)
 *
 *  Note: CC2650EM_xxx reference designs apply to all CC26xx devices.
 *  ==========================================================================
 */
#define CC2650EM_7ID

/* Mapping of pins to board signals using general board aliases
 *      <board signal alias>                  <pin mapping>
 */

/* Analog Capable DIOs */
#define CC2640_DIO7_ANALOG           IOID_7
#define CC2640_DIO8_ANALOG           IOID_8
#define CC2640_DIO9_ANALOG           IOID_9
#define CC2640_DIO10_ANALOG          IOID_10
#define CC2640_DIO11_ANALOG          IOID_11
#define CC2640_DIO12_ANALOG          IOID_12
#define CC2640_DIO13_ANALOG          IOID_13
#define CC2640_DIO14_ANALOG          IOID_14

/* IOs */
#define CC2640_DIO0                  IOID_0
#define CC2640_DIO1                  IOID_1
#define CC2640_DIO2                  IOID_2
#define CC2640_DIO3                  IOID_3
#define CC2640_DIO4                  IOID_4
#define CC2640_DIO5_TDO              IOID_5
#define CC2640_DIO6_TDI              IOID_6
#define CC2640_DIO7                  IOID_7
#define CC2640_DIO8                  IOID_8
#define CC2640_DIO9                  IOID_9
#define CC2640_DIO10                 IOID_10
#define CC2640_DIO11                 IOID_11
#define CC2640_DIO12                 IOID_12
#define CC2640_DIO13                 IOID_13
#define CC2640_DIO14                 IOID_14

/* CD of the BQ25125 */
#define CC2640_PIN_CD                CC2640_DIO2
#define CC2640_CD_I2C_ENABLED        1
#define CC2640_CD_I2C_DISABLED       0

/* Charge of the battery */
#define CC2640_PIN_CHARGE            CC2640_DIO3

/* Interrupt pins */
#define CC2640_PIN_ALERT             CC2640_DIO7
#define CC2640_PIN_INTX              CC2640_DIO8
#define CC2640_PIN_INTY              CC2640_DIO9

/* I2C */
#define CC2640_I2C0_SCL0             CC2640_DIO1
#define CC2640_I2C0_SDA0             CC2640_DIO0

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings.
 */
void CC2640_initGeneral(void);


/*!
 *  @def    CC2640_CryptoName
 *  @brief  Enum of Crypto names
 */
typedef enum CC2640_CryptoName {
    CC2640_CRYPTO0 = 0,

    CC2640_CRYPTOCOUNT
} CC2640_CryptoName;

/*!
 *  @def    CC2640_AESCCMName
 *  @brief  Enum of AESCCM names
 */
typedef enum CC2640_AESCCMName {
    CC2640_AESCCM0 = 0,

    CC2640_AESCCMCOUNT
} CC2640_AESCCMName;

/*!
 *  @def    CC2640_AESGCMName
 *  @brief  Enum of AESGCM names
 */
typedef enum CC2640_AESGCMName {
    CC2640_AESGCM0 = 0,

    CC2640_AESGCMCOUNT
} CC2640_AESGCMName;

/*!
 *  @def    CC2640_AESCBCName
 *  @brief  Enum of AESCBC names
 */
typedef enum CC2640_AESCBCName {
    CC2640_AESCBC0 = 0,

    CC2640_AESCBCCOUNT
} CC2640_AESCBCName;

/*!
 *  @def    CC2640_AESCTRName
 *  @brief  Enum of AESCTR names
 */
typedef enum CC2640_AESCTRName {
    CC2640_AESCTR0 = 0,

    CC2640_AESCTRCOUNT
} CC2640_AESCTRName;

/*!
 *  @def    CC2640_AESECBName
 *  @brief  Enum of AESECB names
 */
typedef enum CC2640_AESECBName {
    CC2640_AESECB0 = 0,

    CC2640_AESECBCOUNT
} CC2640_AESECBName;

/*!
 *  @def    CC2640_AESCTRDRBGName
 *  @brief  Enum of AESCTRDRBG names
 */
typedef enum CC2640_AESCTRDRBGName {
    CC2640_AESCTRDRBG0 = 0,

    CC2640_AESCTRDRBGCOUNT
} CC2640_AESCTRDRBGName;

/*!
 *  @def    CC2640_GPTimerName
 *  @brief  Enum of GPTimer parts
 */
typedef enum CC2640_GPTimerName {
    CC2640_GPTIMER0A = 0,
    CC2640_GPTIMER0B,
    CC2640_GPTIMER1A,
    CC2640_GPTIMER1B,
    CC2640_GPTIMER2A,
    CC2640_GPTIMER2B,
    CC2640_GPTIMER3A,
    CC2640_GPTIMER3B,

    CC2640_GPTIMERPARTSCOUNT
} CC2640_GPTimerName;

/*!
 *  @def    CC2640_GPTimers
 *  @brief  Enum of GPTimers
 */
typedef enum CC2640_GPTimers {
    CC2640_GPTIMER0 = 0,
    CC2640_GPTIMER1,
    CC2640_GPTIMER2,
    CC2640_GPTIMER3,

    CC2640_GPTIMERCOUNT
} CC2640_GPTimers;

/*!
 *  @def    CC2640_I2CName
 *  @brief  Enum of I2C names
 */
typedef enum CC2640_I2CName {
    CC2640_I2C0 = 0,

    CC2640_I2CCOUNT
} CC2640_I2CName;

/*!
 *  @def    CC2640_UDMAName
 *  @brief  Enum of DMA buffers
 */
typedef enum CC2640_UDMAName {
    CC2640_UDMA0 = 0,

    CC2640_UDMACOUNT
} CC2640_UDMAName;

/*!
 *  @def    CC2640_WatchdogName
 *  @brief  Enum of Watchdogs
 */
typedef enum CC2640_WatchdogName {
    CC2640_WATCHDOG0 = 0,

    CC2640_WATCHDOGCOUNT
} CC2640_WatchdogName;

/*!
 *  @def    CC2640_TRNGName
 *  @brief  Enum of TRNG names on the board
 */
typedef enum CC2640_TRNGName {
    CC2640_TRNG0 = 0,
    CC2640_TRNGCOUNT
} CC2640_TRNGName;

#ifdef __cplusplus
}
#endif

#endif /* __CC2640_BOARD_H__ */
