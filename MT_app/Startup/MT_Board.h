#ifndef __MT_BOARD_H
#define __MT_BOARD_H

#define Board_CC2640

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/ADC.h>
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>

#include <ti/drivers/Board.h>

#define Board_initGeneral()     Board_init()  /* deprecated */

#include "CC2640.h"

#define Board_shutDownExtFlash() CC2640_shutDownExtFlash()
#define Board_wakeUpExtFlash() CC2640_wakeUpExtFlash()

/* These #defines allow us to reuse TI-RTOS across other device families */

#define Board_CRYPTO0           CC2640_CRYPTO0
#define Board_AESCCM0           CC2640_AESCCM0
#define Board_AESGCM0           CC2640_AESGCM0
#define Board_AESCBC0           CC2640_AESCBC0
#define Board_AESCTR0           CC2640_AESCTR0
#define Board_AESECB0           CC2640_AESECB0
#define Board_AESCTRDRBG0       CC2640_AESCTRDRBG0

#define Board_PIN_CD            CC2640_PIN_CD
#define Board_PIN_CHARGE        CC2640_PIN_CHARGE
#define Board_DIO3              CC2640_DIO3
#define Board_DIO4              CC2640_DIO4
#define Board_DIO5              CC2640_DIO5_TDO
#define Board_DIO6              CC2640_DIO6_TDI
#define Board_PIN_ALERT         CC2640_PIN_ALERT
#define Board_PIN_INTX          CC2640_PIN_INTX
#define Board_PIN_INTY          CC2640_PIN_INTY
#define Board_PIN_DIO10         CC2640_PIN_DIO10
#define Board_PIN_DIO11         CC2640_PIN_DIO11
#define Board_PIN_DIO12         CC2640_PIN_DIO12
#define Board_PIN_DIO13         CC2640_PIN_DIO13
#define Board_PIN_DIO14         CC2640_PIN_DIO14

#define Board_GPTIMER0A         CC2640_GPTIMER0A
#define Board_GPTIMER0B         CC2640_GPTIMER0B
#define Board_GPTIMER1A         CC2640_GPTIMER1A
#define Board_GPTIMER1B         CC2640_GPTIMER1B
#define Board_GPTIMER2A         CC2640_GPTIMER2A
#define Board_GPTIMER2B         CC2640_GPTIMER2B
#define Board_GPTIMER3A         CC2640_GPTIMER3A
#define Board_GPTIMER3B         CC2640_GPTIMER3B

#define Board_I2C0              CC2640_I2C0
#define Board_I2C_TMP           Board_I2C0

#define Board_NVSINTERNAL       CC2640_NVSCC26XX0
#define Board_NVSEXTERNAL       CC2640_NVSSPI25X0

#define Board_WATCHDOG0         CC2640_WATCHDOG0

#ifdef __cplusplus
}
#endif

#endif /* __MT_BOARD_H */
