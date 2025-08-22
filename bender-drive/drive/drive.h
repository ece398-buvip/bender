/**
 * @file drive.h
 * @author Jacob Simeone (jsimeone@mail.bradley.edu)
 * @brief Core drivetrain state machine for the "bender" rover
 * @date 2025-07-09
 * 
 */
#ifndef __DRIVE_H__
#define __DRIVE_H__

#include <stddef.h>
#include <stdint.h>

typedef enum
{
   DRIVE_OK = 0,
   DRIVE_ERR = -1,
   DRIVE_ERR_NULLPTR = -2,
} drive_err_t;

typedef enum
{
   DRIVE_ST_BOOT,
   DRIVE_ST_RUN,
   DRIVE_ST_DIS, // Disabled state
   DRIVE_ST_ERROR,
} drive_state_t;

typedef drive_err_t (*drive_init_fn)();
typedef drive_err_t (*drive_set_pwm_fn)(int16_t leftPwm, int16_t rightPwm);
typedef drive_err_t (*drive_set_stat_led_fn)(uint8_t stat);
typedef uint32_t (*drive_get_time_ms_fn)();
typedef int (*drive_uart_get_c_fn)();
typedef drive_err_t (*drive_can_tx_fn)(uint8_t data, uint8_t len);

typedef struct
{
   drive_state_t state;
   uint32_t lastBlink_ms;
   uint32_t lastHeartBeat_ms;

   drive_init_fn cbInit;
   drive_set_pwm_fn cbSetPwm;
   drive_set_stat_led_fn cbSetLed;
   drive_get_time_ms_fn cbGetTimeMs;
   drive_uart_get_c_fn cbUartGetC;
   drive_can_tx_fn cbCanTx;
} drive_t;

drive_err_t DriveFSM(drive_t *pHandle);

#define DRIVE_RET_IF_NULL(ptr)                                                                                         \
   if (ptr == NULL)                                                                                                    \
   {                                                                                                                   \
      return DRIVE_ERR_NULLPTR;                                                                                        \
   }

#endif // __DRIVE_H__