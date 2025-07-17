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

#define DRIVE_LED_BLINK_MS (250)

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
   DRIVE_ST_ERROR,
} drive_state_t;

typedef drive_err_t (*drive_init_cb_t)();
typedef drive_err_t (*drive_set_pwm_cb_t)(uint16_t leftPwm, uint16_t rightPwm);
typedef drive_err_t (*drive_set_stat_led_cb_t)(uint8_t stat);
typedef uint32_t (*drive_get_time_ms_cb_t)();
typedef int (*drive_uart_get_c_cb_t)();

typedef struct
{
   drive_state_t state;
   uint32_t lastBlink_ms;

   drive_init_cb_t cbInit;
   drive_set_pwm_cb_t cbSetPwm;
   drive_set_stat_led_cb_t cbSetLed;
   drive_get_time_ms_cb_t cbGetTimeMs;
   drive_uart_get_c_cb_t cbUartGetC;
} drive_t;

drive_err_t DriveFSM(drive_t *pHandle);

#define DRIVE_RET_IF_NULL(ptr)                                                                                         \
   if (ptr == NULL)                                                                                                    \
   {                                                                                                                   \
      return DRIVE_ERR_NULLPTR;                                                                                        \
   }

#endif // __DRIVE_H__