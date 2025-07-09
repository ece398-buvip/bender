#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"

#include "drive/drive.h"

// ========= Private Function Declarations =========

static drive_err_t hal_init();
static drive_err_t hal_set_pwm(uint8_t left, uint8_t right);
static drive_err_t hal_set_stat_led(uint8_t stat);
static uint32_t hal_get_time_ms();

// ========= Private Variables =========

static drive_t benderDrive = {};

// ========= Entry Point / Main =========

int
main()
{
   stdio_init_all();

   benderDrive.pInit = hal_init;
   benderDrive.pSetPwm = hal_set_pwm;
   benderDrive.pSetLed = hal_set_stat_led;
   benderDrive.pGetTimeMs = hal_get_time_ms;

   while (1)
   {
      if (drive_fsm(&benderDrive) != DRIVE_OK)
      {
         break;
      }
   }

   hal_set_stat_led(0);
   hal_set_pwm(0, 0);
}

// ========= Private Function Definitions =========

drive_err_t
hal_init()
{
   // TODO
   // - Initialize PWM for motors
   // - Initialize GPIO for stat LED

   gpio_init(PICO_DEFAULT_LED_PIN);
   gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

   return DRIVE_ERR;
}

drive_err_t
hal_set_pwm(uint8_t left, uint8_t right)
{
   // TODO
   // Set PWM duty cycle of both motors (left and right)

   return DRIVE_OK;
}

drive_err_t
hal_set_stat_led(uint8_t stat)
{
   // TODO
   // Set the GPIO status of the LED pin
   gpio_put(PICO_DEFAULT_LED_PIN, stat);

   return DRIVE_OK;
}

uint32_t
hal_get_time_ms()
{
   return to_ms_since_boot(get_absolute_time());
}