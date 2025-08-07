#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"

#include "drive/drive.h"
#include "settings.h"

// ========= Private Function Declarations =========

static drive_err_t hal_init();
static int hal_uart_get_c(); // Blocks until a character is read
static drive_err_t hal_set_pwm(int16_t left, int16_t right);
static drive_err_t hal_set_stat_led(uint8_t stat);
static uint32_t hal_get_time_ms();

// ========= Private Variables =========

static drive_t s_benderDrive = {};
static uint s_pwmLeftSlice = 0;
static uint s_pwmRightSlice = 0;

// ========= Entry Point / Main =========

int
main()
{
   stdio_init_all();

   // Wait 2 seconds to start up - prevents initializing UART 
   //   interface and making Pi think there's a console to boot to
   sleep_ms(2000);

   s_benderDrive.cbInit = hal_init;
   s_benderDrive.cbSetPwm = hal_set_pwm;
   s_benderDrive.cbSetLed = hal_set_stat_led;
   s_benderDrive.cbGetTimeMs = hal_get_time_ms;
   s_benderDrive.cbUartGetC = hal_uart_get_c;
   s_benderDrive.lastHeartBeat_ms = 0;

   while (1)
   {
      if (DriveFSM(&s_benderDrive) != DRIVE_OK)
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
   pwm_config cfg = {0};

   // Initialize Status LED
   gpio_init(DRIVE_STAT_LED);
   gpio_set_dir(DRIVE_STAT_LED, GPIO_OUT);

   // Initialize PWM
   cfg = pwm_get_default_config();
   pwm_config_set_clkdiv_int(&cfg, 150);
   pwm_config_set_wrap(&cfg, 255);

   // Left motor - forwards/back
   gpio_set_function(DRIVE_PWM_LEFT_F, GPIO_FUNC_PWM);
   gpio_set_function(DRIVE_PWM_LEFT_B, GPIO_FUNC_PWM);

   // Right motor - forwards/back
   gpio_set_function(DRIVE_PWM_RIGHT_F, GPIO_FUNC_PWM);
   gpio_set_function(DRIVE_PWM_RIGHT_B, GPIO_FUNC_PWM);

   s_pwmLeftSlice = pwm_gpio_to_slice_num(DRIVE_PWM_LEFT_F);
   s_pwmRightSlice = pwm_gpio_to_slice_num(DRIVE_PWM_RIGHT_F);

   pwm_set_chan_level(s_pwmLeftSlice, PWM_CHAN_A, 0);
   pwm_set_chan_level(s_pwmLeftSlice, PWM_CHAN_B, 0);

   pwm_set_chan_level(s_pwmRightSlice, PWM_CHAN_A, 0);
   pwm_set_chan_level(s_pwmRightSlice, PWM_CHAN_B, 0);

   pwm_init(s_pwmLeftSlice, &cfg, true);
   pwm_init(s_pwmRightSlice, &cfg, true);

   pwm_set_enabled(s_pwmLeftSlice, true);
   pwm_set_enabled(s_pwmRightSlice, true);

   // Initialize UART
   uart_init(DRIVE_UART_ID, DRIVE_UART_BAUD);
   // uart_set_hw_flow(DRIVE_UART_ID, false, false);
   uart_set_format(DRIVE_UART_ID, 8, 1, UART_PARITY_NONE);
   // uart_set_fifo_enabled(DRIVE_UART_ID, false);
   gpio_set_function(DRIVE_UART_TX, UART_FUNCSEL_NUM(DRIVE_UART_ID, DRIVE_UART_TX));
   gpio_set_function(DRIVE_UART_RX, UART_FUNCSEL_NUM(DRIVE_UART_ID, DRIVE_UART_RX));

   return DRIVE_OK;
}

drive_err_t
hal_set_pwm(int16_t left, int16_t right)
{
   // TODO: Debugging
   printf("L: [%d] R: [%d]\n", left, right);
   // END

   if (left < 0 )
   {
      pwm_set_chan_level(s_pwmLeftSlice, PWM_CHAN_A, 0);
      pwm_set_chan_level(s_pwmLeftSlice, PWM_CHAN_B, (uint16_t)(left * -1));
   }
   else
   {
      pwm_set_chan_level(s_pwmLeftSlice, PWM_CHAN_A, (uint16_t)(left));
      pwm_set_chan_level(s_pwmLeftSlice, PWM_CHAN_B, 0);
   }

   if (right < 0)
   {
      pwm_set_chan_level(s_pwmRightSlice, PWM_CHAN_A, 0);
      pwm_set_chan_level(s_pwmRightSlice, PWM_CHAN_B, (uint16_t)(right * -1));
   }
   else
   {
      pwm_set_chan_level(s_pwmRightSlice, PWM_CHAN_A, (uint16_t)(right));
      pwm_set_chan_level(s_pwmRightSlice, PWM_CHAN_B, 0);
   }

   return DRIVE_OK;
}

drive_err_t
hal_set_stat_led(uint8_t stat)
{
   // Set the GPIO status of the LED pin
   gpio_put(DRIVE_STAT_LED, stat);

   return DRIVE_OK;
}

uint32_t
hal_get_time_ms()
{
   return to_ms_since_boot(get_absolute_time());
}

int hal_uart_get_c()
{
   // int rx = stdio_getchar_timeout_us(0);
   // if (rx == PICO_ERROR_TIMEOUT)
   // {
   //    return -1;
   // }

   if (!uart_is_readable(DRIVE_UART_ID))
   {
      return -1;
   }
   

   return uart_getc(DRIVE_UART_ID);
}