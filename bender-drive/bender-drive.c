#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/pwm.h"

#include "drive/drive.h"
#include "settings.h"

// ========= Private Function Declarations =========

static drive_err_t hal_init();
static int hal_uart_get_c(); // Blocks until a character is read
static uint8_t hal_uart_rx_ready(); // Returns non-zero if character is ready
static drive_err_t hal_set_pwm(uint16_t left, uint16_t right);
static drive_err_t hal_set_stat_led(uint8_t stat);
static uint32_t hal_get_time_ms();

// ========= Private Variables =========

static drive_t s_benderDrive = {};
static uint s_pwmSlice = 0;

// ========= Entry Point / Main =========

int
main()
{
   stdio_init_all();

   s_benderDrive.cbInit = hal_init;
   s_benderDrive.cbSetPwm = hal_set_pwm;
   s_benderDrive.cbSetLed = hal_set_stat_led;
   s_benderDrive.cbGetTimeMs = hal_get_time_ms;
   s_benderDrive.cbUartGetC = hal_uart_get_c;

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
   gpio_init(PICO_DEFAULT_LED_PIN);
   gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

   // Initialize PWM
   cfg = pwm_get_default_config();
   pwm_config_set_clkdiv_int(&cfg, 150);
   pwm_config_set_wrap(&cfg, 255);

   gpio_set_function(DRIVE_PWM_A_GPIO, GPIO_FUNC_PWM);
   gpio_set_function(DRIVE_PWM_B_GPIO, GPIO_FUNC_PWM);

   s_pwmSlice = pwm_gpio_to_slice_num(DRIVE_PWM_A_GPIO);

   pwm_set_chan_level(s_pwmSlice, PWM_CHAN_A, 0);
   pwm_set_chan_level(s_pwmSlice, PWM_CHAN_B, 0);

   pwm_init(s_pwmSlice, &cfg, true);
   pwm_set_enabled(s_pwmSlice, true);

   // Initialize UART
   uart_init(DRIVE_UART_ID, DRIVE_UART_BAUD);
   gpio_set_function(DRIVE_UART_TX, UART_FUNCSEL_NUM(DRIVE_UART_ID, DRIVE_UART_TX));
   gpio_set_function(DRIVE_UART_RX, UART_FUNCSEL_NUM(DRIVE_UART_ID, DRIVE_UART_RX));

   return DRIVE_OK;
}

drive_err_t
hal_set_pwm(uint16_t left, uint16_t right)
{
   printf("L: [%d] R: [%d]\n", left, right);
   // END
   pwm_set_chan_level(s_pwmSlice, 0, left);
   pwm_set_chan_level(s_pwmSlice, 1, right);

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

int hal_uart_get_c()
{
   int rx = stdio_getchar_timeout_us(0);
   if (rx == PICO_ERROR_TIMEOUT)
   {
      return -1;
   }

   return rx;
}