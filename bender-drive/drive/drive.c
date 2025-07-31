#include "drive.h"
#include "settings.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

// ========= Private Variables =========

uint8_t s_uartRx[DRIVE_BUF_LEN_RX] = {0};
size_t s_rxPtr = 0;

// ========= Private Definitions =========

static drive_err_t null_check(drive_t *pHandle);
static void boot_task(drive_t *pHandle);
static void run_task(drive_t *pHandle);
static void error_task(drive_t *pHandle);
static void reset_rx_buf();

// ========= PUBLIC API =========

drive_err_t
DriveFSM(drive_t *pHandle)
{
   if (null_check(pHandle) != DRIVE_OK)
   {
      return DRIVE_ERR_NULLPTR;
   }

   switch (pHandle->state)
   {
      case DRIVE_ST_BOOT:
         boot_task(pHandle);
         break;
      case DRIVE_ST_RUN:
         run_task(pHandle);
         break;
      case DRIVE_ST_ERROR:
         error_task(pHandle);
         break;

      default:
         pHandle->state = DRIVE_ST_ERROR;
         break;
   }

   return DRIVE_OK;
}

// ========= Private Definitions =========

/*
 * Expecting that pHandle is not null in 'private' code
 *   it is public-facing code's responsibility to check that.
 * 
 * Public code is also responsible for checking the validity of
 *   the handle (all callbacks populated and stuff)
 */

drive_err_t
null_check(drive_t *pHandle)
{
   DRIVE_RET_IF_NULL(pHandle);
   DRIVE_RET_IF_NULL(pHandle->cbInit);
   DRIVE_RET_IF_NULL(pHandle->cbSetPwm);
   DRIVE_RET_IF_NULL(pHandle->cbSetLed);
   DRIVE_RET_IF_NULL(pHandle->cbGetTimeMs);
   DRIVE_RET_IF_NULL(pHandle->cbUartGetC);

   return DRIVE_OK;
}

void
boot_task(drive_t *pHandle)
{
   if (pHandle->cbInit() == DRIVE_ERR)
   {
      pHandle->state = DRIVE_ST_ERROR;
      return;
   }

   pHandle->state = DRIVE_ST_RUN;
}

void
run_task(drive_t *pHandle)
{
   uint16_t lpwm = 0;
   uint16_t rpwm = 0;
   uint8_t rx = 0;
   char* lToken = NULL;
   char* rToken = NULL;
   int next = 0;

   // Make sure light is on to indicate to user firmware is running
   pHandle->cbSetLed(1);

   next = pHandle->cbUartGetC();

   if (next >= 0)
   {
      rx = (uint8_t)next;

      if (s_rxPtr + 1 >= sizeof(s_uartRx))
      {
         reset_rx_buf();
         return;
      }

      s_uartRx[s_rxPtr] = rx;
      s_rxPtr++;

      // If the rx character is NOT a newline, don't process
      if (rx != '\n')
      {
         return;
      }

      // Check the line length - we except 7 characters. If not that, delete and don't process
      if (strlen(s_uartRx) != 8)
      {
         reset_rx_buf();
         return;
      }

      // Delete the newline, we don't need it
      s_uartRx[s_rxPtr - 1] = '\0';

      // Process the command (we only have one)
      lToken = s_uartRx;
      rToken = strchr(s_uartRx, ',');
      rToken ++;

      // Only process if rToken was not null (found a , character)
      if (rToken != NULL)
      {
         errno = 0;
         lpwm = strtol(lToken, NULL, 10);
         // Increment the rToken pointer by 1 - the number starts after the comma
         rpwm = strtol(rToken, NULL, 10);

         if (errno == 0)
         {
            // Only apply PWM if there was no errors
            // Set PWM with balues parsed from command line
            if (pHandle->cbSetPwm(lpwm, rpwm) != DRIVE_OK)
            {
               pHandle->state = DRIVE_ST_ERROR;
               return;
            }

         }
      }

      reset_rx_buf();
   }

}

void
error_task(drive_t *pHandle)
{
   static uint8_t ledState = 0;

   // Attempt to set pwm to zero in error state
   pHandle->cbSetPwm(0, 0);

   // Blink an LED @ frequency given by the DRIVE_LED_BLINK_MS period
   if (pHandle->cbGetTimeMs() - pHandle->lastBlink_ms > DRIVE_LED_BLINK_MS)
   {
      printf("Swapping LED\n");
      ledState = !ledState;
      pHandle->cbSetLed(ledState);

      pHandle->lastBlink_ms = pHandle->cbGetTimeMs();
   }
}

void
reset_rx_buf()
{
   memset(s_uartRx, 0, sizeof(s_uartRx));
   s_rxPtr = 0;
}