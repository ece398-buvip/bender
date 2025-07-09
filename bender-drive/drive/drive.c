#include "drive.h"

// ========= Private Definitions =========

static drive_err_t null_check(drive_t *pHandle);
static void boot_task(drive_t *pHandle);
static void run_task(drive_t *pHandle);
static void error_task(drive_t *pHandle);

// ========= PUBLIC API =========

drive_err_t
drive_fsm(drive_t *pHandle)
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

drive_err_t null_check(drive_t *pHandle)
{
   DRIVE_RET_IF_NULL(pHandle);
   DRIVE_RET_IF_NULL(pHandle->pInit);
   DRIVE_RET_IF_NULL(pHandle->pSetPwm);
   DRIVE_RET_IF_NULL(pHandle->pSetLed);
   DRIVE_RET_IF_NULL(pHandle->pGetTimeMs);

   return DRIVE_OK;
}

void
boot_task(drive_t *pHandle)
{
   if (pHandle->pInit() == DRIVE_ERR)
   {
      pHandle->state = DRIVE_ST_ERROR;
      return;
   }
}

void
run_task(drive_t *pHandle)
{
   uint8_t lpwm = 0;
   uint8_t rpwm = 0;

   // TODO
   // - Receive data from UART interface

   if (pHandle->pSetPwm(lpwm, rpwm) != DRIVE_OK)
   {
      pHandle->state = DRIVE_ST_ERROR;
      return;
   }
}

void
error_task(drive_t *pHandle)
{
   static uint8_t ledState = 0;

   // Blink an LED @ frequency given by the DRIVE_LED_BLINK_MS period
   if (pHandle->pGetTimeMs() - pHandle->lastBlink_ms > DRIVE_LED_BLINK_MS)
   {
      ledState = !ledState;
      pHandle->pSetLed(ledState);

      pHandle->lastBlink_ms = pHandle->pGetTimeMs();
   }
}