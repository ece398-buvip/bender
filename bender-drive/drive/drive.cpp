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
static void disabled_task(drive_t *pHandle);
static void error_task(drive_t *pHandle);
static void push_rx_buf(char c);
static void reset_rx_buf();

static int get_speeds(char *strBuf, int16_t *pLpwm, int16_t *pRpwm);
static int get_heartbeat(char *strBuf, uint8_t *pHeartbeatPresent);
static uint8_t check_heartbeat(drive_t *pHandle);

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
      case DRIVE_ST_DIS:
         disabled_task(pHandle);
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
   int16_t lpwm = 0;
   int16_t rpwm = 0;
   uint8_t rx = 0;
   int next = 0;
   uint8_t heartbeatPresent = 0;

   // Check that heartbeat signal is present
   if (!check_heartbeat(pHandle))
   {
      pHandle->state = DRIVE_ST_DIS;
      reset_rx_buf();
      return;
   }

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

      push_rx_buf(rx);

      // If the rx character is NOT a newline, don't process
      if (rx != '\n')
      {
         return;
      }

      if (get_speeds((char *)s_uartRx, &lpwm, &rpwm) == 0)
      {
         if (pHandle->cbSetPwm(lpwm, rpwm) != 0)
         {
            pHandle->state = DRIVE_ST_ERROR;
         }
      }
      else if (get_heartbeat((char *)s_uartRx, &heartbeatPresent) == 0)
      {
         // TODO: A heartbeat was received, reset the timer
         if (heartbeatPresent)
         {
            pHandle->lastHeartBeat_ms = pHandle->cbGetTimeMs();
         }
      }

      reset_rx_buf();
      return;
   }
}

void
disabled_task(drive_t *pHandle)
{
   uint32_t now = pHandle->cbGetTimeMs();
   uint32_t diff = now - pHandle->lastBlink_ms;
   int rx = 0;

   // Put robot in a safe state
   pHandle->cbSetPwm(0, 0);

   // Listen for an incoming heartbeat signal (to re-enable)
   do
   {
      rx = pHandle->cbUartGetC();
      if (rx >= 0)
      {
         if (rx == 'H')
         {
            pHandle->lastHeartBeat_ms = pHandle->cbGetTimeMs();
            pHandle->state = DRIVE_ST_RUN;
         }
      }
   } while (rx >= 0);

   // LED Blink code - 1 second period, ON for DRIVE_LED_DIS_BLINK_MS
   if (diff > 1000)
   {
      pHandle->lastBlink_ms = now;
   }
   else if (diff > DRIVE_LED_DIS_BLINK_MS)
   {
      pHandle->cbSetLed(0);
   }
   else
   {
      pHandle->cbSetLed(1);
   }
}

void
error_task(drive_t *pHandle)
{
   int rx = 0;
   static uint8_t ledState = 0;

   // Attempt to set pwm to zero in error state
   pHandle->cbSetPwm(0, 0);

   // Blink an LED @ frequency given by the DRIVE_LED_BLINK_MS period
   if (pHandle->cbGetTimeMs() - pHandle->lastBlink_ms > DRIVE_LED_ERR_BLINK_MS)
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

void
push_rx_buf(char c)
{

   s_uartRx[s_rxPtr] = c;
   s_rxPtr++;
}

// ========= Helper Functions =========

int
get_speeds(char *strBuf, int16_t *pLpwm, int16_t *pRpwm)
{
   char *lToken = NULL;
   char *rToken = NULL;

   if (strBuf == NULL || pLpwm == NULL || pRpwm == NULL)
   {
      return -1; // Error: null pointer
   }

   if (strlen(strBuf) < 8 || strlen(strBuf) > 10)
   {
      return -2;
   }

   lToken = strBuf;
   rToken = strchr(strBuf, ',');

   if (rToken == NULL)
   {
      return -3; // Error: no comma found
   }

   // String we are interested in starts after the comma
   rToken++;

   // Clear any previous errors
   errno = 0;

   *pLpwm = strtol(lToken, NULL, 10);
   *pRpwm = strtol(rToken, NULL, 10);

   if (errno != 0)
   {
      return -4; // Error: conversion failed
   }

   return 0;
}

int
get_heartbeat(char *strBuf, uint8_t *pHeartbeatPresent)
{
   if (strBuf == NULL)
   {
      return -1;
   }

   if (strBuf[0] == 'H')
   {
      *pHeartbeatPresent = 1;
   }

   return 0;
}

uint8_t
check_heartbeat(drive_t *pHandle)
{
   if (pHandle == NULL)
   {
      // Treat null handle like failed heartbeat
      return 0;
   }

   // TODO: DEBUGGING 
   // disable for now
   // if (pHandle->lastHeartBeat_ms == 0)
   // {
   //    // 0 means no heartbeat has been received yet. Only start
   //    //   monitoring on first heartbeat - return OK for now
   //    return 1;
   // }
   // END

   if ((pHandle->cbGetTimeMs() - pHandle->lastHeartBeat_ms) > DRIVE_HB_MAX_INTERVAL)
   {
      return 0;
   }

   return 1;
}