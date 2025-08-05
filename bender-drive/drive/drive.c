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

static int get_speeds(char* strBuf, int16_t* pLpwm, int16_t* pRpwm);

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
   int16_t lpwm = 0;
   int16_t rpwm = 0;
   uint8_t rx = 0;
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

      if (get_speeds((char*)s_uartRx, &lpwm, &rpwm) != 0)
      {
         reset_rx_buf();
         return;
      }

      if (pHandle->cbSetPwm(lpwm, rpwm) != 0)
      {
         pHandle->state = DRIVE_ST_ERROR;
         reset_rx_buf();
         return;
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

// ========= Private Functions =========

int get_speeds(char* strBuf, int16_t* pLpwm, int16_t* pRpwm)
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