#ifndef __DRIVE_SETTINGS_H__
#define __DRIVE_SETTINGS_H__

// PWM
#define DRIVE_PWM_LEFT_F  (2) // Left Motor - Forwards
#define DRIVE_PWM_LEFT_B  (3) // Left Motor - Backwards

#define DRIVE_PWM_RIGHT_F (4) // Right Motor - Forwards
#define DRIVE_PWM_RIGHT_B (5) // Right Motor - Backwards

// UART
#define DRIVE_UART_TX     (0)
#define DRIVE_UART_RX     (1)
#define DRIVE_UART_BAUD   (115200)
#define DRIVE_UART_ID     uart0

// Status LED
#define DRIVE_STAT_LED    (15)

// Static Buffers

#define DRIVE_BUF_LEN_RX  (64)

#endif // __DRIVE_SETTINGS_H__
