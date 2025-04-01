/**
  ******************************************************************************
  * @file    basictools_conf.h
  * @brief   Configuration file for the BasicTools library
  ******************************************************************************
  * This file contains the configuration parameters for the BasicTools library
  * which handles UART and USB CDC communication.
  */

#ifndef BASICTOOLS_CONF_H
#define BASICTOOLS_CONF_H

/* General Configuration ------------------------------------------------------*/
#define MAX_UART_INSTANCES      3    /* Maximum supported UART peripherals */
#define UART_INVALID_INDEX      0xFF /* Invalid UART index indicator */
#define UART_PRINTF_BUFFER_SIZE 128  /* Buffer size for printf functionality */

/* UART1 Configuration -------------------------------------------------------*/
#define UART1_TX_Buffer_SIZE 1024    /* Size of UART1 circular TX buffer */
#define UART1_RX_DMA_SIZE    1024    /* Size of UART1 DMA RX buffer */
#define UART1_TX_DMA_SIZE    32      /* Size of UART1 DMA TX buffer.
                                        Set to ensure 16/115200*10 = 1.388ms interrupt interval */

/* UART2 Configuration -------------------------------------------------------*/
#define UART2_TX_Buffer_SIZE 1024    /* Size of UART2 circular TX buffer */
#define UART2_RX_DMA_SIZE    1024    /* Size of UART2 DMA RX buffer */
#define UART2_TX_DMA_SIZE    32      /* Size of UART2 DMA TX buffer
                                        Set to ensure 16/115200*10 = 1.388ms interrupt interval */

/* UART3 Configuration -------------------------------------------------------*/
#define UART3_TX_Buffer_SIZE 1024    /* Size of UART3 circular TX buffer */
#define UART3_RX_DMA_SIZE    32      /* Size of UART3 DMA RX buffer */
#define UART3_TX_DMA_SIZE    16      /* Size of UART3 DMA TX buffer
                                        Set to ensure 16/115200*10 = 0.6394ms interrupt interval */

/* USB CDC Configuration -----------------------------------------------------*/
#define USB_CDC_BUFFER_SIZE     2048 /* USB CDC circular buffer size */
#define USB_CDC_TX_PACKET_SIZE  1024 /* Max size for USB transmission */
#define USB_CDC_RX_BUFFER_SIZE  2048 /* USB CDC receive buffer size */
#define USB_TIMEOUT_COUNT       100  /* USB timeout counter (5ms at 50μs intervals) */

#endif /* BASICTOOLS_CONF_H */
