/**
  ******************************************************************************
  * @file    BasicTools.h
  * @brief   Flexible UART and USB CDC handling library for STM32F103
  ******************************************************************************
  */

#ifndef BASIC_TOOLS_H
#define BASIC_TOOLS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"

#if defined(BASICTOOLS_USER_CONF_FILE)
  #include BASICTOOLS_USER_CONF_FILE
#else
  #include "basictools_conf.h"
#endif

/* Exported types ------------------------------------------------------------*/
typedef struct {
    UART_HandleTypeDef *huart;     /* UART handle pointer (NULL if not used) */
    uint8_t *rx_buf;               /* RX buffer for DMA */
    uint8_t *tx_buf;               /* TX buffer for DMA */
    uint8_t *data_buffer;          /* Circular buffer for TX data */
    uint16_t rx_head;              /* RX buffer head position */
    uint16_t tx_head;              /* TX circular buffer head position */
    uint16_t tx_tail;              /* TX circular buffer tail position */
    uint16_t rx_dma_size;          /* RX DMA buffer size */
    uint16_t tx_dma_size;          /* TX DMA buffer size */
    uint16_t tx_buffer_size;       /* TX circular buffer size */
    uint8_t dma_half_complete;     /* Flag for DMA half-complete state */
    uint8_t initialized;           /* Flag to mark if this UART is active */
} UART_Context;

typedef union {
    struct {
        uint8_t USB_Print_Busy:1;  /* Flag for USB print busy state */
        uint8_t reserved:7;
    };
    uint8_t All;
} System_Flags;


/* Exported variables --------------------------------------------------------*/
extern System_Flags SystemFlags;
extern volatile uint32_t USB_Connected;     /* USB connection timeout counter */

/* UART Functions ------------------------------------------------------------*/

/**
  * @brief  Register a UART with its buffers
  * @param  huart: Pointer to UART handle
  * @param  rx_buf: Pointer to RX buffer
  * @param  rx_size: Size of RX buffer
  * @param  tx_buf: Pointer to TX DMA buffer
  * @param  data_buffer: Pointer to TX circular buffer
  * @param  tx_dma_size: Size of TX DMA buffer
  * @param  tx_buffer_size: Size of TX circular buffer
  * @retval UART index in registry or UART_INVALID_INDEX if registration failed
  */
uint8_t UART_Register(UART_HandleTypeDef *huart,
                     uint8_t *rx_buf, uint16_t rx_size,
                     uint8_t *tx_buf, uint8_t *data_buffer,
                     uint16_t tx_dma_size, uint16_t tx_buffer_size);

/**
  * @brief  Read a single byte from UART
  * @param  huart: Pointer to UART handle
  * @retval Byte read from UART (0xFF if error)
  */
uint8_t SerialRead(UART_HandleTypeDef *huart);

/**
  * @brief  Read multiple bytes from UART
  * @param  huart: Pointer to UART handle
  * @param  Buffer: Buffer to store read data
  * @param  ByteCount: Number of bytes to read
  * @retval None
  */
void SerialReadBuf(UART_HandleTypeDef *huart, uint8_t* Buffer, uint16_t ByteCount);

/**
  * @brief  Flush UART receive buffer
  * @param  huart: Pointer to UART handle
  * @retval None
  */
void SerialReadFlush(UART_HandleTypeDef *huart);

/**
  * @brief  Get number of bytes available for reading
  * @param  huart: Pointer to UART handle
  * @retval Number of bytes available (0xFFFF if error)
  */
uint16_t SerialAvailable(UART_HandleTypeDef *huart);

/**
  * @brief  Write a single byte to UART
  * @param  huart: Pointer to UART handle
  * @param  Data: Byte to write
  * @retval None
  */
void SerialWrite(UART_HandleTypeDef *huart, uint8_t Data);

/**
  * @brief  Write multiple bytes to UART
  * @param  huart: Pointer to UART handle
  * @param  Data: Buffer containing data to write
  * @param  ByteCount: Number of bytes to write
  * @retval None
  */
void SerialWriteBuf(UART_HandleTypeDef *huart, uint8_t* Data, uint16_t ByteCount);

/**
  * @brief  Printf-style formatted output to UART
  * @param  huart: Pointer to UART handle
  * @param  fmt: Format string
  * @param  ...: Variable arguments
  * @retval None
  */
void SerialPrintf(UART_HandleTypeDef *huart, const char* fmt, ...);

/**
  * @brief  Handle UART TX DMA complete interrupt
  * @param  huart: Pointer to UART handle
  * @retval None
  */
void Handle_Uart_TX_DMA_Complete(UART_HandleTypeDef *huart);

/**
  * @brief  Handle UART TX DMA half-complete interrupt
  * @param  huart: Pointer to UART handle
  * @retval None
  */
void Handle_Uart_TX_DMA_HalfComplete(UART_HandleTypeDef *huart);

/* USB CDC Functions ---------------------------------------------------------*/

/**
  * @brief  Initialize USB CDC functionality
  * @param  usb_device: Pointer to USB device handle
  * @param  tx_buffer: Pointer to TX circular buffer
  * @param  tx_size: Size of TX buffer
  * @param  rx_buffer: Pointer to RX buffer for receiving data
  * @param  rx_size: Size of RX buffer
  * @param  tx_packet_buffer: Buffer for USB transmission
  * @retval None
  */
void USB_CDC_Init(USBD_HandleTypeDef *usb_device,
                 uint8_t *tx_buffer, uint16_t tx_size,
                 uint8_t *rx_buffer, uint16_t rx_size,
                 uint8_t *tx_packet_buffer);

/**
  * @brief  Set USB CDC configuration state
  * @param  configured: 1 if configured, 0 if not
  * @retval None
  */
void USB_CDC_SetConfiguration(uint8_t configured);

/**
  * @brief  Get number of bytes available for reading from USB CDC
  * @retval Number of bytes available
  */
uint16_t USBSerialAvailable(void);

/**
  * @brief  Read a single byte from USB CDC
  * @retval Byte read from USB CDC (0xFF if no data)
  */
uint8_t USBSerialRead(void);

/**
  * @brief  Read multiple bytes from USB CDC
  * @param  Buffer: Buffer to store read data
  * @param  ByteCount: Number of bytes to read
  * @retval None
  */
void USBSerialReadBuf(uint8_t* Buffer, uint16_t ByteCount);

/**
  * @brief  Flush USB CDC receive buffer
  * @retval None
  */
void USBSerialReadFlush(void);

/**
  * @brief  Write a single byte to USB CDC
  * @param  Data: Byte to write
  * @retval None
  */
void USBSerialWrite(uint8_t Data);

/**
  * @brief  Write multiple bytes to USB CDC
  * @param  Data: Buffer containing data to write
  * @param  ByteCount: Number of bytes to write
  * @retval None
  */
void USBSerialWriteBuf(uint8_t* Data, uint16_t ByteCount);

/**
  * @brief  Printf-style formatted output to USB CDC
  * @param  fmt: Format string
  * @param  ...: Variable arguments
  * @retval None
  */
void USBSerialPrintf(const char* fmt, ...);

/**
  * @brief  Process USB CDC transmissions (call from timer OC interrupt)
  * @retval None
  */
void USB_CDC_TimerCallback(void);

/**
  * @brief  USB CDC receive callback (call from usbd_cdc_if.c)
  * @param  Buf: Buffer containing received data
  * @param  Len: Length of received data
  * @retval None
  */
void USB_CDC_ReceiveCallback(uint8_t* Buf, uint32_t Len);

/* Utility Functions ---------------------------------------------------------*/

/**
  * @brief  Calculate remaining bytes in circular buffer
  * @param  HeadPointer: Pointer to head index
  * @param  TailPointer: Pointer to tail index
  * @param  Buffer_SIZE: Size of circular buffer
  * @retval Number of bytes available in buffer
  */
uint16_t CircBufRemainingByte(uint16_t* HeadPointer, uint16_t* TailPointer, uint16_t Buffer_SIZE);

/**
  * @brief  Copy data from circular buffer to memory
  * @param  MemoryToWrite: Destination memory
  * @param  CircularBufferToCopy: Source circular buffer
  * @param  Pointer: Pointer to current position in circular buffer (will be updated)
  * @param  ByteCount: Number of bytes to copy
  * @param  BufferSize: Size of circular buffer
  * @retval None
  */
void CircBufToMemcpy(uint8_t* MemoryToWrite, uint8_t* CircularBufferToCopy,
                    uint16_t* Pointer, uint16_t ByteCount, uint16_t BufferSize);

/**
  * @brief  Copy data from memory to circular buffer
  * @param  CircularBufferToWrite: Destination circular buffer
  * @param  MemoryToCopy: Source memory
  * @param  Pointer: Pointer to current position in circular buffer (will be updated)
  * @param  ByteCount: Number of bytes to copy
  * @param  BufferSize: Size of circular buffer
  * @retval None
  */
void MemToCircBufcpy(uint8_t* CircularBufferToWrite, uint8_t* MemoryToCopy,
                    uint16_t* Pointer, uint16_t ByteCount, uint16_t BufferSize);


/**
 * @brief  Returns the number of milliseconds since the system started
 * @retval Millisecond count
 */
uint32_t millis(void);

/**
 * @brief  Returns the number of microseconds since the system started
 * @note   Offers microsecond precision but may have small measurement errors
 *         due to function call overhead
 * @retval Microsecond count
 */
uint32_t micros(void);

#ifdef __cplusplus
}
#endif

#endif /* BASIC_TOOLS_H */
