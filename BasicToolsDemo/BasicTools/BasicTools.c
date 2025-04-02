/**
  ******************************************************************************
  * @file    BasicTools.c
  * @brief   Flexible UART and USB CDC handling library for STM32F103
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "BasicTools.h"

/* Private variables ---------------------------------------------------------*/
static UART_Context uart_contexts[MAX_UART_INSTANCES];
static uint8_t uart_count = 0;
static uint32_t MicrosMax = 0;
static uint32_t MicrosDivisor = 0;

/* USB CDC context variables */
static USBD_HandleTypeDef *usb_device = NULL;
static uint8_t *usb_tx_buffer = NULL;     /* USB transmission packet buffer */
static uint8_t *usb_tx_data_buffer = NULL; /* Circular buffer for TX data */
static uint8_t *usb_rx_buffer = NULL;     /* Circular buffer for RX data */
static uint16_t usb_tx_head = 0;          /* TX circular buffer head */
static uint16_t usb_tx_tail = 0;          /* TX circular buffer tail */
static uint16_t usb_rx_head = 0;          /* RX circular buffer head */
static uint16_t usb_rx_tail = 0;          /* RX circular buffer tail */
static uint16_t usb_tx_buffer_size = 0;   /* TX circular buffer size */
static uint16_t usb_rx_buffer_size = 0;   /* RX circular buffer size */
static uint8_t usb_initialized = 0;       /* Flag indicating CDC is initialized */

/* Exported variables --------------------------------------------------------*/
System_Flags SystemFlags = {0};
volatile uint32_t USB_Connected = 0;  /* USB connection timeout counter */

/* Private function prototypes -----------------------------------------------*/
static UART_Context* GetUartContext(UART_HandleTypeDef *huart);

/* UART Functions ------------------------------------------------------------*/

/**
  * @brief  Register a UART with its buffers
  */
uint8_t UART_Register(UART_HandleTypeDef *huart,
                     uint8_t *rx_buf, uint16_t rx_size,
                     uint8_t *tx_buf, uint8_t *data_buffer,
                     uint16_t tx_dma_size, uint16_t tx_buffer_size) {

    /* Check parameters */
    if (uart_count >= MAX_UART_INSTANCES) {
        return UART_INVALID_INDEX;
    }

    if (huart == NULL || rx_buf == NULL || tx_buf == NULL || data_buffer == NULL) {
        return UART_INVALID_INDEX;
    }

    uint8_t index = uart_count++;

    /* Initialize the context */
    uart_contexts[index].huart = huart;
    uart_contexts[index].rx_buf = rx_buf;
    uart_contexts[index].tx_buf = tx_buf;
    uart_contexts[index].data_buffer = data_buffer;
    uart_contexts[index].rx_head = 0;
    uart_contexts[index].tx_head = 0;
    uart_contexts[index].tx_tail = 0;
    uart_contexts[index].rx_dma_size = rx_size;
    uart_contexts[index].tx_dma_size = tx_dma_size;
    uart_contexts[index].tx_buffer_size = tx_buffer_size;
    uart_contexts[index].dma_half_complete = 0;
    uart_contexts[index].initialized = 1;

    return index;
}

/**
  * @brief  Find UART context by handle
  */
static UART_Context* GetUartContext(UART_HandleTypeDef *huart) {
    if (huart == NULL) return NULL;

    for (uint8_t i = 0; i < uart_count; i++) {
        if (uart_contexts[i].initialized && uart_contexts[i].huart == huart) {
            return &uart_contexts[i];
        }
    }

    return NULL;
}

/**
  * @brief  Read a single byte from UART
  */
uint8_t SerialRead(UART_HandleTypeDef *huart) {
    UART_Context *ctx = GetUartContext(huart);
    uint8_t data = 0xFF;

    if (ctx != NULL) {
        CircBufToMemcpy(&data, ctx->rx_buf, &ctx->rx_head, 1, ctx->rx_dma_size);
    }

    return data;
}

/**
  * @brief  Read multiple bytes from UART
  */
void SerialReadBuf(UART_HandleTypeDef *huart, uint8_t* Buffer, uint16_t ByteCount) {
    UART_Context *ctx = GetUartContext(huart);

    if (ctx != NULL && Buffer != NULL) {
        /* Prevent reading beyond available data */
        uint16_t available = SerialAvailable(huart);
        uint16_t count = (ByteCount > available) ? available : ByteCount;

        CircBufToMemcpy(Buffer, ctx->rx_buf, &ctx->rx_head, count, ctx->rx_dma_size);
    }
}

/**
  * @brief  Flush UART receive buffer
  */
void SerialReadFlush(UART_HandleTypeDef *huart) {
    UART_Context *ctx = GetUartContext(huart);

    if (ctx != NULL) {
        ctx->rx_head = huart->RxXferSize - huart->hdmarx->Instance->CNDTR;
    }
}

/**
  * @brief  Get number of bytes available for reading
  */
uint16_t SerialAvailable(UART_HandleTypeDef *huart) {
    UART_Context *ctx = GetUartContext(huart);

    if (ctx != NULL) {
        uint16_t dma_tail = huart->RxXferSize - huart->hdmarx->Instance->CNDTR;
        return dma_tail >= ctx->rx_head ?
               (dma_tail - ctx->rx_head) :
               (ctx->rx_dma_size - ctx->rx_head + dma_tail);
    }

    return 0xFFFF;  /* Error indicator */
}

/**
  * @brief  Write a single byte to UART
  */
void SerialWrite(UART_HandleTypeDef *huart, uint8_t Data) {
    UART_Context *ctx = GetUartContext(huart);

    if (ctx != NULL) {
        /* Check if UART is busy or if buffer is not empty */
        if((ctx->tx_head != ctx->tx_tail) || (huart->gState != HAL_UART_STATE_READY)) {
            /* Just enqueue the byte */
            MemToCircBufcpy(ctx->data_buffer, &Data, &ctx->tx_head, 1, ctx->tx_buffer_size);
        } else {
            /* Buffer is empty and UART is ready, enqueue and start transmission */
            MemToCircBufcpy(ctx->data_buffer, &Data, &ctx->tx_head, 1, ctx->tx_buffer_size);
            CircBufToMemcpy(ctx->tx_buf, ctx->data_buffer, &ctx->tx_tail, 1, ctx->tx_buffer_size);
            HAL_UART_Transmit_DMA(huart, (uint8_t *)ctx->tx_buf, 1);
        }
    }
}

/**
  * @brief  Write multiple bytes to UART
  */
void SerialWriteBuf(UART_HandleTypeDef *huart, uint8_t* Data, uint16_t ByteCount) {
    UART_Context *ctx = GetUartContext(huart);

    if (ctx != NULL && Data != NULL && ByteCount > 0) {
        /* Always enqueue data in circular buffer */
        MemToCircBufcpy(ctx->data_buffer, Data, &ctx->tx_head, ByteCount, ctx->tx_buffer_size);

        /* If UART is ready, start transmission */
        if(huart->gState == HAL_UART_STATE_READY) {
            uint16_t txSize = (ByteCount < ctx->tx_dma_size) ? ByteCount : ctx->tx_dma_size;
            CircBufToMemcpy(ctx->tx_buf, ctx->data_buffer, &ctx->tx_tail, txSize, ctx->tx_buffer_size);
            HAL_UART_Transmit_DMA(huart, (uint8_t *)ctx->tx_buf, txSize);
        }
    }
}

/**
  * @brief  Printf-style formatted output to UART
  */
void SerialPrintf(UART_HandleTypeDef *huart, const char* fmt, ...) {
    uint8_t formattedString[UART_PRINTF_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    int16_t sendLength = vsnprintf((char *)formattedString, sizeof(formattedString), fmt, args);
    va_end(args);

    if (sendLength > 0) {
        SerialWriteBuf(huart, formattedString, sendLength);
    }
}

/**
  * @brief  Handle UART TX DMA complete interrupt
  * @note   This function handles both normal and circular DMA modes with optimized buffer management.
  *         It automatically detects the DMA mode and applies the appropriate strategy.
  *         For circular DMA: Optimizes by updating buffer sections when possible
  *         For normal DMA: Always restarts transfers as required by the hardware
  * @param  huart: UART handle containing DMA information
  * @retval None
  */
void Handle_Uart_TX_DMA_Complete(UART_HandleTypeDef *huart) {
    UART_Context *ctx = GetUartContext(huart);
    if (ctx == NULL) return;

    /* Calculate remaining bytes to send */
    uint16_t remainingBytes = CircBufRemainingByte(&ctx->tx_head, &ctx->tx_tail, ctx->tx_buffer_size);

    /* If nothing more to send, just abort and exit */
    if (remainingBytes == 0) {
        /* Only abort for circular mode, not needed for normal mode */
        if (huart->hdmatx->Init.Mode == DMA_CIRCULAR) {
            HAL_UART_AbortTransmit_IT(huart);
        }
        ctx->dma_half_complete = 0;
        return;
    }

    /* For normal DMA mode, immediately initiate next dma trasnfer, then exit */
    if (huart->hdmatx->Init.Mode != DMA_CIRCULAR)
    {  // Normal DMA mode (always takes this path)
        /* Determine optimal transfer size */
        uint16_t transferSize = (remainingBytes < ctx->tx_dma_size) ? remainingBytes : ctx->tx_dma_size;

        /* Fill buffer with new data and start transmission */
		CircBufToMemcpy(ctx->tx_buf, ctx->data_buffer, &ctx->tx_tail,
					   transferSize, ctx->tx_buffer_size);
		HAL_UART_Transmit_DMA(huart, (uint8_t *)ctx->tx_buf, transferSize);

        /* Reset the half-complete flag */
        ctx->dma_half_complete = 0;
        return;
    }

    /* Decide on the next transfer approach */
    /* Condition : We have more data than a full DMA buffer AND previous transfer was full-sized
       This allows continuous streaming by just updating the second half of the buffer
       while the first half is still being transmitted by DMA hardware.
       No half-complete flag needed for this. */
    if (remainingBytes > ctx->tx_dma_size && huart->TxXferSize == ctx->tx_dma_size)
    {
        CircBufToMemcpy(ctx->tx_buf + (ctx->tx_dma_size >> 1),
                       ctx->data_buffer, &ctx->tx_tail,
                       ctx->tx_dma_size >> 1, ctx->tx_buffer_size);
    }
    /* Condition : Medium-sized data transfer with half-complete interrupt processed
       We have between half and full buffer of data remaining AND the half-complete interrupt
       has been handled (first half already updated). Just need to update the second half of
       the buffer to complete this transfer. */
    else if (remainingBytes > (ctx->tx_dma_size >> 1) && remainingBytes <= ctx->tx_dma_size &&  ctx->dma_half_complete) {
        CircBufToMemcpy(ctx->tx_buf + (ctx->tx_dma_size >> 1),
                       ctx->data_buffer, &ctx->tx_tail,
                       ctx->tx_dma_size >> 1, ctx->tx_buffer_size);
    }
    /* Condition 1: More data than buffer size but previous transfer wasn't full-sized
       Condition 2: Medium-sized data (0.5-1× buffer) but half-complete flag not set
       Condition 3: Small amount of data (less than half buffer) remaining
       In all these cases, we need to abort current transfer and start a new one */
    else
    {
        HAL_UART_AbortTransmit_IT(huart);

        /* Determine optimal transfer size:
           - For large amounts of data, use full buffer size
           - For smaller amounts, just transfer what's left */
        uint16_t transferSize = (remainingBytes < ctx->tx_dma_size) ?
                               remainingBytes : ctx->tx_dma_size;

        /* Fill buffer with new data and start transmission */
        CircBufToMemcpy(ctx->tx_buf, ctx->data_buffer, &ctx->tx_tail,
                       transferSize, ctx->tx_buffer_size);
        HAL_UART_Transmit_DMA(huart, (uint8_t *)ctx->tx_buf, transferSize);
    }

    /* Reset the half-complete flag for next transfer cycle
       This ensures proper tracking for the next set of transfers */
    ctx->dma_half_complete = 0;
}

/**
  * @brief  Handle UART TX DMA half-complete interrupt
  */
void Handle_Uart_TX_DMA_HalfComplete(UART_HandleTypeDef *huart) {
    UART_Context *ctx = GetUartContext(huart);
    if (ctx == NULL) return;

    /* Only process for full DMA transfers */
    if (huart->TxXferSize < ctx->tx_dma_size) return;
    if (huart->hdmatx->Init.Mode != DMA_CIRCULAR) return;

    /* If we have more data to send than the DMA buffer size, update first half */
    if (CircBufRemainingByte(&ctx->tx_head, &ctx->tx_tail, ctx->tx_buffer_size) > ctx->tx_dma_size) {
        CircBufToMemcpy(ctx->tx_buf, ctx->data_buffer, &ctx->tx_tail,
                       ctx->tx_dma_size >> 1, ctx->tx_buffer_size);
        ctx->dma_half_complete = 1;  /* Signal to the complete handler */
    }
}

/* USB CDC Functions ---------------------------------------------------------*/

/**
  * @brief  Initialize USB CDC functionality
  */
void USB_CDC_Init(USBD_HandleTypeDef *usb_device_handle,
                 uint8_t *tx_buffer, uint16_t tx_size,
                 uint8_t *rx_buffer, uint16_t rx_size,
                 uint8_t *tx_packet_buffer) {
    if (usb_device_handle == NULL || tx_buffer == NULL ||
        rx_buffer == NULL || tx_packet_buffer == NULL) {
        return;
    }

    usb_device = usb_device_handle;
    usb_tx_data_buffer = tx_buffer;
    usb_rx_buffer = rx_buffer;
    usb_tx_buffer = tx_packet_buffer;
    usb_tx_head = 0;
    usb_tx_tail = 0;
    usb_rx_head = 0;
    usb_rx_tail = 0;
    usb_tx_buffer_size = tx_size;
    usb_rx_buffer_size = rx_size;
    usb_initialized = 1;

    USB_Connected = 0;
    SystemFlags.USB_Print_Busy = 0;
}

/**
  * @brief  Set USB CDC configuration state
  */
void USB_CDC_SetConfiguration(uint8_t configured) {
    if (configured) {
        USB_Connected = USB_TIMEOUT_COUNT; /* Initialize timeout counter */
    } else {
        USB_Connected = 0;
        SystemFlags.USB_Print_Busy = 0;
    }
}

/**
  * @brief  Get number of bytes available for reading from USB CDC
  */
uint16_t USBSerialAvailable(void) {
    if (!usb_initialized) return 0;

    return CircBufRemainingByte(&usb_rx_head, &usb_rx_tail, usb_rx_buffer_size);
}

/**
  * @brief  Read a single byte from USB CDC
  */
uint8_t USBSerialRead(void) {
    if (!usb_initialized || usb_rx_head == usb_rx_tail) return 0xFF;

    uint8_t data;
    CircBufToMemcpy(&data, usb_rx_buffer, &usb_rx_tail, 1, usb_rx_buffer_size);

    return data;
}

/**
  * @brief  Read multiple bytes from USB CDC
  */
void USBSerialReadBuf(uint8_t* Buffer, uint16_t ByteCount) {
    if (!usb_initialized || Buffer == NULL) return;

    uint16_t available = USBSerialAvailable();
    uint16_t count = (ByteCount > available) ? available : ByteCount;

    CircBufToMemcpy(Buffer, usb_rx_buffer, &usb_rx_tail, count, usb_rx_buffer_size);
}

/**
  * @brief  Flush USB CDC receive buffer
  */
void USBSerialReadFlush(void) {
    if (!usb_initialized) return;

    usb_rx_tail = usb_rx_head;
}

/**
  * @brief  Write a single byte to USB CDC
  */
void USBSerialWrite(uint8_t Data) {
    if (!usb_initialized || !USB_Connected) return;

    /* Add byte to circular buffer */
    MemToCircBufcpy(usb_tx_data_buffer, &Data, &usb_tx_head, 1, usb_tx_buffer_size);
}

/**
  * @brief  Write multiple bytes to USB CDC with dual-path transmission
  */
void USBSerialWriteBuf(uint8_t *Data, uint16_t ByteCount) {
    if (!usb_initialized || Data == NULL || ByteCount == 0 || !USB_Connected) return;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)usb_device->pClassData;
    if (hcdc == NULL) return;

    /* Check current transmission state */
    if((usb_tx_head != usb_tx_tail) || (hcdc->TxState != 0)) {
        /* Just buffer the data if transmission is ongoing or buffer not empty */
        MemToCircBufcpy(usb_tx_data_buffer, Data, &usb_tx_head, ByteCount, usb_tx_buffer_size);
    } else {
        /* Immediate transmission - set busy flag to prevent timer callback interference */
        SystemFlags.USB_Print_Busy = 1;

        /* Reset USB timeout counter */
        USB_Connected = USB_TIMEOUT_COUNT;

        /* Buffer the data first */
        MemToCircBufcpy(usb_tx_data_buffer, Data, &usb_tx_head, ByteCount, usb_tx_buffer_size);

        /* Determine optimal packet size */
        uint16_t txSize = (ByteCount < USB_CDC_TX_PACKET_SIZE) ? ByteCount : USB_CDC_TX_PACKET_SIZE;

        /* Copy data to transmission buffer */
        CircBufToMemcpy(usb_tx_buffer, usb_tx_data_buffer, &usb_tx_tail, txSize, usb_tx_buffer_size);

        /* Start transmission */
        extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
        CDC_Transmit_FS(usb_tx_buffer, txSize);

        /* Release busy flag */
        SystemFlags.USB_Print_Busy = 0;
    }
}

/**
  * @brief  Printf-style formatted output to USB CDC
  */
void USBSerialPrintf(const char* fmt, ...) {
    if (!usb_initialized || !USB_Connected) return;

    uint8_t formattedString[UART_PRINTF_BUFFER_SIZE];
    va_list args;
    va_start(args, fmt);
    int16_t sendLength = vsnprintf((char *)formattedString, sizeof(formattedString), fmt, args);
    va_end(args);

    if (sendLength > 0) {
        USBSerialWriteBuf(formattedString, sendLength);
    }
}

/**
  * @brief  Process USB CDC transmissions (call from timer OC interrupt)
  */
void USB_CDC_TimerCallback(void) {
    if (!usb_initialized) return;

    /* Get CDC handle to check internal state */
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)usb_device->pClassData;
    if (hcdc == NULL) {
        USB_Connected = 0;
        SystemFlags.USB_Print_Busy = 0;
        return;
    }

    /* Check if there's pending data and no active transmission */
    uint16_t remainingBytes = CircBufRemainingByte(&usb_tx_head, &usb_tx_tail, usb_tx_buffer_size);

    if (remainingBytes && (hcdc->TxState == 0) && (!SystemFlags.USB_Print_Busy)) {
        /* Reset USB timeout counter when transmission is possible */
        USB_Connected = USB_TIMEOUT_COUNT;

        /* Determine packet size */
        uint16_t txSize = (remainingBytes < USB_CDC_TX_PACKET_SIZE) ?
                          remainingBytes : USB_CDC_TX_PACKET_SIZE;

        /* Copy data to transmission buffer */
        CircBufToMemcpy(usb_tx_buffer, usb_tx_data_buffer, &usb_tx_tail, txSize, usb_tx_buffer_size);

        /* Start transmission */
        extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
        CDC_Transmit_FS(usb_tx_buffer, txSize);
    } else {
        /* Update connection timeout counter if transmission is active */
        if (hcdc->TxState == 1) {
            /* Decrement counter if it's greater than 0 */
            USB_Connected = USB_Connected > 0 ? USB_Connected - 1 : 0;
        }
    }
}

/**
  * @brief  USB CDC receive callback - stores data in circular buffer
  */
void USB_CDC_ReceiveCallback(uint8_t* Buf, uint32_t Len) {
    if (!usb_initialized || Buf == NULL || Len == 0) return;

    /* Reset USB timeout counter on data reception */
    USB_Connected = USB_TIMEOUT_COUNT;

    /* Store received data in circular buffer */
    uint16_t spaceAvailable = usb_rx_buffer_size - CircBufRemainingByte(&usb_rx_head, &usb_rx_tail, usb_rx_buffer_size);
    uint16_t bytesToStore = (Len > spaceAvailable) ? spaceAvailable : Len;

    if (bytesToStore > 0) {
        MemToCircBufcpy(usb_rx_buffer, Buf, &usb_rx_head, bytesToStore, usb_rx_buffer_size);
    }
}

/* Utility Functions ---------------------------------------------------------*/

/**
  * @brief  Calculate remaining bytes in circular buffer
  */
uint16_t CircBufRemainingByte(uint16_t* HeadPointer, uint16_t* TailPointer, uint16_t Buffer_SIZE) {
    return *HeadPointer >= *TailPointer ?
           (*HeadPointer - *TailPointer) :
           (Buffer_SIZE - *TailPointer + *HeadPointer);
}

/**
  * @brief  Copy data from circular buffer to memory
  */
void CircBufToMemcpy(uint8_t* MemoryToWrite, uint8_t* CircularBufferToCopy,
                    uint16_t* Pointer, uint16_t ByteCount, uint16_t BufferSize) {
    if (ByteCount == 0) return;

    if ((*Pointer + ByteCount) <= BufferSize) {
        /* Simple case: no buffer wraparound */
        memcpy(MemoryToWrite, &CircularBufferToCopy[*Pointer], ByteCount);
        *Pointer = *Pointer + ByteCount;
    } else {
        /* Buffer wraparound case */
        uint16_t firstChunk = BufferSize - *Pointer;
        memcpy(MemoryToWrite, &CircularBufferToCopy[*Pointer], firstChunk);
        memcpy(MemoryToWrite + firstChunk, &CircularBufferToCopy[0], ByteCount - firstChunk);
        *Pointer = ByteCount - firstChunk;
    }
}

/**
  * @brief  Copy data from memory to circular buffer
  */
void MemToCircBufcpy(uint8_t* CircularBufferToWrite, uint8_t* MemoryToCopy,
                    uint16_t* Pointer, uint16_t ByteCount, uint16_t BufferSize) {
    if (ByteCount == 0) return;

    if ((*Pointer + ByteCount) <= BufferSize) {
        /* Simple case: no buffer wraparound */
        memcpy(&CircularBufferToWrite[*Pointer], MemoryToCopy, ByteCount);
        *Pointer = *Pointer + ByteCount;
    } else {
        /* Buffer wraparound case */
        uint16_t firstChunk = BufferSize - *Pointer;
        memcpy(&CircularBufferToWrite[*Pointer], MemoryToCopy, firstChunk);
        memcpy(&CircularBufferToWrite[0], MemoryToCopy + firstChunk, ByteCount - firstChunk);
        *Pointer = ByteCount - firstChunk;
    }
}


/**
 * @brief  Initialize timing constants to reduce computation time
 * @note   Call once during system initialization for best performance
 */
void BasicTools_InitTiming(void)
{
	MicrosMax = (SysTick->LOAD + 1);
    MicrosDivisor = MicrosMax/1000;
}

/**
 * @brief  Returns the number of milliseconds since the system started
 * @retval Millisecond count
 */
uint32_t millis(void)
{
    return HAL_GetTick();
}

/**
 * @brief  Returns the number of microseconds since the system started
 * @note   Uses SysTick for sub-millisecond precision with optimized calculation
 * @retval Microsecond count
 */
uint32_t micros(void)
{
    /* Initialize constants if not already done */
    if (MicrosMax == 0) BasicTools_InitTiming();

    /* Critical section - capture values as atomically as possible */
    uint32_t tick_before = SysTick->VAL;
    uint32_t ms = HAL_GetTick();
    uint32_t tick_after = SysTick->VAL;

    /* Calculate microseconds with pre-computed divisor for speed */
    uint32_t us = (MicrosMax - tick_after) / MicrosDivisor;

    /* Detect millisecond boundary crossing during measurement */
    return (tick_before >= tick_after) ? (ms * 1000 + us) : (HAL_GetTick() * 1000 + us);
}
