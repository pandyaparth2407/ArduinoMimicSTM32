# ArduinoMimicSTM32
## BasicToolsDemoCode
A lightweight, Arduino-inspired utility library for STM32 microcontrollers with professional-grade timing precision and communication capabilities.  

## Description  

This repository demonstrates a production-ready implementation of BasicTools, an embedded utility library that bridges the gap between Arduino simplicity and professional embedded development. The code showcases a robust approach to common microcontroller tasks with optimized performance and low overhead.  

## Key Features  

- **High-precision timing** - Microsecond-accurate time measurement using optimized SysTick handling with pre-computed divisors  
- **Dual-protocol communication** - Seamless data exchange between USB CDC and UART interfaces with consistent API  
- **Intelligent buffer management** - Implements receive timeout detection for complete message handling  
- **HAL integration** - Works alongside STM32 HAL without modification while extending functionality  
- **DMA-accelerated transfers** - Leverages hardware DMA for efficient data movement with proper state management  

## Implementation Notes  

The demo implements a bidirectional communication gateway between USB CDC and two UART ports, demonstrating timing-based packet detection and throughput optimization. The `micros()` implementation uses SysTick counter value manipulation to achieve microsecond precision without compromising MCU performance.  

## Setup Requirements  
### Basic IOC Setting 
- in IOC, Pinout & Configuration -> System Core -> SYS-> Debug -> (Serial Wire) to allow stlink connection
- in IOC, Clock Configuration -> Must adjust until get (48) To USB (MHz)

### Required Files to Modify  

1. **main.c** - Include library headers and add initialization code  
2. **usbd_cdc_if.c** - Include library headers and Modify CDC receive callback to integrate with library buffer management  

### Interrupt Configuration  

The following interrupts **must** be enabled in your project:  
- USART1_IRQn  
- USART2_IRQn  
- DMA1_Channel4_IRQn (UART1 TX)  
- DMA1_Channel5_IRQn (UART1 RX)  
- DMA1_Channel6_IRQn (UART2 RX)  
- DMA1_Channel7_IRQn (UART2 TX) 
- TIM1_UP_IRQn (Timer1 Update interrupt)  
- TIM1_CC_IRQn (Timer1 Capture Compare interrupt)   
- SysTick_IRQn (already enabled by default in HAL)  

### DMA Configuration  

Configure the following DMA channels in circular mode:   
- DMA1_Channel4 (UART1 TX) - Memory increment, circular mode, 8-bit size  
- DMA1_Channel5 (UART1 RX) - Memory increment, circular mode, 8-bit size 
- DMA1_Channel6 (UART2 RX) - Memory increment, circular mode, 8-bit size  
- DMA1_Channel7 (UART2 TX) - Memory increment, circular mode, 8-bit size  

### Critical Settings  

1. In CubeMX GUI or `.ioc` file:  
   - Set both UARTs to Asynchronous mode  
   - Enable global interrupt for both UARTs  
   - Enable DMA for both TX and RX on both UARTs  
   - Configure USB Device CDC class if using USB communication  

2. In project settings:  
   - **IMPORTANT:** Untick "Exclude BasicTools folder from build" in project properties  
   - Tick the Box for float with printf from newlib-nano (Properties -> MCU Settings )  
   - Add BasicTools directory to include paths in project properties (Properties -> C/C++ General -> Paths and Symbol -> Includes tab)  

## Target Applications  

- Data acquisition systems requiring precise timestamping  
- Protocol bridges and converters  
- Diagnostic tools and test equipment  
- Educational platforms for embedded systems

## License  

This project is licensed under the MIT License - see the LICENSE file for details.

## Usage Example  

```c  
#include "BasicTools.h"  

// UART1 buffers  
uint8_t uart1_rx_buf[UART1_RX_DMA_SIZE];  
uint8_t uart1_tx_buf[UART1_TX_DMA_SIZE];  
uint8_t uart1_data_buffer[UART1_TX_Buffer_SIZE];  

// USB CDC  
//    Extern needed  
extern USBD_HandleTypeDef hUsbDeviceFS;  
//    Buffers needed  
uint8_t usb_cdc_tx_data[USB_CDC_BUFFER_SIZE];           // Circular buffer for TX data  
uint8_t usb_cdc_rx_buffer[USB_CDC_BUFFER_SIZE];         // Buffer for RX data  
uint8_t usb_cdc_tx_buffer[USB_CDC_TX_PACKET_SIZE];      // Buffer for USB packet transmission  

// Initialize the library  
// UART1 Setup  
  UART_Register(&huart1,  
               uart1_rx_buf, UART1_RX_DMA_SIZE,  
               uart1_tx_buf, uart1_data_buffer,  
               UART1_TX_DMA_SIZE, UART1_TX_Buffer_SIZE);  
  HAL_UART_Receive_DMA(&huart1, uart1_rx_buf, UART1_RX_DMA_SIZE);	// Once started, will keep listening into buffer  
// USB CDC UART Setup  
  USB_CDC_Init(&hUsbDeviceFS, usb_cdc_tx_data, USB_CDC_BUFFER_SIZE,  
              usb_cdc_rx_buffer, USB_CDC_BUFFER_SIZE, usb_cdc_tx_buffer);  

// Use Arduino-like functions  
uint32_t start = micros();  
// ... your code here ...  
uint32_t elapsed = micros() - start;  

// Send data through UART1  
UART1_printf(&huart1,"Elapsed time: %lu microseconds\r\n", elapsed);  
if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) USBSerialPrintf("Elapsed time: %lu microseconds\r\n", elapsed);  
// Process in main loop  
while(1) {  
  UART1_printf(&huart1,"Time: %lu ms\r\n", millis());  
  if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)  
  {  
    USBSerialPrintf("Time: %lu ms\r\n", millis());  
  }  
  HAL_Delay(100);  
}  
