#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

volatile uint32_t microros_transport_open_calls = 0;
volatile uint32_t microros_transport_open_failures = 0;
volatile uint32_t microros_transport_last_open_status = 0;
volatile uint32_t microros_transport_write_calls = 0;
volatile uint32_t microros_transport_write_failures = 0;
volatile uint32_t microros_transport_write_success_bytes = 0;
volatile uint32_t microros_transport_read_calls = 0;
volatile uint32_t microros_transport_read_nonzero_calls = 0;
volatile uint32_t microros_transport_read_success_bytes = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    microros_transport_open_calls++;
    HAL_StatusTypeDef ret = HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
    microros_transport_last_open_status = (uint32_t)ret;
    if (ret != HAL_OK) {
        microros_transport_open_failures++;
        return false;
    }
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    (void)err;
    microros_transport_write_calls++;

    HAL_StatusTypeDef ret;
    if (uart->gState == HAL_UART_STATE_READY){
        ret = HAL_UART_Transmit_DMA(uart, (uint8_t *)buf, len);
        while (ret == HAL_OK && uart->gState != HAL_UART_STATE_READY){
            osDelay(1);
        }

        if (ret == HAL_OK) {
            microros_transport_write_success_bytes += (uint32_t)len;
            return len;
        }
        microros_transport_write_failures++;
        return 0;
    }else{
        microros_transport_write_failures++;
        return 0;
    }
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    (void)err;
    microros_transport_read_calls++;

    int ms_used = 0;
    do
    {
        __disable_irq();
        dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
        __enable_irq();
        ms_used++;
        osDelay(portTICK_RATE_MS);
    } while (dma_head == dma_tail && ms_used < timeout);
    
    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)){
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }

    if (wrote > 0) {
        microros_transport_read_nonzero_calls++;
        microros_transport_read_success_bytes += (uint32_t)wrote;
    }
    
    return wrote;
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM
