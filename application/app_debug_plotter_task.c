/***
 * @file app_debug_plotter_task.c
 * @brief send variable values to debugger socket(USART6) using DMA
 * @note  written for https://github.com/je00/Vodka
 */

#include "app_debug_plotter_task.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "cmsis_os.h"

#include "struct_typedef.h"


uint8_t plotter_uart_tx_buf[DEBUG_PLOTTER_TX_BUF_SIZE];
VariableNode *variable_list = NULL;


void plotterAddVariable(void* variable_ptr, VariableType variable_type) {
    VariableNode *new_node = (VariableNode*)malloc(sizeof(VariableNode));
    new_node->variable_ptr = variable_ptr;
    new_node->variable_type = variable_type;
    new_node->next = variable_list;
    variable_list = new_node;
}


void plotterRemoveVariable(void* variable_ptr) {
    VariableNode **current_node = &variable_list;
    while (*current_node) {
        if ((*current_node)->variable_ptr == variable_ptr) {
            VariableNode* nextNode = (*current_node)->next;
            free(*current_node);
            *current_node = nextNode;
        } else {
            current_node = &(*current_node)->next;
        }
    }
}


/***
 * @brief generate format string to send
 * @note  THIS FUNCTION SHOULD ONLY BE CALLED WHEN VARIABLE LIST IS NOT EMPTY
 * @note  format: for example "%d,%u,%f\r\n" for 3 channels of a plot
 * @note  one data frame end with "\r\n"
 * @note  each channel is separated by ','
 */
static void generateFormatString(uint8_t *dst, VariableNode *variable_list_head) {
    char *buf_ptr = (char *)dst;
    size_t remaining_size = DEBUG_PLOTTER_TX_BUF_SIZE;
    int16_t written = 0;

    for (VariableNode* node = variable_list_head; node != NULL; node = node->next) {
        written = 0;

        switch (node->variable_type) {
        case VARIABLE_TYPE_I8:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%d" : "%d,"), *(int8_t*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_U8:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%u" : "%u,"), *(uint8_t*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_I16:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%d" : "%d,"), *(int16_t*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_U16:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%u" : "%u,"), *(uint16_t*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_I32:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%ld" : "%ld,"), *(int32_t*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_U32:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%lu" : "%lu,"), *(uint32_t*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_I64:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%lld" : "%lld,"), *(int64_t*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_U64:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%llu" : "%llu,"), *(uint64_t*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_FP32:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%.7f" : "%.7f,"), *(fp32*)(node->variable_ptr));
            break;
        case VARIABLE_TYPE_FP64:
            written = snprintf(buf_ptr, remaining_size, ((node->next == NULL) ? "%.7f" : "%.7f,"), *(fp64*)(node->variable_ptr));
            break;
        default:
            break;
        }

        // update buffer pointer and remaining size
        if (written >= 0 && written < remaining_size) {
            buf_ptr += written;
            remaining_size -= written;
        } else {
            // buffer overflow
            break;
        }
    }

    written = snprintf(buf_ptr, remaining_size, "\r\n");    // End the data frame with "\r\n"
    if (written >= 0 && written < remaining_size) {
        buf_ptr += written;
        remaining_size -= written;
    } else {
        // buffer overflow
        return;
    }
}


void debugPlotterTask(void const *pvParameter) {
    memset(plotter_uart_tx_buf, 0, DEBUG_PLOTTER_TX_BUF_SIZE);   // clear tx buffer
    // start DMA transmission
    // DMA should be set to circular mode, so that there's no need to restart it on every iteration
    

    while (1) {
        osDelay(25 / portTICK_PERIOD_MS);

        // if variable list is empty, skip this iteration
        if (variable_list == NULL) {
            continue;
        }

        while (HAL_UART_GetState(&DEBUG_PLOTTER_USART_HANDLE) != HAL_UART_STATE_READY) {
            continue;   // wait for DMA to be ready
        }

        memset(plotter_uart_tx_buf, 0, DEBUG_PLOTTER_TX_BUF_SIZE);   // clear tx buffer
        generateFormatString(plotter_uart_tx_buf, variable_list);
        HAL_UART_Transmit_DMA(&DEBUG_PLOTTER_USART_HANDLE, plotter_uart_tx_buf, strlen((char *)plotter_uart_tx_buf));
    }
}
