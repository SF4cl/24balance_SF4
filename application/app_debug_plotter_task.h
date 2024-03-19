
#ifndef BALANCE_CHASSIS_APP_DEBUG_PLOTTER_TASK_H
#define BALANCE_CHASSIS_APP_DEBUG_PLOTTER_TASK_H

#define USE_DEBUG_PLOTTER

#define DEBUG_PLOTTER_TX_BUF_SIZE  (256)
#define DEBUG_PLOTTER_USART_HANDLE (huart6)

typedef enum {
    VARIABLE_TYPE_I8 = 0,
    VARIABLE_TYPE_U8,
    VARIABLE_TYPE_I16,
    VARIABLE_TYPE_U16,
    VARIABLE_TYPE_I32,
    VARIABLE_TYPE_U32,
    VARIABLE_TYPE_I64,
    VARIABLE_TYPE_U64,
    VARIABLE_TYPE_FP32,
    VARIABLE_TYPE_FP64
} VariableType;

typedef struct VariableNode {
    void* variable_ptr;
    VariableType variable_type;
    struct VariableNode* next;
} VariableNode;


void plotterAddVariable(void* variable_ptr, VariableType variable_type);
void plotterRemoveVariable(void* variable_ptr);
void debugPlotterTask(void const *pvParameter);


#endif
