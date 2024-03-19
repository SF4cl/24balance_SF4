
#ifndef BALANCE_CHASSIS_BSP_SPI_H
#define BALANCE_CHASSIS_BSP_SPI_H

#include "struct_typedef.h"

extern void spi1DMAInit(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
extern void spi1DMAEnable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

#endif
