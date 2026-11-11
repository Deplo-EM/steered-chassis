#pragma once
#include <cstdint>
#define HAL_SPI_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define IS_SPI_ALL_INSTANCE(x) true
#define IS_GPIO_ALL_INSTANCE(x) true
#define IS_GPIO_PIN(x) true
struct SPI_HandleTypeDef { void* Instance=nullptr; };
struct GPIO_TypeDef {};
enum HAL_StatusTypeDef { HAL_OK=0, HAL_ERROR=1, HAL_BUSY=2, HAL_TIMEOUT=3 };
enum GPIO_PinState { GPIO_PIN_RESET=0, GPIO_PIN_SET=1 };
constexpr uint32_t GPIO_PIN_0=1;
inline void __disable_irq() {}
inline void __enable_irq() {}
void HAL_GPIO_WritePin(GPIO_TypeDef*,uint16_t,GPIO_PinState);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef*,uint8_t*,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef*,uint8_t*,uint8_t*,uint16_t,uint32_t);
