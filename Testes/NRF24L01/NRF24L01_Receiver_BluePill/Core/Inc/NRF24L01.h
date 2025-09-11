/*
 * NRF24L01.h
 *
 *  Created on: Aug 28, 2025
 *      Author: Kauê Lucas
 */

#ifndef INC_NRF24L01_H_
#define INC_NRF24L01_H_

#include "stm32f1xx_hal.h" // ajuste conforme sua MCU

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *CE_port;
    uint16_t CE_pin;
    GPIO_TypeDef *CSN_port;
    uint16_t CSN_pin;
} NRF24_HandleTypeDef;

// ==== Funções ====
void NRF24_Init_TX(NRF24_HandleTypeDef *nrf);
void NRF24_Init_RX(NRF24_HandleTypeDef *nrf);
uint8_t NRF24_Send(NRF24_HandleTypeDef *nrf, uint8_t *data, uint8_t len);
uint8_t NRF24_Receive(NRF24_HandleTypeDef *nrf, uint8_t *data, uint8_t len);
void NRF24_DebugLED(GPIO_TypeDef *LED_port, uint16_t LED_pin, uint8_t status);


#endif /* INC_NRF24L01_H_ */
