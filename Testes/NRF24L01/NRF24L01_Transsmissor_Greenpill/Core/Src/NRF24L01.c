/*
 * NRF24L01.c
 *
 *  Created on: Aug 28, 2025
 *      Author: Kauê Lucas
 */

#include "NRF24L01.h"

// ==== Macros ====
#define CE_HIGH(nrf) HAL_GPIO_WritePin(nrf->CE_port, nrf->CE_pin, GPIO_PIN_SET)
#define CE_LOW(nrf)  HAL_GPIO_WritePin(nrf->CE_port, nrf->CE_pin, GPIO_PIN_RESET)
#define CSN_HIGH(nrf) HAL_GPIO_WritePin(nrf->CSN_port, nrf->CSN_pin, GPIO_PIN_SET)
#define CSN_LOW(nrf)  HAL_GPIO_WritePin(nrf->CSN_port, nrf->CSN_pin, GPIO_PIN_RESET)

// ==== Registros ====
#define NRF24_REG_CONFIG    	0x00
#define NRF24_REG_RF_CH     	0x05
#define NRF24_REG_RF_SETUP  	0x06
#define NRF24_REG_STATUS    	0x07
#define NRF24_REG_TX_ADDR   	0x10
#define NRF24_REG_RX_ADDR_P0 	0x0A
#define NRF24_REG_RX_PW_P0  	0x11
#define NRF24_REG_EN_AA     	0x01
#define NRF24_REG_EN_RXADDR 	0x02
#define NRF24_REG_SETUP_AW  	0x03
#define NRF24_REG_SETUP_RETR 	0x04

// ==== Comandos ====
#define NRF24_CMD_W_REGISTER  	0x20
#define NRF24_CMD_R_REGISTER  	0x00
#define NRF24_CMD_W_TX_PAYLOAD 	0xA0
#define NRF24_CMD_R_RX_PAYLOAD 	0x61
#define NRF24_CMD_FLUSH_TX     	0xE1
#define NRF24_CMD_NOP          	0xFF

// ==== Funções internas ====
static uint8_t NRF24_SPI_RW(NRF24_HandleTypeDef *nrf, uint8_t data) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(nrf->hspi, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

static void NRF24_WriteReg(NRF24_HandleTypeDef *nrf, uint8_t reg, uint8_t value) {
    CSN_LOW(nrf);
    NRF24_SPI_RW(nrf, NRF24_CMD_W_REGISTER | reg);
    NRF24_SPI_RW(nrf, value);
    CSN_HIGH(nrf);
}

static uint8_t NRF24_ReadReg(NRF24_HandleTypeDef *nrf, uint8_t reg) {
    uint8_t value;
    CSN_LOW(nrf);
    NRF24_SPI_RW(nrf, NRF24_CMD_R_REGISTER | reg);
    value = NRF24_SPI_RW(nrf, NRF24_CMD_NOP);
    CSN_HIGH(nrf);
    return value;
}

static void NRF24_WriteAddr(NRF24_HandleTypeDef *nrf, uint8_t reg, uint8_t *addr, uint8_t len) {
    CSN_LOW(nrf);
    NRF24_SPI_RW(nrf, NRF24_CMD_W_REGISTER | reg);
    for(int i=0;i<len;i++) NRF24_SPI_RW(nrf, addr[i]);
    CSN_HIGH(nrf);
}

// ==== Inicializar como TX ====
void NRF24_Init_TX(NRF24_HandleTypeDef *nrf) {
    CE_LOW(nrf);
    HAL_Delay(5);

    NRF24_WriteReg(nrf, NRF24_REG_CONFIG, 0x0E); // PWR_UP=1, PRIM_RX=0
    NRF24_WriteReg(nrf, NRF24_REG_RF_CH, 76);
    NRF24_WriteReg(nrf, NRF24_REG_RF_SETUP, 0x06);
    NRF24_WriteReg(nrf, NRF24_REG_SETUP_AW, 0x03);
    NRF24_WriteReg(nrf, NRF24_REG_EN_AA, 0x01);
    NRF24_WriteReg(nrf, NRF24_REG_EN_RXADDR, 0x01);
    NRF24_WriteReg(nrf, NRF24_REG_SETUP_RETR, 0x3F);

    uint8_t addr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
    NRF24_WriteAddr(nrf, NRF24_REG_TX_ADDR, addr, 5);
    NRF24_WriteAddr(nrf, NRF24_REG_RX_ADDR_P0, addr, 5);
}

// ==== Inicializar como RX ====
void NRF24_Init_RX(NRF24_HandleTypeDef *nrf) {
    CE_LOW(nrf);
    HAL_Delay(5);

    NRF24_WriteReg(nrf, NRF24_REG_CONFIG, 0x0F); // PWR_UP=1, PRIM_RX=1
    NRF24_WriteReg(nrf, NRF24_REG_RF_CH, 76);
    NRF24_WriteReg(nrf, NRF24_REG_RF_SETUP, 0x06);
    NRF24_WriteReg(nrf, NRF24_REG_SETUP_AW, 0x03);
    NRF24_WriteReg(nrf, NRF24_REG_EN_AA, 0x01);
    NRF24_WriteReg(nrf, NRF24_REG_EN_RXADDR, 0x01);
    NRF24_WriteReg(nrf, NRF24_REG_RX_PW_P0, 32);

    uint8_t addr[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
    NRF24_WriteAddr(nrf, NRF24_REG_RX_ADDR_P0, addr, 5);

    CE_HIGH(nrf);
}

// ==== Enviar ====
uint8_t NRF24_Send(NRF24_HandleTypeDef *nrf, uint8_t *data, uint8_t len) {
    CE_LOW(nrf);

    CSN_LOW(nrf);
    NRF24_SPI_RW(nrf, NRF24_CMD_W_TX_PAYLOAD);
    for(int i=0;i<len;i++) NRF24_SPI_RW(nrf, data[i]);
    CSN_HIGH(nrf);

    CE_HIGH(nrf);
    HAL_Delay(1);
    CE_LOW(nrf);

    // Ler STATUS
    uint8_t status = NRF24_ReadReg(nrf, NRF24_REG_STATUS);

    // Verifica se foi enviado com sucesso
    if (status & 0x20) { // TX_DS
        NRF24_WriteReg(nrf, NRF24_REG_STATUS, 0x20); // limpa flag
        return 1; // sucesso
    } else if (status & 0x10) { // MAX_RT
        NRF24_WriteReg(nrf, NRF24_REG_STATUS, 0x10); // limpa flag
        return 0; // falha
    }
    return 0;
}

// ==== Receber ====
uint8_t NRF24_Receive(NRF24_HandleTypeDef *nrf, uint8_t *data, uint8_t len) {
    uint8_t status = NRF24_ReadReg(nrf, NRF24_REG_STATUS);

    if (status & 0x40) { // RX_DR
        CSN_LOW(nrf);
        NRF24_SPI_RW(nrf, NRF24_CMD_R_RX_PAYLOAD);
        for(int i=0;i<len;i++) data[i] = NRF24_SPI_RW(nrf, NRF24_CMD_NOP);
        CSN_HIGH(nrf);

        NRF24_WriteReg(nrf, NRF24_REG_STATUS, 0x40); // limpa flag
        return 1; // sucesso
    }
    return 0; // nada recebido
}

// ==== Debug LED ====
void NRF24_DebugLED(GPIO_TypeDef *LED_port, uint16_t LED_pin, uint8_t status) {
    if (status) {
        // Sucesso -> 1 piscada
        HAL_GPIO_TogglePin(LED_port, LED_pin);
        HAL_Delay(200);
        HAL_GPIO_TogglePin(LED_port, LED_pin);
    } else {
        // Falha -> 3 piscadas rápidas
        for (int i=0; i<3; i++) {
            HAL_GPIO_TogglePin(LED_port, LED_pin);
            HAL_Delay(100);
            HAL_GPIO_TogglePin(LED_port, LED_pin);
            HAL_Delay(100);
        }
    }
}

