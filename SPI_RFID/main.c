#include "stm32f10x.h"                 
#include "stm32f10x_i2c.h"              
#include "stm32f10x_usart.h"  
#include "stm32f10x_rcc.h"              // Keil::Device:StdPeriph Drivers:RCC
#include "stm32f10x_tim.h"              // Keil::Device:StdPeriph Drivers:TIM
#include "stm32f10x_gpio.h" 
#include "string.h"
#include <stdio.h>                  
#include "misc.h"  

void Timer2_Init(void); // timer2 tao tre
void USART1_Config(void);  // cau hinh uart
void RFID_Init(void); // khoi dong RFID
void SPI_Config(void); // cau hinh spi 
void USART_SendString(USART_TypeDef* USARTx, const char* str); //gui chuoi
void RFID_ReadID(void); // Ðoc ID tu RFID
void RFID_WriteReg(uint8_t reg, uint8_t value);
void Delay(uint32_t delay);
uint8_t SPI_SendByte(uint8_t byte);
uint8_t RFID_ReadReg(uint8_t reg);


uint8_t SPI_Transfer(uint8_t data); // G?i và nhan du lieu qua SPI

#define RFID_CS_LOW() GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define RFID_CS_HIGH() GPIO_SetBits(GPIOB, GPIO_Pin_12)

void USART1_Config(void) {
    
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    // clock cho USART1 và GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // cau  hình chân PA9 và PA10 cho USART
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; // TX
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate Function Push-Pull
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10; // RX
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING; // Input floating
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Cau  hình USART1
    USART_InitStruct.USART_BaudRate = 9600;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &USART_InitStruct);

    // Kick  USART1
    USART_Cmd(USART1, ENABLE);
}

void USART_SendString(USART_TypeDef* USARTx, const char* str) {
    while (*str) {
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
        USART_SendData(USARTx, *str++);
    }
}
void SPI_Config(void)
{
    GPIO_InitTypeDef gpioInit;
    SPI_InitTypeDef SPI_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // C?u hình chân SPI2: SCK (PB13), MOSI (PB15), MISO (PB14)
    gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpioInit);

    gpioInit.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    gpioInit.GPIO_Pin = GPIO_Pin_14;
    GPIO_Init(GPIOB, &gpioInit);

    // C?u hình SPI2
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI2, &SPI_InitStructure);

    SPI_Cmd(SPI2, ENABLE);
}
uint8_t SPI_SendByte(uint8_t byte)
{
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);  // Ch? TX tr?ng
    SPI_I2S_SendData(SPI2, byte);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); // Ch? RX d?y
    return SPI_I2S_ReceiveData(SPI2);  // Nhan du lieu tu MISO
}
uint8_t RFID_ReadReg(uint8_t reg)
{
    uint8_t result;
    RFID_CS_LOW();
    SPI_SendByte(((reg << 1) & 0x7E) | 0x80); // Ð?a ch? thanh ghi
    result = SPI_SendByte(0x00);  // Ð?c d? li?u
    RFID_CS_HIGH();
    return result;
}
void RFID_WriteReg(uint8_t reg, uint8_t value)
{
    RFID_CS_LOW();
    SPI_SendByte((reg << 1) & 0x7E);  // Ð?a ch? thanh ghi (bit 7 = 0 d? ghi)
    SPI_SendByte(value);  // Ghi d? li?u
    RFID_CS_HIGH();
}
void RFID_Init(void) {
    // Reset RFID
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    // C?u hình chân CS
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12; // CS
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; // Ch? d? d?u ra Push-Pull
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Ð?t chân CS v? tr?ng thái cao
    RFID_CS_HIGH();

    // Reset module RFID
    RFID_WriteReg(0x01, 0x0F); // Ví d?: G?i l?nh reset (c?n di?u ch?nh theo datasheet c?a module)
    Delay(100); // Ð?i 100ms
}

void RFID_ReadID(void) {
    uint8_t id[5]; // Gi? s? ID có d? dài 5 byte
    char id_str[30];

    // Kích ho?t RFID
    RFID_CS_LOW();

    // G?i l?nh d? d?c ID (các l?nh c? th? s? ph? thu?c vào module b?n s? d?ng)
    // Ví d?: G?i l?nh d?c ID
    for (int i = 0; i < 5; i++) {
        id[i] = SPI_SendByte(0x00); // G?i l?nh và nh?n d? li?u
    }

    // Ng?t k?t n?i RFID
    RFID_CS_HIGH();

    // Chuy?n d?i ID thành chu?i và g?i qua UART
    sprintf(id_str, "ID: %02X %02X %02X %02X %02X\r\n", id[0], id[1], id[2], id[3], id[4]);
    USART_SendString(USART1, id_str);
}

void Delay(uint32_t delay) {
    for (uint32_t i = 0; i < delay * 1000; i++) {
        __NOP(); // Không làm gì, ch? d? t?o d? tr?
    }
}

int main(void) {
    // Kh?i t?o các thành ph?n
    USART1_Config(); // C?u hình UART
    SPI_Config(); // C?u hình SPI
    RFID_Init(); // Kh?i d?ng RFID

    while (1) {
        RFID_ReadID(); // Ð?c ID t? RFID
        Delay(1000); // Ð?c ID m?i giây
    }
}
	