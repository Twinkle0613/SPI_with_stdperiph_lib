#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_spi.h"
#include <stdint.h>


#define Input_Mode 0x0

#define  idle0_1Edge  0
#define  idle0_2Edge  1
#define  idle1_1Edge  2
#define  idle1_2Edge  3


#define SPI_Enable ((uint16_t)0x0040)
#define SPI_Disable ((uint16_t)0xFFBF)

#define SPI_RXonlyMode_Enable ((uint16_t)0x0400)
#define SPI_RXonlyMode_Disable ((uint16_t)0xFBFF)

void GPIO_conf(GPIO_TypeDef* GPIOx, uint16_t Pin, int Speed, int Mode){
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = Pin;
    gpio.GPIO_Speed = Speed;
    gpio.GPIO_Mode = Mode;
    GPIO_Init(GPIOx,&gpio);
}

void rxOnlyMode(SPI_TypeDef* SPIx, FunctionalState NewState){
  if (NewState != DISABLE)
  {
    SPIx->CR1 |= SPI_RXonlyMode_Enable;
  }
  else
  {
    SPIx->CR1 &= SPI_RXonlyMode_Disable;
  }
}

void enableSPI(SPI_TypeDef* SPIx,FunctionalState NewState){
  if (NewState != DISABLE)
  {
    SPIx->CR1 |= SPI_Enable;
  }
  else
  {
    SPIx->CR1 &= SPI_Disable;
  }
}

void SPI_conf(SPI_TypeDef* SPIx,
              uint16_t Mode,
              uint16_t DataSize,
              uint16_t BaudRatePrescaler,
              uint16_t FirstBit,
              uint16_t spi_clock
              ){
                
  SPI_InitTypeDef spi;
  spi.SPI_Mode = Mode;
  spi.SPI_DataSize = DataSize;
  spi.SPI_BaudRatePrescaler = BaudRatePrescaler;
  spi.SPI_FirstBit = FirstBit;
  switch(spi_clock){
    case 0:
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    break;
    case 1:
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    break;
    case 2:
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    break;
    case 3:
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    break;
    default:break;
  }

  spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi.SPI_NSS = SPI_NSS_Hard;
  spi.SPI_CRCPolynomial = 7;
  SPI_Init(SPIx,&spi);
}
int main(void)
{  

  /*******************GPIO configuration********************/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA,DISABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB,DISABLE);
  /*****Master Mode*****/
  GPIO_conf(GPIOA,GPIO_Pin_4,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);   //SPI1_NSS
  GPIO_conf(GPIOA,GPIO_Pin_5,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);   //SPI1_SCK
  GPIO_conf(GPIOA,GPIO_Pin_6,Input_Mode,GPIO_Mode_IPU);           //SPI1_MISO
  GPIO_conf(GPIOA,GPIO_Pin_7,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);   //SPI1_MOSI
  /*****Slave Mode******/
  GPIO_conf(GPIOB,GPIO_Pin_12,Input_Mode,GPIO_Mode_IN_FLOATING);   //SPI3_NSS
  GPIO_conf(GPIOB,GPIO_Pin_13,Input_Mode,GPIO_Mode_IN_FLOATING);   //SPI3_SCK
  GPIO_conf(GPIOB,GPIO_Pin_14,GPIO_Speed_50MHz,GPIO_Mode_AF_PP);   //SPI3_MISO
  GPIO_conf(GPIOB,GPIO_Pin_15,Input_Mode,GPIO_Mode_IN_FLOATING);   //SPI3_MOSI
  
  //uint32_t checkGPIOA_CRH = GPIOA->CRH;
  //uint32_t checkGPIOA_CRL = GPIOA->CRL;
  //uint32_t checkGPIOB_CRL = GPIOB->CRL;
 /*******************SPI configuration********************/
 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);
  SPI_conf(SPI1,SPI_Mode_Master,SPI_DataSize_8b,SPI_BaudRatePrescaler_4,SPI_FirstBit_LSB,idle0_1Edge);
  SPI_conf(SPI2,SPI_Mode_Slave,SPI_DataSize_8b,SPI_BaudRatePrescaler_2,SPI_FirstBit_LSB,idle0_1Edge);
  SPI_SSOutputCmd(SPI1,ENABLE);
  


  enableSPI(SPI1,ENABLE);
  enableSPI(SPI2,ENABLE);
  
 // uint32_t checkSPI1_CR1 = SPI1->CR1;
 // uint32_t checkSPI1_CR2 = SPI1->CR2;
 // uint32_t checkSPI1_CRCPR = SPI1->CRCPR;
 
  //uint32_t checkSPI2_CR1 = SPI2->CR1;
  //uint32_t checkSPI2_CR2 = SPI2->CR2;
  //uint32_t checkSPI2_CRCPR = SPI2->CRCPR;
  //rxOnlyMode(SPI1,ENABLE); //If Slave Transmitted and Master Received, Rx only mode should enabled after SPE = 1.
  uint16_t data1;
  uint16_t data2;

    while(1)
    {
    /*****Master Transmitted and Slave Received*****/
    
     if( SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) ){
        SPI_I2S_SendData(SPI1,'H');
      }
      
     if( SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)  ){
        data1 = SPI_I2S_ReceiveData(SPI2);
      }

    /*****Slave Transmitted and Master Received*****/

     // if( SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) ){
       // SPI_I2S_SendData(SPI2,'L');
     // }

     // if( SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) ){
       // data2 = SPI_I2S_ReceiveData(SPI1);
     // }

    }
}


/******GPIO Setting Option******/
  /*
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz, 
  GPIO_Speed_50MHz
  
  GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 0x14,
  GPIO_Mode_Out_PP = 0x10,
  GPIO_Mode_AF_OD = 0x1C,
  GPIO_Mode_AF_PP = 0x18
  */ 
