/**
 * This Library was originally written by RedMagic Electric Drivetrain Studio in 2025
 * Notes are generated by doubao ai
 * Module for stm32l432xx software iic
 * Official Documentation Routines
 */

#ifndef __MAX30102_H
#define __MAX30102_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

/* I2C引脚定义 */
#define I2C_SCL_PIN       GPIO_PIN_0
#define I2C_SCL_PORT      GPIOB
#define I2C_SDA_PIN       GPIO_PIN_1
#define I2C_SDA_PORT      GPIOB

/* MAX30102寄存器地址 */
#define MAX30102_WRITE_ADDR      0xAE  // 7位写地址  
#define MAX30102_READ_ADDR       0xAF  // 7位读地址  

#define MAX30102_REG_INTR1       0x00  // 中断状态寄存器1  
#define MAX30102_REG_INTR2       0x01  // 中断状态寄存器2  
#define MAX30102_REG_INTR_ENABLE1 0x02  // 中断使能寄存器1（原FIFO_WR改为中断使能）
#define MAX30102_REG_INTR_ENABLE2 0x03  // 中断使能寄存器2（原FIFO_OV改为中断使能）

#define MAX30102_REG_FIFO_WR     0x04  // FIFO写指针（原FIFO_RD改为写指针）
#define MAX30102_REG_FIFO_OV     0x05  // FIFO溢出计数器（原FIFO_DAT改为溢出计数器）
#define MAX30102_REG_FIFO_RD     0x06  // FIFO读指针（新增，原地址0x06）
#define MAX30102_REG_FIFO_DATA   0x07  // FIFO数据寄存器（新增，原地址0x07）

#define MAX30102_REG_FIFO_CFG    0x08  // FIFO配置寄存器（原地址0x08）
#define MAX30102_REG_MODE_CFG    0x09  // 模式配置寄存器  
#define MAX30102_REG_SPO2_CFG    0x0A  // SpO2配置寄存器  
#define MAX30102_REG_LED1_PA     0x0C  // 红外LED功率控制  
#define MAX30102_REG_LED2_PA     0x0D  // 红光LED功率控制  

#define MAX30102_REG_MULTILED1   0x11  // 多通道模式1（原多通道配置寄存器）
#define MAX30102_REG_MULTILED2   0x12  // 多通道模式2（新增）

#define MAX30102_REG_TEMP_INT    0x1F  // 温度整数部分（原地址0x1F）
#define MAX30102_REG_TEMP_FRAC   0x20  // 温度小数部分（原地址0x20）
#define MAX30102_REG_TEMP_CFG    0x21  // 温度配置寄存器（新增）

#define MAX30102_REG_REV_ID      0xFE  // 修订ID  
#define MAX30102_REG_PART_ID     0xFF  // 部件ID  

/**
 * I2C底层操作函数声明 - 软件模拟I2C通信协议
 */
void I2C_GPIO_Init(void);           // 初始化I2C通信引脚
void I2C_Start(void);               // 发送I2C起始信号
void I2C_Stop(void);                // 发送I2C停止信号
void I2C_SendByte(uint8_t byte);    // 发送一个字节数据
uint8_t I2C_ReadByte(uint8_t ack);  // 读取一个字节数据并发送应答信号
uint8_t I2C_WaitAck(void);          // 等待从设备应答信号
void I2C_Ack(void);                 // 发送应答信号
void I2C_NAck(void);                // 发送非应答信号
void delay_us(uint32_t us);         // 微秒级延时函数

/**
 * MAX30102功能函数声明 - 控制MAX30102芯片的各种操作
 */
void MAX30102_Init(void);                 // 初始化MAX30102传感器
uint8_t MAX30102_ReadReg(uint8_t regAddr); // 读取MAX30102指定寄存器的值
void MAX30102_WriteReg(uint8_t regAddr, uint8_t value); // 向MAX30102指定寄存器写入值
void MAX30102_ReadFIFO(uint32_t *red, uint32_t *ir); // 读取FIFO中的红光和红外光数据
float MAX30102_ReadTemperature(void);     // 读取MAX30102内部温度

#endif /* __MAX30102_H */
