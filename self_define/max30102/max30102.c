#include "max30102.h"

/**
 * I2C GPIO初始化 - 配置PB0(SCL)和PB1(SDA)为开漏输出
 */
void I2C_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* 使能GPIOB时钟 */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* 配置SCL和SDA引脚 */
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 低速
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* 初始状态：SCL和SDA都为高 */
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
}

/**
 * I2C起始信号
 */
void I2C_Start(void) {
    /* SDA从高到低，SCL保持高 */
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
}

/**
 * I2C停止信号
 */
void I2C_Stop(void) {
    /* SDA从低到高，SCL保持高 */
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    delay_us(4);
}

/**
 * 发送一个字节并等待ACK
 */
void I2C_SendByte(uint8_t byte) {
    uint8_t i;
    
    for (i = 0; i < 8; i++) {
        if (byte & 0x80)
            HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
        
        byte <<= 1;
        delay_us(2);
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        delay_us(2);
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        delay_us(2);
    }
}

/**
 * 读取一个字节并返回
 * ack = 1时发送ACK，ack = 0时发送NACK
 */
uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t i, byte = 0;
    
    /* 配置SDA为输入模式 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    for (i = 0; i < 8; i++) {
        byte <<= 1;
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
        delay_us(2);
        if (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN))
            byte |= 0x01;
        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
        delay_us(2);
    }
    
    /* 恢复SDA为输出模式 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    /* 发送ACK或NACK */
    if (ack)
        I2C_Ack();
    else
        I2C_NAck();
    
    return byte;
}

/**
 * 等待从设备的ACK信号
 */
uint8_t I2C_WaitAck(void) {
    uint8_t ack;
    
    /* 配置SDA为输入模式 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    delay_us(2);
    ack = HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    delay_us(2);
    
    /* 恢复SDA为输出模式 */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    return ack;  // 返回0表示ACK，返回1表示NACK
}

/**
 * 发送ACK信号
 */
void I2C_Ack(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
}

/**
 * 发送NACK信号
 */
void I2C_NAck(void) {
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    delay_us(2);
}

/**
 * 微秒级延时（使用SysTick定时器）
 */
void delay_us(uint32_t us) {
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;  // 获取重装载值
    
    ticks = us * (SystemCoreClock / 1000000);  // 计算需要的节拍数
    
    told = SysTick->VAL;  // 保存当前计数器值
    
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            // 计算差值
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
            
            told = tnow;
            
            // 判断是否达到指定的节拍数
            if (tcnt >= ticks) break;
        }
    }
}

/**
 * MAX30102初始化
 */
void MAX30102_Init(void) {
    uint8_t i;
    
    // 复位MAX30102
    MAX30102_WriteReg(MAX30102_REG_MODE_CFG, 0x40);  // 复位位
    delay_us(10000);  // 等待复位完成
    
    // 清除所有中断
    MAX30102_ReadReg(MAX30102_REG_INTR1);
    MAX30102_ReadReg(MAX30102_REG_INTR2);
    
    // 配置FIFO
    MAX30102_WriteReg(MAX30102_REG_FIFO_CFG, 0x00);  // 无FIFO滚动，采样平均为1
    
    // 配置模式为SpO2模式（同时采集红光和红外光）
    MAX30102_WriteReg(MAX30102_REG_MODE_CFG, 0x03);  // SpO2模式
    
    // 配置SpO2参数
    // 采样率：100Hz，ADC范围：16384nA，脉宽：411μs
    MAX30102_WriteReg(MAX30102_REG_SPO2_CFG, 0x27);
    
    // 配置LED功率
    // 红外LED：0x24（约4.5mA），红光LED：0x24（约4.5mA）
    MAX30102_WriteReg(MAX30102_REG_LED1_PA, 0x24);
    MAX30102_WriteReg(MAX30102_REG_LED2_PA, 0x24);
    
    // 禁用环境光抑制
    MAX30102_WriteReg(MAX30102_REG_MULTILED1, 0x00);
    
    // 清除FIFO
    for (i = 0; i < 16; i++) {
        uint32_t red, ir;
        MAX30102_ReadFIFO(&red, &ir);
    }
}

/**
 * 读取MAX30102寄存器
 */
uint8_t MAX30102_ReadReg(uint8_t regAddr) {
    uint8_t value;
    
    I2C_Start();
    I2C_SendByte(MAX30102_WRITE_ADDR);  // 已包含写位(0)，无需&0xFE
    I2C_WaitAck();
    I2C_SendByte(regAddr);                   // 发送寄存器地址
    I2C_WaitAck();
    
    I2C_Start();                              // 重新开始
    I2C_SendByte(MAX30102_READ_ADDR);  // 已包含读位(1)
    I2C_WaitAck();
    value = I2C_ReadByte(0);                  // 读取数据，发送NACK
    I2C_Stop();
    
    return value;
}

/**
 * 写入MAX30102寄存器
 */
void MAX30102_WriteReg(uint8_t regAddr, uint8_t value) {
    I2C_Start();
    I2C_SendByte(MAX30102_WRITE_ADDR);  // 已包含写位(0)，无需&0xFE
    I2C_WaitAck();
    I2C_SendByte(regAddr);                   // 发送寄存器地址
    I2C_WaitAck();
    I2C_SendByte(value);                     // 发送数据
    I2C_WaitAck();
    I2C_Stop();
}

/**
 * 从FIFO读取红光和红外光数据
 */
void MAX30102_ReadFIFO(uint32_t *red, uint32_t *ir) {
    uint8_t byte1, byte2, byte3;
    
    I2C_Start();
    I2C_SendByte(MAX30102_WRITE_ADDR);  // 已包含写位(0)，无需&0xFE
    I2C_WaitAck();
    I2C_SendByte(MAX30102_REG_FIFO_DATA);    // 发送FIFO数据寄存器地址
    I2C_WaitAck();
    
    I2C_Start();                              // 重新开始
    I2C_SendByte(MAX30102_READ_ADDR);  // 已包含读位(1)
    I2C_WaitAck();
    
    // 读取红光数据（3字节）
    byte1 = I2C_ReadByte(1);
    byte2 = I2C_ReadByte(1);
    byte3 = I2C_ReadByte(1);
    *red = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | byte3;
    *red &= 0x03FFFF;  // 保留低18位
    
    // 读取红外光数据（3字节）
    byte1 = I2C_ReadByte(1);
    byte2 = I2C_ReadByte(1);
    byte3 = I2C_ReadByte(0);  // 最后一个字节，发送NACK
    *ir = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | byte3;
    *ir &= 0x03FFFF;  // 保留低18位
    
    I2C_Stop();
}

/**
 * 读取MAX30102内部温度
 */
float MAX30102_ReadTemperature(void) {
    uint8_t int_part;
    uint8_t frac_part;
    float temperature;
    
    // 触发温度读取
    MAX30102_WriteReg(MAX30102_REG_MODE_CFG, 0x02);  // 温度模式
    
    // 等待转换完成
    delay_us(10000);
    
    // 读取温度值
    int_part = MAX30102_ReadReg(MAX30102_REG_TEMP_INT);
    frac_part = MAX30102_ReadReg(MAX30102_REG_TEMP_FRAC);
    
    // 恢复SpO2模式
    MAX30102_WriteReg(MAX30102_REG_MODE_CFG, 0x03);
    
    // 计算温度值（整数部分 + 小数部分/4）
    temperature = (float)int_part + (float)frac_part / 4.0f;
    
    return temperature;
}
