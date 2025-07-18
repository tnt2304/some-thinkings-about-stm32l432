#include "max30102.h"

/**
 * I2C GPIO��ʼ�� - ����PB0(SCL)��PB1(SDA)Ϊ��©���
 */
void I2C_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* ʹ��GPIOBʱ�� */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* ����SCL��SDA���� */
    GPIO_InitStruct.Pin = I2C_SCL_PIN | I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;  // ��©���
    GPIO_InitStruct.Pull = GPIO_PULLUP;          // ����
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // ����
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    /* ��ʼ״̬��SCL��SDA��Ϊ�� */
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
}

/**
 * I2C��ʼ�ź�
 */
void I2C_Start(void) {
    /* SDA�Ӹߵ��ͣ�SCL���ָ� */
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
}

/**
 * I2Cֹͣ�ź�
 */
void I2C_Stop(void) {
    /* SDA�ӵ͵��ߣ�SCL���ָ� */
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);
    delay_us(4);
    HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
    delay_us(4);
}

/**
 * ����һ���ֽڲ��ȴ�ACK
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
 * ��ȡһ���ֽڲ�����
 * ack = 1ʱ����ACK��ack = 0ʱ����NACK
 */
uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t i, byte = 0;
    
    /* ����SDAΪ����ģʽ */
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
    
    /* �ָ�SDAΪ���ģʽ */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    /* ����ACK��NACK */
    if (ack)
        I2C_Ack();
    else
        I2C_NAck();
    
    return byte;
}

/**
 * �ȴ����豸��ACK�ź�
 */
uint8_t I2C_WaitAck(void) {
    uint8_t ack;
    
    /* ����SDAΪ����ģʽ */
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
    
    /* �ָ�SDAΪ���ģʽ */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(I2C_SDA_PORT, &GPIO_InitStruct);
    
    return ack;  // ����0��ʾACK������1��ʾNACK
}

/**
 * ����ACK�ź�
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
 * ����NACK�ź�
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
 * ΢�뼶��ʱ��ʹ��SysTick��ʱ����
 */
void delay_us(uint32_t us) {
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;  // ��ȡ��װ��ֵ
    
    ticks = us * (SystemCoreClock / 1000000);  // ������Ҫ�Ľ�����
    
    told = SysTick->VAL;  // ���浱ǰ������ֵ
    
    while (1) {
        tnow = SysTick->VAL;
        if (tnow != told) {
            // �����ֵ
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += reload - tnow + told;
            
            told = tnow;
            
            // �ж��Ƿ�ﵽָ���Ľ�����
            if (tcnt >= ticks) break;
        }
    }
}

/**
 * MAX30102��ʼ��
 */
void MAX30102_Init(void) {
    uint8_t i;
    
    // ��λMAX30102
    MAX30102_WriteReg(MAX30102_REG_MODE_CFG, 0x40);  // ��λλ
    delay_us(10000);  // �ȴ���λ���
    
    // ��������ж�
    MAX30102_ReadReg(MAX30102_REG_INTR1);
    MAX30102_ReadReg(MAX30102_REG_INTR2);
    
    // ����FIFO
    MAX30102_WriteReg(MAX30102_REG_FIFO_CFG, 0x00);  // ��FIFO����������ƽ��Ϊ1
    
    // ����ģʽΪSpO2ģʽ��ͬʱ�ɼ����ͺ���⣩
    MAX30102_WriteReg(MAX30102_REG_MODE_CFG, 0x03);  // SpO2ģʽ
    
    // ����SpO2����
    // �����ʣ�100Hz��ADC��Χ��16384nA������411��s
    MAX30102_WriteReg(MAX30102_REG_SPO2_CFG, 0x27);
    
    // ����LED����
    // ����LED��0x24��Լ4.5mA�������LED��0x24��Լ4.5mA��
    MAX30102_WriteReg(MAX30102_REG_LED1_PA, 0x24);
    MAX30102_WriteReg(MAX30102_REG_LED2_PA, 0x24);
    
    // ���û���������
    MAX30102_WriteReg(MAX30102_REG_MULTILED1, 0x00);
    
    // ���FIFO
    for (i = 0; i < 16; i++) {
        uint32_t red, ir;
        MAX30102_ReadFIFO(&red, &ir);
    }
}

/**
 * ��ȡMAX30102�Ĵ���
 */
uint8_t MAX30102_ReadReg(uint8_t regAddr) {
    uint8_t value;
    
    I2C_Start();
    I2C_SendByte(MAX30102_WRITE_ADDR);  // �Ѱ���дλ(0)������&0xFE
    I2C_WaitAck();
    I2C_SendByte(regAddr);                   // ���ͼĴ�����ַ
    I2C_WaitAck();
    
    I2C_Start();                              // ���¿�ʼ
    I2C_SendByte(MAX30102_READ_ADDR);  // �Ѱ�����λ(1)
    I2C_WaitAck();
    value = I2C_ReadByte(0);                  // ��ȡ���ݣ�����NACK
    I2C_Stop();
    
    return value;
}

/**
 * д��MAX30102�Ĵ���
 */
void MAX30102_WriteReg(uint8_t regAddr, uint8_t value) {
    I2C_Start();
    I2C_SendByte(MAX30102_WRITE_ADDR);  // �Ѱ���дλ(0)������&0xFE
    I2C_WaitAck();
    I2C_SendByte(regAddr);                   // ���ͼĴ�����ַ
    I2C_WaitAck();
    I2C_SendByte(value);                     // ��������
    I2C_WaitAck();
    I2C_Stop();
}

/**
 * ��FIFO��ȡ���ͺ��������
 */
void MAX30102_ReadFIFO(uint32_t *red, uint32_t *ir) {
    uint8_t byte1, byte2, byte3;
    
    I2C_Start();
    I2C_SendByte(MAX30102_WRITE_ADDR);  // �Ѱ���дλ(0)������&0xFE
    I2C_WaitAck();
    I2C_SendByte(MAX30102_REG_FIFO_DATA);    // ����FIFO���ݼĴ�����ַ
    I2C_WaitAck();
    
    I2C_Start();                              // ���¿�ʼ
    I2C_SendByte(MAX30102_READ_ADDR);  // �Ѱ�����λ(1)
    I2C_WaitAck();
    
    // ��ȡ������ݣ�3�ֽڣ�
    byte1 = I2C_ReadByte(1);
    byte2 = I2C_ReadByte(1);
    byte3 = I2C_ReadByte(1);
    *red = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | byte3;
    *red &= 0x03FFFF;  // ������18λ
    
    // ��ȡ��������ݣ�3�ֽڣ�
    byte1 = I2C_ReadByte(1);
    byte2 = I2C_ReadByte(1);
    byte3 = I2C_ReadByte(0);  // ���һ���ֽڣ�����NACK
    *ir = ((uint32_t)byte1 << 16) | ((uint32_t)byte2 << 8) | byte3;
    *ir &= 0x03FFFF;  // ������18λ
    
    I2C_Stop();
}

/**
 * ��ȡMAX30102�ڲ��¶�
 */
float MAX30102_ReadTemperature(void) {
    uint8_t int_part;
    uint8_t frac_part;
    float temperature;
    
    // �����¶ȶ�ȡ
    MAX30102_WriteReg(MAX30102_REG_MODE_CFG, 0x02);  // �¶�ģʽ
    
    // �ȴ�ת�����
    delay_us(10000);
    
    // ��ȡ�¶�ֵ
    int_part = MAX30102_ReadReg(MAX30102_REG_TEMP_INT);
    frac_part = MAX30102_ReadReg(MAX30102_REG_TEMP_FRAC);
    
    // �ָ�SpO2ģʽ
    MAX30102_WriteReg(MAX30102_REG_MODE_CFG, 0x03);
    
    // �����¶�ֵ���������� + С������/4��
    temperature = (float)int_part + (float)frac_part / 4.0f;
    
    return temperature;
}
