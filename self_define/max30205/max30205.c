#include "max30205.h"

/*定义I2C3的地址*/
extern I2C_HandleTypeDef hi2c3;

/**
 * @brief  通过I2C从MAX30205传感器指定寄存器读取多个字节数据
 * @param  address: MAX30205设备的I2C从机地址（需包含读写位，如0x48<<1）
 * @param  reg: 要读取的寄存器地址（如温度寄存器MAX30205_TEMP_REG）
 * @param  data: 用于存储读取数据的缓冲区指针
 * @param  size: 要读取的数据字节数（1 <= size <= 2）
 * @return oid: 函数执行状态（通常为HAL库状态码，如HAL_OK/HAL_ERROR）
 * @note   - 使用I2C内存读取模式（I2C_MEMADD_SIZE_8BIT）
 *         - 需提前初始化I2C总线（如hi2c3）
 *         - 建议在返回HAL_OK后再访问data缓冲区
 */
void max30205_i2c_readBytes(uint8_t address, uint8_t reg, uint8_t* data, uint8_t size)	
{
  HAL_I2C_Mem_Read(&hi2c3, MAX30205_ADDR, reg,	1, data, size, 100);
}

/**
 * @brief  通过I2C从MAX30205传感器指定寄存器读取单字节数据
 * @param  address: MAX30205设备的I2C从机地址（需包含读写位，如0x48<<1）
 * @param  reg: 要读取的寄存器地址（如配置寄存器MAX30205_CONFIG_REG）
 * @return uint8_t: 读取到的寄存器值（0x00~0xFF）
 * @note   - 内部调用HAL_I2C_Mem_Read，自动处理1字节读取
 *         - 若读取失败，返回值可能为0xFF（需结合硬件确认）
 *         - 建议在读取敏感寄存器（如温度）前等待转换完成
 */
uint8_t max30205_i2c_readByte(uint8_t address, uint8_t reg)
{
	uint8_t value = 0;
  HAL_I2C_Mem_Read(&hi2c3, MAX30205_ADDR, reg,	1, (uint8_t*)&value, 1, 100);
	return value;
}

/**
 * @brief  通过I2C从MAX30205传感器指定寄存器读取单字节数据
 * @param  address: MAX30205设备的I2C从机地址（需包含读写位，如0x48<<1）
 * @param  reg: 要读取的寄存器地址（如配置寄存器MAX30205_CONFIG_REG）
 * @return uint8_t: 读取到的寄存器值（0x00~0xFF）
 * @note   - 内部调用HAL_I2C_Mem_Read，自动处理1字节读取
 *         - 若读取失败，返回值可能为0xFF（需结合硬件确认）
 *         - 建议在读取敏感寄存器（如温度）前等待转换完成
 */
uint8_t max30205_i2c_writeByte(uint8_t address, uint8_t reg, uint8_t value)
{
	uint8_t ret;
  ret = HAL_I2C_Mem_Write(&hi2c3, MAX30205_ADDR, reg,	1, (uint8_t*)&value, 1, 100);
	return ret;
}

/**
 * @brief       MAX30205传感器初始化函数
 * @details     配置传感器的工作模式、温度阈值寄存器（默认均设为0x00）
 *              - 配置寄存器（CONFIG）：设置为默认模式（连续转换模式，参考数据手册）
 *              - 滞后阈值寄存器（THYST）：温度滞后阈值（高温报警后降温时的回差）
 *              - 过热阈值寄存器（TOS）：高温报警阈值
 * @note        - 初始化后传感器默认处于连续转换模式，持续输出温度数据
 *              - 阈值寄存器的单位需根据数据手册解析（通常为0.0625°C/LSB）
 */
void MAX30205_Init(void)
{
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_CONFIG, 0x00); //模式选择	
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_THYST , 0x00); //阈值设定
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_TOS, 0x00); 
}

/**
 * @brief       将MAX30205设置为关机模式（低功耗状态）
 * @details     - 读取当前配置寄存器值，保留原有位设置
 *              - 置位第0位（SHDN位）为1，进入关机模式
 * @note        - 关机模式下传感器停止温度转换，功耗降至最低
 *              - 唤醒需通过One-Shot模式或重新配置寄存器
 */
void MAX30205_Shutdown(void)
{
  uint8_t config_reg = max30205_i2c_readByte(MAX30205_ADDR, MAX30205_CONFIG);  // 选择寄存器
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_CONFIG, config_reg | 0x01);
}

/**
 * @brief       配置MAX30205进入One-Shot单次温度转换模式
 * @details     - 先调用关机函数确保传感器处于低功耗状态
 *              - 读取配置寄存器，置位第7位（OS位）为1，触发单次转换
 * @note        - 单次模式下，传感器完成一次温度转换后自动进入关机模式
 *              - 适用于非连续采样场景，降低平均功耗
 *              - 转换完成后需通过读取温度寄存器获取数据
 */
void MAX30205_OneShotMode(void)
{
	MAX30205_Shutdown();
	uint8_t config_reg = max30205_i2c_readByte(MAX30205_ADDR, MAX30205_CONFIG);  // 选择寄存器
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_CONFIG, config_reg | 0x80);
}

/**
 * @brief       获取MAX30205的温度数据（单位：°C）
 * @details     - 触发一次One-Shot模式（确保获取最新转换数据）
 *              - 读取2字节温度寄存器（TEMP_REG，大端格式）
 *              - 将16位原始数据转换为浮点数，分辨率为0.00390625°C/LSB
 * @return      float：温度值（如25.5°C）
 * @note        - 原始数据为带符号整数，最高位为符号位（0=正数，1=负数）
 *              - 转换过程：buffer[0]为高8位，buffer[1]为低8位，组合为16位整数
 *              - 建议在调用前确保传感器已完成转换（可通过延时或状态寄存器判断）
 */
float MAX30205_GetTemperature(void) 
{
	uint8_t buffer[2];
	uint8_t config_reg = max30205_i2c_readByte(MAX30205_ADDR, MAX30205_CONFIG);  // 选择寄存器
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_CONFIG, config_reg | 0x80);   // 设置为one-shot模式
	max30205_i2c_readBytes(MAX30205_ADDR, MAX30205_TEMP, buffer, 2);
	int16_t raw = ((uint16_t)buffer[0] << 8) | buffer[1];
	return	((double)raw  * 0.00390625);
}
