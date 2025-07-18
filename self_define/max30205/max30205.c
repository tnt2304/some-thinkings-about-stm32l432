#include "max30205.h"

/*����I2C3�ĵ�ַ*/
extern I2C_HandleTypeDef hi2c3;

/**
 * @brief  ͨ��I2C��MAX30205������ָ���Ĵ�����ȡ����ֽ�����
 * @param  address: MAX30205�豸��I2C�ӻ���ַ���������дλ����0x48<<1��
 * @param  reg: Ҫ��ȡ�ļĴ�����ַ�����¶ȼĴ���MAX30205_TEMP_REG��
 * @param  data: ���ڴ洢��ȡ���ݵĻ�����ָ��
 * @param  size: Ҫ��ȡ�������ֽ�����1 <= size <= 2��
 * @return oid: ����ִ��״̬��ͨ��ΪHAL��״̬�룬��HAL_OK/HAL_ERROR��
 * @note   - ʹ��I2C�ڴ��ȡģʽ��I2C_MEMADD_SIZE_8BIT��
 *         - ����ǰ��ʼ��I2C���ߣ���hi2c3��
 *         - �����ڷ���HAL_OK���ٷ���data������
 */
void max30205_i2c_readBytes(uint8_t address, uint8_t reg, uint8_t* data, uint8_t size)	
{
  HAL_I2C_Mem_Read(&hi2c3, MAX30205_ADDR, reg,	1, data, size, 100);
}

/**
 * @brief  ͨ��I2C��MAX30205������ָ���Ĵ�����ȡ���ֽ�����
 * @param  address: MAX30205�豸��I2C�ӻ���ַ���������дλ����0x48<<1��
 * @param  reg: Ҫ��ȡ�ļĴ�����ַ�������üĴ���MAX30205_CONFIG_REG��
 * @return uint8_t: ��ȡ���ļĴ���ֵ��0x00~0xFF��
 * @note   - �ڲ�����HAL_I2C_Mem_Read���Զ�����1�ֽڶ�ȡ
 *         - ����ȡʧ�ܣ�����ֵ����Ϊ0xFF������Ӳ��ȷ�ϣ�
 *         - �����ڶ�ȡ���мĴ��������¶ȣ�ǰ�ȴ�ת�����
 */
uint8_t max30205_i2c_readByte(uint8_t address, uint8_t reg)
{
	uint8_t value = 0;
  HAL_I2C_Mem_Read(&hi2c3, MAX30205_ADDR, reg,	1, (uint8_t*)&value, 1, 100);
	return value;
}

/**
 * @brief  ͨ��I2C��MAX30205������ָ���Ĵ�����ȡ���ֽ�����
 * @param  address: MAX30205�豸��I2C�ӻ���ַ���������дλ����0x48<<1��
 * @param  reg: Ҫ��ȡ�ļĴ�����ַ�������üĴ���MAX30205_CONFIG_REG��
 * @return uint8_t: ��ȡ���ļĴ���ֵ��0x00~0xFF��
 * @note   - �ڲ�����HAL_I2C_Mem_Read���Զ�����1�ֽڶ�ȡ
 *         - ����ȡʧ�ܣ�����ֵ����Ϊ0xFF������Ӳ��ȷ�ϣ�
 *         - �����ڶ�ȡ���мĴ��������¶ȣ�ǰ�ȴ�ת�����
 */
uint8_t max30205_i2c_writeByte(uint8_t address, uint8_t reg, uint8_t value)
{
	uint8_t ret;
  ret = HAL_I2C_Mem_Write(&hi2c3, MAX30205_ADDR, reg,	1, (uint8_t*)&value, 1, 100);
	return ret;
}

/**
 * @brief       MAX30205��������ʼ������
 * @details     ���ô������Ĺ���ģʽ���¶���ֵ�Ĵ�����Ĭ�Ͼ���Ϊ0x00��
 *              - ���üĴ�����CONFIG��������ΪĬ��ģʽ������ת��ģʽ���ο������ֲᣩ
 *              - �ͺ���ֵ�Ĵ�����THYST�����¶��ͺ���ֵ�����±�������ʱ�Ļز
 *              - ������ֵ�Ĵ�����TOS�������±�����ֵ
 * @note        - ��ʼ���󴫸���Ĭ�ϴ�������ת��ģʽ����������¶�����
 *              - ��ֵ�Ĵ����ĵ�λ����������ֲ������ͨ��Ϊ0.0625��C/LSB��
 */
void MAX30205_Init(void)
{
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_CONFIG, 0x00); //ģʽѡ��	
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_THYST , 0x00); //��ֵ�趨
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_TOS, 0x00); 
}

/**
 * @brief       ��MAX30205����Ϊ�ػ�ģʽ���͹���״̬��
 * @details     - ��ȡ��ǰ���üĴ���ֵ������ԭ��λ����
 *              - ��λ��0λ��SHDNλ��Ϊ1������ػ�ģʽ
 * @note        - �ػ�ģʽ�´�����ֹͣ�¶�ת�������Ľ������
 *              - ������ͨ��One-Shotģʽ���������üĴ���
 */
void MAX30205_Shutdown(void)
{
  uint8_t config_reg = max30205_i2c_readByte(MAX30205_ADDR, MAX30205_CONFIG);  // ѡ��Ĵ���
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_CONFIG, config_reg | 0x01);
}

/**
 * @brief       ����MAX30205����One-Shot�����¶�ת��ģʽ
 * @details     - �ȵ��ùػ�����ȷ�����������ڵ͹���״̬
 *              - ��ȡ���üĴ�������λ��7λ��OSλ��Ϊ1����������ת��
 * @note        - ����ģʽ�£����������һ���¶�ת�����Զ�����ػ�ģʽ
 *              - �����ڷ�������������������ƽ������
 *              - ת����ɺ���ͨ����ȡ�¶ȼĴ�����ȡ����
 */
void MAX30205_OneShotMode(void)
{
	MAX30205_Shutdown();
	uint8_t config_reg = max30205_i2c_readByte(MAX30205_ADDR, MAX30205_CONFIG);  // ѡ��Ĵ���
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_CONFIG, config_reg | 0x80);
}

/**
 * @brief       ��ȡMAX30205���¶����ݣ���λ����C��
 * @details     - ����һ��One-Shotģʽ��ȷ����ȡ����ת�����ݣ�
 *              - ��ȡ2�ֽ��¶ȼĴ�����TEMP_REG����˸�ʽ��
 *              - ��16λԭʼ����ת��Ϊ���������ֱ���Ϊ0.00390625��C/LSB
 * @return      float���¶�ֵ����25.5��C��
 * @note        - ԭʼ����Ϊ���������������λΪ����λ��0=������1=������
 *              - ת�����̣�buffer[0]Ϊ��8λ��buffer[1]Ϊ��8λ�����Ϊ16λ����
 *              - �����ڵ���ǰȷ�������������ת������ͨ����ʱ��״̬�Ĵ����жϣ�
 */
float MAX30205_GetTemperature(void) 
{
	uint8_t buffer[2];
	uint8_t config_reg = max30205_i2c_readByte(MAX30205_ADDR, MAX30205_CONFIG);  // ѡ��Ĵ���
  max30205_i2c_writeByte(MAX30205_ADDR, MAX30205_CONFIG, config_reg | 0x80);   // ����Ϊone-shotģʽ
	max30205_i2c_readBytes(MAX30205_ADDR, MAX30205_TEMP, buffer, 2);
	int16_t raw = ((uint16_t)buffer[0] << 8) | buffer[1];
	return	((double)raw  * 0.00390625);
}
