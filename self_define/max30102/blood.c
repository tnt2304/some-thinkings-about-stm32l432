
#include "blood.h"
#include "max30102.h"
#include <math.h>

/* �ڲ�״̬���� */
static uint32_t redBuffer[BLOOD_BUFFER_SIZE];
static uint32_t irBuffer[BLOOD_BUFFER_SIZE];
static uint8_t bufferIndex = 0;
static uint32_t sampleCount = 0;
static uint32_t lastPulseTime = 0;
static uint32_t pulseIntervals[5] = {0};
static uint8_t intervalIndex = 0;
static float filteredRed = 0.0f;
static float filteredIr = 0.0f;
static float highPassRed = 0.0f;
static float highPassIr = 0.0f;
static float prevRed = 0.0f;
static float prevIr = 0.0f;
static float prevRedHp = 0.0f;
static float prevIrHp = 0.0f;
static uint8_t currentSpO2 = 0;
static uint8_t currentHeartRate = 0;
static bool pulseDetected = false;

/**
 * һ�׵�ͨ�˲��� - ƽ���ź�ȥ����Ƶ����
 * @param input ��ǰ����ֵ
 * @param output ��һ�ε����ֵ���˲������
 * @param alpha �˲�ϵ��(0.0~1.0)��Խ����ӦԽ�죬ԽСƽ��Ч��Խ��
 * @return �˲�������ֵ
 * @note ���ݺ�����H(z) = �� / (1 - (1-��)z^(-1))
 *       - ���ӽ�1ʱ���˲�����ӦѸ�ٵ�ƽ��Ч����
 *       - ���ӽ�0ʱ���˲�����Ӧ������ƽ��Ч��ǿ
 */
static float LowPassFilter(float input, float output, float alpha) {
    return alpha * input + (1.0f - alpha) * output;
}

/**
 * һ�׸�ͨ�˲��� - �����źŸ�Ƶ�ɷ֣�ȥ��ֱ��ƫ��
 * @param input ��ǰ����ֵ
 * @param prevInput ��һ�ε�����ֵ�����ⲿά����
 * @param prevOutput ��һ�ε����ֵ�����ⲿά����
 * @param alpha �˲�ϵ��(0.0~1.0)��ԽС��ֹƵ��Խ��
 * @return �˲�������ֵ
 * @note ���ݺ�����H(z) = (1-��)(1-z^(-1)) / (1 - (1-��)z^(-1))
 *       - ��ֹƵ�� fc �� fs * �� / (2��)
 *       - ʹ��ǰ���ʼ��prevInput��prevOutputΪ0
 */
static float HighPassFilter(float input, float *prevInput, float *prevOutput, float alpha) {
    float output = alpha * (*prevOutput + input - *prevInput);
    *prevInput = input;
    *prevOutput = output;
    return output;
}

/**
 * ���������ֵ - ʹ��״̬���㷨ʶ����������
 * @param signal ��ǰ����������źţ����˲���
 * @param prevSignal ��һ�ε��ź�ֵ�����ڱȽϱ仯���ƣ�
 * @param threshold ��ֵ�����ֵ�������źŷ��ȶ�̬������
 * @return true��ʾ��⵽��ֵ��false��ʾδ��⵽
 * @note ״̬��˵����
 *       - ״̬0���ȴ��ź������أ���⵽�����������״̬1
 *       - ״̬1�������ź������أ���¼���ֵ���½�ʱ����״̬2
 *       - ״̬2������ź��½��أ�������ֵʱȷ�Ϸ�ֵ������״̬0
 */
static bool DetectPeak(float signal, float *prevSignal, float threshold) {
    static float peakValue = 0.0f;
    static uint8_t peakState = 0; // 0:�ȴ�������, 1:������, 2:�½���
    
    bool isPeak = false;
    
    if (peakState == 0) {
        // �ȴ�������
        if (signal > *prevSignal) {
            peakState = 1;
            peakValue = signal;
        }
    } else if (peakState == 1) {
        // ������
        if (signal > peakValue) {
            peakValue = signal;
        } else {
            peakState = 2;
        }
    } else if (peakState == 2) {
        // �½���
        if (signal < threshold) {
            // ȷ�Ϸ�ֵ
            isPeak = true;
            peakState = 0;
        }
    }
    
    *prevSignal = signal;
    return isPeak;
}

/**
 * �������� - ���������������ÿ������������
 * @param currentTime ��ǰ����ʱ�������λ��������������
 * @param isPeak ��־λ����ʾ�Ƿ��⵽������ֵ
 * @return ����õ�������ֵ����/���ӣ�
 * @note �㷨˵����
 *       - �洢���5����Ч�������������ƽ��ֵ
 *       - ���� = 60�� / ƽ���������/�Σ�
 *       - �����쳣��������ʷ�Χ40-220BPM��
 */
static uint8_t CalculateHeartRate(uint32_t currentTime, bool isPeak) {
    if (!isPeak) return currentHeartRate;
    
    uint32_t interval = currentTime - lastPulseTime;
    lastPulseTime = currentTime;
    
    // ������Ƿ��ں���Χ��
    const uint32_t minInterval = BLOOD_SAMPLE_RATE * 60 / 220; // �������220BPM
    const uint32_t maxInterval = BLOOD_SAMPLE_RATE * 60 / 40;  // ��С����40BPM
    
    if (interval < minInterval || interval > maxInterval) {
        return currentHeartRate;
    }
    
    // ������
    pulseIntervals[intervalIndex] = interval;
    intervalIndex = (intervalIndex + 1) % 5;
    
    // ����ƽ�����
    uint32_t sum = 0;
    uint8_t validCount = 0;
    for (uint8_t i = 0; i < 5; i++) {
        if (pulseIntervals[i] > 0) {
            sum += pulseIntervals[i];
            validCount++;
        }
    }
    
    if (validCount > 0) {
        uint32_t avgInterval = sum / validCount;
        currentHeartRate = (uint8_t)((float)BLOOD_SAMPLE_RATE * 60.0f / avgInterval);
    }
    
    return currentHeartRate;
}

/**
 * ����Ѫ�����Ͷ� - ����Beer-Lambert���ɵľ��鹫ʽ
 * @param redData ��⴫������������
 * @param irData ����⴫������������
 * @param size �������鳤��
 * @return ����õ���Ѫ��ֵ��%��
 * @note �㷨���裺
 *       1. ��ȡ�ź��е�AC��������������������DC��ֱ�������ߣ�����
 *       2. ����Rֵ = (AC_red/DC_red) / (AC_ir/DC_ir)
 *       3. ͨ�����鹫ʽ SpO2 = -45.06��R2 + 30.35��R + 94.85 ����Ѫ��
 *       4. ���������[BLOOD_MIN_SPO2, BLOOD_MAX_SPO2]��Χ��
 */
static uint8_t CalculateSpO2(uint32_t *redData, uint32_t *irData, uint8_t size) {
    // Ѱ���ź��еĽ���(AC)��ֱ��(DC)����
    uint32_t redMax = 0, redMin = 0xFFFFFFFF;
    uint32_t irMax = 0, irMin = 0xFFFFFFFF;
    uint32_t redSum = 0, irSum = 0;
    
    for (uint8_t i = 0; i < size; i++) {
        if (redData[i] > redMax) redMax = redData[i];
        if (redData[i] < redMin) redMin = redData[i];
        if (irData[i] > irMax) irMax = irData[i];
        if (irData[i] < irMin) irMin = irData[i];
        redSum += redData[i];
        irSum += irData[i];
    }
    
    // ���㽻����ֱ������
    float redAC = (float)(redMax - redMin);
    float redDC = (float)redSum / size;
    float irAC = (float)(irMax - irMin);
    float irDC = (float)irSum / size;
    
    // ����������
    if (redDC <= 0 || irDC <= 0 || irAC <= 0) {
        return 0;
    }
    
    // ����Rֵ = (ACred/DCred)/(ACir/DCir)
    float ratio = (redAC / redDC) / (irAC / irDC);
    
    // ���ݾ��鹫ʽ����SpO2
    // SpO2 = -45.060 * ratio^2 + 30.354 * ratio + 94.845
    float spo2 = -45.060f * ratio * ratio + 30.354f * ratio + 94.845f;
    
    // �����ں���Χ��
    if (spo2 > BLOOD_MAX_SPO2) spo2 = BLOOD_MAX_SPO2;
    if (spo2 < BLOOD_MIN_SPO2) spo2 = BLOOD_MIN_SPO2;
    
    return (uint8_t)spo2;
}

/**
 * ��ʼ��Ѫ������ģ�� - �����������������ڲ�״̬
 * @return BLOOD_OK��ʾ��ʼ���ɹ�������ֵ��ʾ����
 * @note ��ʼ�����̣�
 *       1. ����MAX30102_Init()��ʼ��Ӳ��������
 *       2. ����Blood_Reset()�����ʷ���ݺ�״̬����
 */
BloodError Blood_Init(void) {
    // ��ʼ��MAX30102������
    MAX30102_Init();
    
    // �����ڲ�״̬
    Blood_Reset();
    
    return BLOOD_OK;
}

/**
 * ���´��������� - ����ԭʼ���ݲ���������ָ��
 * @param red ��⴫����ԭʼֵ
 * @param ir ����⴫����ԭʼֵ
 * @note ���ݴ������̣�
 *       1. ����ԭʼ����
 *       2. ��ͨ�˲�ƽ���ź�
 *       3. ��ͨ�˲�ȥ������Ư��
 *       4. ��̬��ֵ���������ֵ
 *       5. �����Լ������ʺ�Ѫ��
 */
void Blood_Update(uint32_t red, uint32_t ir) {
    // ����ԭʼ���ݵ�������
    redBuffer[bufferIndex] = red;
    irBuffer[bufferIndex] = ir;
    bufferIndex = (bufferIndex + 1) % BLOOD_BUFFER_SIZE;
    
    // ��ͨ�˲� - ƽ���ź�
    filteredRed = LowPassFilter((float)red, filteredRed, 0.2f);
    filteredIr = LowPassFilter((float)ir, filteredIr, 0.2f);
    
    // ��ͨ�˲� - ȥ������Ư��
    highPassRed = HighPassFilter(filteredRed, &prevRed, &prevRedHp, 0.95f);
    highPassIr = HighPassFilter(filteredIr, &prevIr, &prevIrHp, 0.95f);
    
    // ��̬��ֵ���������ֵ
    float threshold = 0.5f * fabs(highPassRed);
    bool isPeak = DetectPeak(highPassRed, &prevRedHp, threshold);
    
    // ��������
    sampleCount++;
    if (isPeak) {
        currentHeartRate = CalculateHeartRate(sampleCount, true);
        pulseDetected = true;
    }
    
    // ÿ100����������һ��Ѫ��
    if (sampleCount % 100 == 0) {
        currentSpO2 = CalculateSpO2(redBuffer, irBuffer, BLOOD_BUFFER_SIZE);
    }
}

/**
 * ��ȡ���µ�Ѫ������������ - �����ṩ��������ӿ�
 * @param result �洢��������Ľṹ��ָ��
 * @return BLOOD_OK��ʾ��ȡ�ɹ�������ֵ��ʾ����
 * @note �������̣�
 *       1. ������������Ч��
 *       2. ��MAX30102��ȡԭʼ����
 *       3. ����Blood_Update()��������
 *       4. ������ṹ�岢����
 */
BloodError Blood_Read(BloodResult *result) {
    if (result == NULL) {
        return BLOOD_ERROR_READ;
    }
    
    uint32_t red, ir;
    MAX30102_ReadFIFO(&red, &ir);
    
    // ���´���������
    Blood_Update(red, ir);
    
    // �����
    result->spo2 = currentSpO2;
    result->heartRate = currentHeartRate;
    result->pulseDetected = pulseDetected;
    result->timestamp = sampleCount;
    
    // ������������־
    pulseDetected = false;
    
    return BLOOD_OK;
}

/**
 * ���ò���״̬ - �����ʷ���ݺ�״̬����
 * @note �������ݣ�
 *       1. �����������Ͳ�������
 *       2. ���������¼��ʱ���
 *       3. �˲��м������״̬������
 *       4. ����õ���Ѫ��������ֵ
 */
void Blood_Reset(void) {
    bufferIndex = 0;
    sampleCount = 0;
    lastPulseTime = 0;
    
    for (uint8_t i = 0; i < 5; i++) {
        pulseIntervals[i] = 0;
    }
    
    filteredRed = 0.0f;
    filteredIr = 0.0f;
    highPassRed = 0.0f;
    highPassIr = 0.0f;
    prevRed = 0.0f;
    prevIr = 0.0f;
    prevRedHp = 0.0f;
    prevIrHp = 0.0f;
    
    currentSpO2 = 0;
    currentHeartRate = 0;
    pulseDetected = false;
}
