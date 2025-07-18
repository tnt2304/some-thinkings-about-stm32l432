
#include "blood.h"
#include "max30102.h"
#include <math.h>

/* 内部状态变量 */
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
 * 一阶低通滤波器 - 平滑信号去除高频噪声
 * @param input 当前输入值
 * @param output 上一次的输出值（滤波结果）
 * @param alpha 滤波系数(0.0~1.0)，越大响应越快，越小平滑效果越好
 * @return 滤波后的输出值
 * @note 传递函数：H(z) = α / (1 - (1-α)z^(-1))
 *       - α接近1时，滤波器响应迅速但平滑效果弱
 *       - α接近0时，滤波器响应缓慢但平滑效果强
 */
static float LowPassFilter(float input, float output, float alpha) {
    return alpha * input + (1.0f - alpha) * output;
}

/**
 * 一阶高通滤波器 - 保留信号高频成分，去除直流偏置
 * @param input 当前输入值
 * @param prevInput 上一次的输入值（需外部维护）
 * @param prevOutput 上一次的输出值（需外部维护）
 * @param alpha 滤波系数(0.0~1.0)，越小截止频率越低
 * @return 滤波后的输出值
 * @note 传递函数：H(z) = (1-α)(1-z^(-1)) / (1 - (1-α)z^(-1))
 *       - 截止频率 fc ≈ fs * α / (2π)
 *       - 使用前需初始化prevInput和prevOutput为0
 */
static float HighPassFilter(float input, float *prevInput, float *prevOutput, float alpha) {
    float output = alpha * (*prevOutput + input - *prevInput);
    *prevInput = input;
    *prevOutput = output;
    return output;
}

/**
 * 检测脉搏峰值 - 使用状态机算法识别脉搏波峰
 * @param signal 当前输入的脉搏信号（已滤波）
 * @param prevSignal 上一次的信号值（用于比较变化趋势）
 * @param threshold 峰值检测阈值（基于信号幅度动态调整）
 * @return true表示检测到峰值，false表示未检测到
 * @note 状态机说明：
 *       - 状态0：等待信号上升沿，检测到上升沿则进入状态1
 *       - 状态1：跟踪信号上升沿，记录最大值，下降时进入状态2
 *       - 状态2：检测信号下降沿，低于阈值时确认峰值并返回状态0
 */
static bool DetectPeak(float signal, float *prevSignal, float threshold) {
    static float peakValue = 0.0f;
    static uint8_t peakState = 0; // 0:等待上升沿, 1:上升中, 2:下降中
    
    bool isPeak = false;
    
    if (peakState == 0) {
        // 等待上升沿
        if (signal > *prevSignal) {
            peakState = 1;
            peakValue = signal;
        }
    } else if (peakState == 1) {
        // 上升中
        if (signal > peakValue) {
            peakValue = signal;
        } else {
            peakState = 2;
        }
    } else if (peakState == 2) {
        // 下降中
        if (signal < threshold) {
            // 确认峰值
            isPeak = true;
            peakState = 0;
        }
    }
    
    *prevSignal = signal;
    return isPeak;
}

/**
 * 计算心率 - 基于脉搏间隔计算每分钟心跳次数
 * @param currentTime 当前采样时间戳（单位：采样周期数）
 * @param isPeak 标志位，表示是否检测到脉搏峰值
 * @return 计算得到的心率值（次/分钟）
 * @note 算法说明：
 *       - 存储最近5次有效脉搏间隔，计算平均值
 *       - 心率 = 60秒 / 平均间隔（秒/次）
 *       - 过滤异常间隔（心率范围40-220BPM）
 */
static uint8_t CalculateHeartRate(uint32_t currentTime, bool isPeak) {
    if (!isPeak) return currentHeartRate;
    
    uint32_t interval = currentTime - lastPulseTime;
    lastPulseTime = currentTime;
    
    // 检查间隔是否在合理范围内
    const uint32_t minInterval = BLOOD_SAMPLE_RATE * 60 / 220; // 最大心率220BPM
    const uint32_t maxInterval = BLOOD_SAMPLE_RATE * 60 / 40;  // 最小心率40BPM
    
    if (interval < minInterval || interval > maxInterval) {
        return currentHeartRate;
    }
    
    // 保存间隔
    pulseIntervals[intervalIndex] = interval;
    intervalIndex = (intervalIndex + 1) % 5;
    
    // 计算平均间隔
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
 * 计算血氧饱和度 - 基于Beer-Lambert定律的经验公式
 * @param redData 红光传感器数据数组
 * @param irData 红外光传感器数据数组
 * @param size 数据数组长度
 * @return 计算得到的血氧值（%）
 * @note 算法步骤：
 *       1. 提取信号中的AC（交流，脉搏波动）和DC（直流，基线）分量
 *       2. 计算R值 = (AC_red/DC_red) / (AC_ir/DC_ir)
 *       3. 通过经验公式 SpO2 = -45.06×R2 + 30.35×R + 94.85 计算血氧
 *       4. 结果限制在[BLOOD_MIN_SPO2, BLOOD_MAX_SPO2]范围内
 */
static uint8_t CalculateSpO2(uint32_t *redData, uint32_t *irData, uint8_t size) {
    // 寻找信号中的交流(AC)和直流(DC)分量
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
    
    // 计算交流和直流分量
    float redAC = (float)(redMax - redMin);
    float redDC = (float)redSum / size;
    float irAC = (float)(irMax - irMin);
    float irDC = (float)irSum / size;
    
    // 避免除零错误
    if (redDC <= 0 || irDC <= 0 || irAC <= 0) {
        return 0;
    }
    
    // 计算R值 = (ACred/DCred)/(ACir/DCir)
    float ratio = (redAC / redDC) / (irAC / irDC);
    
    // 根据经验公式计算SpO2
    // SpO2 = -45.060 * ratio^2 + 30.354 * ratio + 94.845
    float spo2 = -45.060f * ratio * ratio + 30.354f * ratio + 94.845f;
    
    // 限制在合理范围内
    if (spo2 > BLOOD_MAX_SPO2) spo2 = BLOOD_MAX_SPO2;
    if (spo2 < BLOOD_MIN_SPO2) spo2 = BLOOD_MIN_SPO2;
    
    return (uint8_t)spo2;
}

/**
 * 初始化血氧测量模块 - 启动传感器并重置内部状态
 * @return BLOOD_OK表示初始化成功，其他值表示错误
 * @note 初始化流程：
 *       1. 调用MAX30102_Init()初始化硬件传感器
 *       2. 调用Blood_Reset()清空历史数据和状态变量
 */
BloodError Blood_Init(void) {
    // 初始化MAX30102传感器
    MAX30102_Init();
    
    // 重置内部状态
    Blood_Reset();
    
    return BLOOD_OK;
}

/**
 * 更新传感器数据 - 处理原始数据并计算生理指标
 * @param red 红光传感器原始值
 * @param ir 红外光传感器原始值
 * @note 数据处理流程：
 *       1. 缓存原始数据
 *       2. 低通滤波平滑信号
 *       3. 高通滤波去除基线漂移
 *       4. 动态阈值检测脉搏峰值
 *       5. 周期性计算心率和血氧
 */
void Blood_Update(uint32_t red, uint32_t ir) {
    // 保存原始数据到缓冲区
    redBuffer[bufferIndex] = red;
    irBuffer[bufferIndex] = ir;
    bufferIndex = (bufferIndex + 1) % BLOOD_BUFFER_SIZE;
    
    // 低通滤波 - 平滑信号
    filteredRed = LowPassFilter((float)red, filteredRed, 0.2f);
    filteredIr = LowPassFilter((float)ir, filteredIr, 0.2f);
    
    // 高通滤波 - 去除基线漂移
    highPassRed = HighPassFilter(filteredRed, &prevRed, &prevRedHp, 0.95f);
    highPassIr = HighPassFilter(filteredIr, &prevIr, &prevIrHp, 0.95f);
    
    // 动态阈值检测脉搏峰值
    float threshold = 0.5f * fabs(highPassRed);
    bool isPeak = DetectPeak(highPassRed, &prevRedHp, threshold);
    
    // 更新心率
    sampleCount++;
    if (isPeak) {
        currentHeartRate = CalculateHeartRate(sampleCount, true);
        pulseDetected = true;
    }
    
    // 每100个样本计算一次血氧
    if (sampleCount % 100 == 0) {
        currentSpO2 = CalculateSpO2(redBuffer, irBuffer, BLOOD_BUFFER_SIZE);
    }
}

/**
 * 读取最新的血氧和心率数据 - 对外提供测量结果接口
 * @param result 存储测量结果的结构体指针
 * @return BLOOD_OK表示读取成功，其他值表示错误
 * @note 函数流程：
 *       1. 检查输入参数有效性
 *       2. 从MAX30102读取原始数据
 *       3. 调用Blood_Update()处理数据
 *       4. 填充结果结构体并返回
 */
BloodError Blood_Read(BloodResult *result) {
    if (result == NULL) {
        return BLOOD_ERROR_READ;
    }
    
    uint32_t red, ir;
    MAX30102_ReadFIFO(&red, &ir);
    
    // 更新传感器数据
    Blood_Update(red, ir);
    
    // 填充结果
    result->spo2 = currentSpO2;
    result->heartRate = currentHeartRate;
    result->pulseDetected = pulseDetected;
    result->timestamp = sampleCount;
    
    // 重置脉搏检测标志
    pulseDetected = false;
    
    return BLOOD_OK;
}

/**
 * 重置测量状态 - 清空历史数据和状态变量
 * @note 重置内容：
 *       1. 缓冲区索引和采样计数
 *       2. 脉搏间隔记录和时间戳
 *       3. 滤波中间变量和状态机参数
 *       4. 计算得到的血氧和心率值
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
