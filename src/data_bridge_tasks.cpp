#include "tasks/data_bridge_tasks.h"

// 陀螺仪数据读取
void IMUTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1); // 将 1ms 转换为 Tick 数 (即 1 个 Tick)
    xLastWakeTime = xTaskGetTickCount();         // 初始化“上次唤醒时间”为当前时间

    while (true)
    {
        mpu6050.update(false);

        imuStatePackage imuPacket;

        imuPacket.ax = mpu6050.getAccX();
        imuPacket.ay = mpu6050.getAccY();
        imuPacket.az = mpu6050.getAccZ();
        imuPacket.gx = mpu6050.getGyroX();
        imuPacket.gy = mpu6050.getGyroY();
        imuPacket.gz = mpu6050.getGyroZ();
        Append_CRC16_Check_Sum((uint8_t *)&imuPacket, sizeof(imuStatePackage)); // 计算 CRC

        // --- 串口写入保护 ---
        // 获取互斥锁 (如果另一个任务正在使用串口，此任务会在此处阻塞)
        if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
        {
            // 只有拿到锁的任务才能执行 Serial.write
            Serial.write((uint8_t *)&imuPacket, sizeof(imuStatePackage)); // 发送数据
            // 发送完毕后，立刻释放互斥锁
            xSemaphoreGive(xSerialMutex);
        }
        // --- 串口写入保护结束 ---

        // static auto lastTime = micros();
        // auto currentTime = micros();
        // auto dt = (currentTime - lastTime) * 1.0e-6f;
        // lastTime = currentTime;
        // Serial.printf("currentTime:%d\tdt:%.3fms\tfreq:%.3f\n", currentTime, dt * 1000.f, 1.0f / dt);

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

void canRecTask(void *pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(1); // 1ms = 1 个 Tick)
    xLastWakeTime = xTaskGetTickCount();           // 初始化“上次唤醒时间”为当前时间
    while (true)
    {
        uint8_t motorID = recCANMessage();
        if (motorID != 0xFF)
        {
            uint8_t index = motorID - 1;
            motorStatePackage motorPacket;
            motorPacket.motorID = motorID;
            motorPacket.motorPos = devicesState[index].pos;
            motorPacket.motorVel = devicesState[index].vel;
            motorPacket.motorTor = devicesState[index].tor;

            Append_CRC16_Check_Sum((uint8_t *)&motorPacket, sizeof(motorStatePackage)); // 计算 CRC
            // 获取互斥锁
            if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
            {
                Serial.write((uint8_t *)&motorPacket, sizeof(motorStatePackage)); // 发送数据
                // 释放互斥锁
                xSemaphoreGive(xSerialMutex);
            }
            vTaskDelayUntil(&xLastWakeTime, xPeriod);
        }
    }
}

// 如果不想用 ESP-IDF 专用 API，可以用一个简单的循环数组
static uint8_t rxRingBuffer[SERIAL_RING_BUFFER_SIZE];
static volatile size_t rxHead = 0;
static volatile size_t rxTail = 0;
static size_t rxCount = 0;

// 向环形缓冲区写入一个字节（生产者：串口接收）
void ringBufferWrite(uint8_t data)
{
    size_t nextHead = (rxHead + 1) % SERIAL_RING_BUFFER_SIZE;
    if (nextHead != rxTail)  // 未满
    {
        rxRingBuffer[rxHead] = data;
        rxHead = nextHead;
    }
    else  // 满了，丢弃旧数据（保留新数据）
    {
        rxRingBuffer[rxHead] = data;
        rxHead = nextHead;
        rxTail = (rxTail + 1) % SERIAL_RING_BUFFER_SIZE;  // 移动尾指针，丢弃最旧的数据
    }
}

// 从环形缓冲区读取一个字节（消费者：解析任务），返回读取成功与否
bool ringBufferRead(uint8_t *data)
{
    if (rxHead == rxTail)
        return false;
    *data = rxRingBuffer[rxTail];
    rxTail = (rxTail + 1) % SERIAL_RING_BUFFER_SIZE;
    return true;
}

// 接收任务
void serialRecTask(void *pvParameters)
{
    uint8_t byte;
    const int MAX_PACKETS_PER_LOOP = 5; // 单次循环最多处理 5 个完整包，防止占 CPU 过久

    while (true)
    {
        // 1. 将串口硬件 FIFO 中的数据快速转移到环形缓冲区
        while (Serial.available() > 0)
        {
            byte = Serial.read();
            ringBufferWrite(byte);
        }

        // 2. 解析环形缓冲区中的数据，但限制处理量
        static enum { WAIT_HEADER,
                      READ_PACKET } state = WAIT_HEADER;
        static uint8_t packetBuffer[SERIAL_PACKET_SIZE];
        static size_t packetIndex = 0;
        int packetsProcessed = 0;

        while (ringBufferRead(&byte) && packetsProcessed < MAX_PACKETS_PER_LOOP)
        {
            switch (state)
            {
            case WAIT_HEADER:
                if (byte == 0xA5)
                {
                    packetBuffer[0] = byte;
                    packetIndex = 1;
                    state = READ_PACKET;
                }
                break;
            case READ_PACKET:
                packetBuffer[packetIndex++] = byte;
                if (packetIndex == SERIAL_PACKET_SIZE)
                {
                    SerialCommandPackage *pkt = (SerialCommandPackage *)packetBuffer;
                    if (Verify_CRC16_Check_Sum((uint8_t *)pkt, SERIAL_PACKET_SIZE))
                    {
                        lastRecCmdTime = millis();
                        if (xSemaphoreTake(xCommandMutex, 0) == pdTRUE)
                        {
                            latestCommand = *pkt;
                            xSemaphoreGive(xCommandMutex);
                        }
                    }
                    state = WAIT_HEADER;
                    ++packetsProcessed;
                }
                break;
            }
        }

        // 3. 主动让出 CPU，避免看门狗复位
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
