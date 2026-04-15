#include "ppm.h"

uint16_t ppmValues[NUM_CHANNELS] = {0};
int filteredPPMValues[NUM_CHANNELS] = {0};
uint32_t lastTime = 0;
uint8_t currentChannel = 0;

uint8_t enableHubMotor = 0;
PPM_KNOB_MODE knobMode;      // 左旋钮模式控制
JUMPING_PROCESS jumpProcess; // 跳跃过程
float alpha = 0.02;          // 设置滤波系数（值越小，平滑度越高）;

// 遥控器PPM读取初始化
void ppm_init()
{
    pinMode(PPM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PPM_PIN), onPPMInterrupt, RISING); // 开启ppm读取中断
}

// 遥控器低通滤波接口
void storeFilteredPPMData()
{
    filteredPPMValues[0] = lowPassFilter(ppmValues[0], filteredPPMValues[0], alpha); // 右摇杆左右 左1000 右2000
    filteredPPMValues[1] = lowPassFilter(ppmValues[1], filteredPPMValues[1], alpha); // 右摇杆上下 下1000 上2000
    filteredPPMValues[2] = lowPassFilter(ppmValues[2], filteredPPMValues[2], alpha); // 左摇杆上下 下1000 上2000
    filteredPPMValues[3] = lowPassFilter(ppmValues[3], filteredPPMValues[3], alpha); // 左摇杆左右 左1000 右2000
    filteredPPMValues[4] = lowPassFilter(ppmValues[4], filteredPPMValues[4], alpha); // 左旋钮 左1000 右2000
    filteredPPMValues[5] = lowPassFilter(ppmValues[5], filteredPPMValues[5], alpha); // 右旋钮 左2000 右1000
    filteredPPMValues[6] = ppmValues[6];                                             // 左拨杆 下1890+ 上1090-
    filteredPPMValues[7] = ppmValues[7];                                             // 右拨杆 下1860+ 中14xx 上1060-
    // Serial.printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
    //               filteredPPMValues[0],
    //               filteredPPMValues[1],
    //               filteredPPMValues[2],
    //               filteredPPMValues[3],
    //               filteredPPMValues[4],
    //               filteredPPMValues[5],
    //               filteredPPMValues[6],
    //               filteredPPMValues[7]);
}

void IRAM_ATTR onPPMInterrupt()
{
    uint32_t now = micros();
    uint32_t duration = now - lastTime;
    lastTime = now;

    if (duration >= SYNC_GAP)
    {
        // Sync pulse detected, reset channel index
        currentChannel = 0;
    }
    else if (currentChannel < NUM_CHANNELS)
    {
        ppmValues[currentChannel] = duration;
        currentChannel++;
    }
}

PPM_SWITCH_POS getSwitchPos(int switchValue)
{
    if (switchValue < 1200)
        return PPM_SWITCH_POS::UP;
    else if (switchValue >= 1200 && switchValue < 1600)
        return PPM_SWITCH_POS::MIDDLE;
    else
        return PPM_SWITCH_POS::DOWN;
}

void remoteSwitch()
{
    /***** 右拨杆 *****/
    switch (getSwitchPos(PPM_SWITCH_RIGHT))
    {
    case PPM_SWITCH_POS::UP:
        enableHubMotor = 0; // 失能轮毂电机
        break;
    case PPM_SWITCH_POS::MIDDLE:
        enableHubMotor = 1;
        break;
    case PPM_SWITCH_POS::DOWN:
        enableHubMotor = 1;
        break;
    default:
        break;
    }
    return; /////////////////////////////////////////////////////
    /***** 左拨杆 *****/
    // switch (getSwitchPos(PPM_SWITCH_LEFT))
    // {
    // case PPM_SWITCH_POS::UP:
    //     break;
    // case PPM_SWITCH_POS::DOWN:
    //     break;
    // default:
    //     break;
    // }

    static auto lastSwitchLeft = getSwitchPos(PPM_SWITCH_LEFT);
    auto switchLeft = getSwitchPos(PPM_SWITCH_LEFT);

    /***** 左旋钮 *****/
    if (PPM_LEFT_KNOB < 1200) // 左旋钮逆时针到顶 跳跃
    {
        switch (switchLeft)
        {
        case PPM_SWITCH_POS::UP:
            knobMode = PPM_KNOB_MODE::Normal;
            jumpProcess = JUMPING_PROCESS::LAND;
            break;
        case PPM_SWITCH_POS::DOWN:
            if (lastSwitchLeft == PPM_SWITCH_POS::UP) // 左拨杆刚从上打到下时才开启跳跃模式
            {
                knobMode = PPM_KNOB_MODE::Jump;
                jumpProcess = JUMPING_PROCESS::READY;
            }
            break;
        default:
            break;
        }
    }
    else if (PPM_LEFT_KNOB > 1800) // 左旋钮顺时针到顶 抖肩
    {
        switch (switchLeft)
        {
        case PPM_SWITCH_POS::UP:
            knobMode = PPM_KNOB_MODE::Normal;
            break;
        case PPM_SWITCH_POS::DOWN:
            knobMode = PPM_KNOB_MODE::ShakeShoulder;
            break;
        default:
            break;
        }
    }
    else
    {
        knobMode = PPM_KNOB_MODE::Normal;
    }
    lastSwitchLeft = switchLeft;
}