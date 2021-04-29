/**
 * @file key.cpp
 * @author 彭阳
 * @brief 
 * @version 2.3.0
 * @date 2020-12-22
 * 
 */

#include "key.h"

Key* Key::currentKey = NULL;
TimerHandle_t Key::timer = NULL;
Key* Key::finalKey = NULL;

/**
 * @brief 判断端口位数
 * 
 * @param port Stm32端口
 * @return uint32_t 端口最高位位置
 * @note 静态函数
 */
static uint32_t judge_BitSite(uint16_t port)
{
    return port == 0x00 ? 0x00 : judge_BitSite(port >> 1) + 1;
}

/**
 * @brief Construct a new Key:: Key object
 * 
 * @param GPIOx 按键端口
 * @param GPIO_Pin_x 按键引脚
 * @param timeThreshold 长按时间阈值
 * @param longPressSpeed 长按增长频率（长按视为一秒按下几次)
 */
Key::Key(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x, uint32_t timeThreshold, uint8_t longPressSpeed) : GPIOx(GPIOx),
                                                                                                     GPIO_Pin_x(GPIO_Pin_x),
                                                                                                     time(new Cycle()),
                                                                                                     timeThreshold(timeThreshold),
                                                                                                     longPressSpeed(longPressSpeed),
                                                                                                     pressTime(0),
                                                                                                     status(0)
{
    if (currentKey == NULL)
    {
        this->lastkey = this;
    }
    else
    {
        this->lastkey = currentKey;
    }
    finalKey = this;
    currentKey = this;
}

/**
 * @brief 单个按键初始化
 * 
 */
void Key::init()
{
    uint8_t pin = judge_BitSite(GPIO_Pin_x) - 1;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(0x01 << (judge_BitSite(((uint32_t)GPIOx - AHB1PERIPH_BASE) >> 2) >> 2), ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; //普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; //开漏输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
    GPIO_Init(GPIOx, &GPIO_InitStructure);
}

/**
 * @brief 全部按键初始化
 * @warning 不要在任务调度器没开启时调用
 */
void Key::keyInit()
{
    if (timer == NULL)
    {
        //第一次调用构造器时创建软件定时器
        timer = xTimerCreate((const char *)"AutoReloadTimer",
                             (TickType_t)10,
                             (UBaseType_t)pdTRUE,
                             (void *)1,
                             (TimerCallbackFunction_t)(keyScan));
        xTimerStop(timer, portMAX_DELAY);
    }

    //遍历全部按键
    for (currentKey = finalKey; currentKey != currentKey->lastkey; currentKey = currentKey->lastkey)
    {
        currentKey->init();
    }
    currentKey->init();

    //开启软件定时器
    xTimerStart(timer, portMAX_DELAY);
}

/**
 * @brief 扫描全部按键
 * @note 此函数会自动调用，频率100Hz，也可以自行手动调用
 */
void Key::keyScan()
{
    //遍历全部按键
    for (currentKey = finalKey; currentKey != currentKey->lastkey; currentKey = currentKey->lastkey)
    {
        currentKey->scan();
    }
    currentKey->scan();
}

/**
 * @brief 更新单个按键状态
 * 
 */
void Key::scan()
{
    realStatus = GPIO_ReadInputDataBit(GPIOx, GPIO_Pin_x); //获取按键状态

    if (realStatus == 0) //按下
    {
        pressTime += (uint32_t)(time->getCycleT() * 1000); //按下以后开始记录时间
        if (pressTime > timeThreshold)                     //大于阈值时开始计算长按
        {
            if ((pressTime - timeThreshold) > 1000 / longPressSpeed)
            {
                pressTime -= 1000 / longPressSpeed;
                status = 1; //按了一次
            }
        }
    }
    else
    {
        if ((pressTime > 0) && (pressTime < timeThreshold - 1000 / longPressSpeed)) //不被视为长按
        {
            status = 1; //按了一次
        }
        time->getCycleT();
        pressTime = 0; //清空计时
    }
}

/**
 * @brief 获取真实按键状态
 * 
 */
uint8_t Key::getRealStatus()
{
    return this->realStatus;
}
