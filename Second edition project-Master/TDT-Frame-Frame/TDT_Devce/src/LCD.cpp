/******************************
 @file TDT_Devce\src\LCD.cpp
 @brief  LCD屏幕驱动
 @author 彭阳
 @version 2.3.0
 @date 20.12.19
 @history:
	——————————————————————————————————————————————————————————————————————————
	20.12.19 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************/
#include "LCD.h"
#include "font.h"
#include "FreeRTOS.h"
#include "task.h"
#include <Stdarg.h>
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
 * @brief Construct a new LCD::LCD object   
 * 
 * @param dcport 数据线端口
 * @param dcPin 数据线引脚
 * @param resetPort 复位线端口
 * @param resetPin 复位线引脚
 * @param spix Spi端口
 */
LCD::LCD(GPIO_TypeDef *dcPort, uint16_t dcPin, GPIO_TypeDef *resetPort, uint16_t resetPin, SPI_TypeDef *spix) : Spi(spix, 0),
                                                                                                                dc(Pin{dcPort, dcPin}),
                                                                                                                reset(Pin{resetPort, resetPin}),
                                                                                                                direction(Lcd::down),
                                                                                                                fontColor(Lcd::BLACK),
                                                                                                                backColor(Lcd::WHITE)
{
}

/**
 * @brief lcd数据线拉高
 * 
 * @note 内联函数
 */
void LCD ::lcdDcSet() { dc.port->BSRRL = dc.pin; }

/**
 * @brief lcd数据线拉低
 * 
 * @note 内联函数
 */
void LCD ::lcdDcClear() { dc.port->BSRRH = dc.pin; }

/** 
 * @brief lcd复位线拉高
 * 
 * @note 内联函数
 */
void LCD ::lcdResetSet() { reset.port->BSRRL = reset.pin; }

/**
 * @brief lcd复位线拉低
 * 
 * @note 内联函数
 */
void LCD::lcdResetClear() { reset.port->BSRRH = reset.pin; }

/**
 * @brief lcd数据线和复位线引脚初始化
 * 
 */
void LCD::lcdPinInit(void)
{
    RCC_AHB1PeriphClockCmd(((0x01 << (((judge_BitSite((uint32_t)dc.port - AHB1PERIPH_BASE)) >> 2))) >> 1) | ((0x01 << (((judge_BitSite((uint32_t)reset.port - AHB1PERIPH_BASE)) >> 2))) >> 1), ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = dc.pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT; //普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   //上拉
    GPIO_Init(dc.port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = reset.pin;
    GPIO_Init(reset.port, &GPIO_InitStructure);
}

/**
 * @brief LCD屏幕初始化
 * 
 */
void LCD::lcdInit(void)
{
    init();       //SPI初始化
    lcdPinInit(); //lcd数据线和复位线引脚初始化
    lcdReset();   //LCD 复位

    //LCD屏初始化开始
    lcdSendCommand(0xCF); //功耗控制
    lcdSendData(0x00);    //默认为0x00
    lcdSendData(0xF9);    //vgh=6vcl vgl=-3vcl 亮度最高
    lcdSendData(0X30);    //启用保护电路
    lcdSendCommand(0xED); //开机顺序
    lcdSendData(0x64);
    lcdSendData(0x03);
    lcdSendData(0X12);
    lcdSendData(0X81);
    lcdSendCommand(0xE8); //定时控制
    lcdSendData(0x85);
    lcdSendData(0x10);
    lcdSendData(0x7A);
    lcdSendCommand(0xCB); //功率控制A
    lcdSendData(0x39);
    lcdSendData(0x2C);
    lcdSendData(0x00);
    lcdSendData(0x34);
    lcdSendData(0x02);
    lcdSendCommand(0xF7); //泵比控制
    lcdSendData(0x20);
    lcdSendCommand(0xEA); //驱动时序控制B
    lcdSendData(0x00);
    lcdSendData(0x00);
    lcdSendCommand(0xC0); //功率控制0
    lcdSendData(0x1B);    //VRH[5:0]
    lcdSendCommand(0xC1); //功率控制1
    lcdSendData(0x12);    //SAP[2:0];BT[3:0] //0x01
    lcdSendCommand(0xC5); //VCOM控制1
    lcdSendData(0x26);    //3F
    lcdSendData(0x26);    //3C
    lcdSendCommand(0xC7); //VCOM控制2
    lcdSendData(0XB0);
    lcdSendCommand(0x36); //屏幕方向控制
    lcdSendData(0x08);
    lcdSendCommand(0x3A); //像素格式控制
    lcdSendData(0x55);
    lcdSendCommand(0xB1);
    lcdSendData(0x00);
    lcdSendData(0x1A);
    lcdSendCommand(0xB6); // Display Function Control
    lcdSendData(0x0A);
    lcdSendData(0xA2);
    lcdSendCommand(0xF2); //使能伽马
    lcdSendData(0x00);
    lcdSendCommand(0x26); //设定伽马值
    lcdSendData(0x01);
    lcdSendCommand(0xE0); //正极伽马校准
    lcdSendData(0x1F);
    lcdSendData(0x24);
    lcdSendData(0x24);
    lcdSendData(0x0D);
    lcdSendData(0x12);
    lcdSendData(0x09);
    lcdSendData(0x52);
    lcdSendData(0xB7);
    lcdSendData(0x3F);
    lcdSendData(0x0C);
    lcdSendData(0x15);
    lcdSendData(0x06);
    lcdSendData(0x0E);
    lcdSendData(0x08);
    lcdSendData(0x00);
    lcdSendCommand(0XE1); //负极伽马校准
    lcdSendData(0x00);
    lcdSendData(0x1B);
    lcdSendData(0x1B);
    lcdSendData(0x02);
    lcdSendData(0x0E);
    lcdSendData(0x06);
    lcdSendData(0x2E);
    lcdSendData(0x48);
    lcdSendData(0x3F);
    lcdSendData(0x03);
    lcdSendData(0x0A);
    lcdSendData(0x09);
    lcdSendData(0x31);
    lcdSendData(0x37);
    lcdSendData(0x1F);
    lcdSendCommand(0x2B); //设定y轴显示地址
    lcdSendData(0x00);
    lcdSendData(0x00);
    lcdSendData(0x01);
    lcdSendData(0x3f);
    lcdSendCommand(0x2A); //设定x轴显示地址
    lcdSendData(0x00);
    lcdSendData(0x00);
    lcdSendData(0x00);
    lcdSendData(0xef);
    lcdSendCommand(0x11); //退出睡眠模式
    //LCD屏初始化结束

    vTaskDelay(pdMS_TO_TICKS(120));
    lcdSendCommand(0x29); //打开显示屏

    lcdSetDirection(direction); //设置LCD显示方向
    lcdClear(Lcd::WHITE);       //清全屏白色
}

/**
 * @brief 对spi发送函数进行二次封装
 * 
 * @param data 要发送的8位数据
 */
void LCD::lcdSendByte(uint8_t data)
{
    readWriteByte((uint16_t)data);
}

/**
 * @brief 复位lcd屏
 * 
 * @note 此函数会调用vTaskDelay导致任务延时
 */
void LCD::lcdReset(void)
{
    lcdResetClear();
    vTaskDelay(pdMS_TO_TICKS(100));
    lcdResetSet();
    vTaskDelay(pdMS_TO_TICKS(50));
}

/**
 * @brief 发送命令
 * 
 * @param data 要发送的8位命令
 */
void LCD::lcdSendCommand(u8 data)
{
    lcdDcClear(); //数据线拉低
    lcdSendByte(data);
}

/**
 * @brief 发送数据
 * 
 * @param data 要发送的8位数据
 */
void LCD::lcdSendData(u8 data)
{
    lcdDcSet(); //数据线拉高
    lcdSendByte(data);
}

/**
 * @brief 发送16位数据
 * 
 * @param data 要发送的16位数据
 */
void LCD::lcdSendData16Bit(uint16_t data)
{
    lcdDcSet();
    lcdSendByte(data >> 8);
    lcdSendByte(data);
}

/**
 * @brief 开始写入缓存
 * 
 * @note 发送任何其他命令会结束写入缓存状态
 */
void LCD::lcdWriteRAMPrepare(void)
{
    lcdSendCommand(Lcd::Command::writeRAM);
}

/**
 * @brief 设置屏幕方向
 * 
 * @param direction 屏幕方向
 */
void LCD::lcdSetDirection(enum Lcd::lcdDirection direction)
{
    using namespace Lcd;
    this->direction = direction;
    switch (direction)
    {
    case up:
        width = LcdWidth;
        height = LcdHight;
        lcdSendCommand(0x36);
        lcdSendData((1 << 3) | (0 << 5) | (0 << 6) | (0 << 7));
        break;
    case right:
        width = LcdHight;
        height = LcdWidth;
        lcdSendCommand(0x36);
        lcdSendData((1 << 3) | (1 << 5) | (1 << 6) | (0 << 7));
        break;
    case down:
        width = LcdWidth;
        height = LcdHight;
        lcdSendCommand(0x36);
        lcdSendData((1 << 3) | (0 << 5) | (1 << 6) | (1 << 7));
        break;
    case left:
        width = LcdHight;
        height = LcdWidth;
        lcdSendCommand(0x36);
        lcdSendData((1 << 3) | (1 << 5) | (0 << 6) | (1 << 7));
        break;
    default:
        break;
    }
}

/**
 * @brief 设定lcd屏幕绘制区域
 * 
 * 使用此函数设定范围以后范围之外的区域显示内容不会改变
 * @param xStart x轴起始地址
 * @param yStart y轴起始地址
 * @param xEnd x轴结束地址
 * @param yEnd y轴结束地址
 */
void LCD::lcdSetWindows(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    lcdSendCommand(Lcd::Command::setX);
    lcdSendData16Bit(xStart);
    lcdSendData16Bit(xEnd);

    lcdSendCommand(Lcd::Command::setY);
    lcdSendData16Bit(yStart);
    lcdSendData16Bit(yEnd);

    lcdWriteRAMPrepare(); //开始写入GRAM
}

/**
 * @brief 设定光标
 * 
 * 设定后仅会改变光标处的像素点
 * @param xPos x轴坐标
 * @param yPos y轴坐标
 */
void LCD::lcdSetCursor(uint16_t xPos, uint16_t yPos)
{
    lcdSetWindows(xPos, yPos, xPos, yPos);
}

/**
 * @brief 设置显示字符串颜色
 * 
 * @param fontColor 字体颜色
 * @param backColor 背景颜色
 * @note 仅对lcdPrintf有影响
 */
void LCD::lcdSetColor(enum Lcd::Color fontColor, enum Lcd::Color backColor)
{
    this->fontColor = fontColor;
    this->backColor = backColor;
}

/**
 * @brief 清屏
 * 
 * @param color 清屏使用的颜色
 */
void LCD::lcdClear(enum Lcd::Color color)
{
    using namespace Lcd;
    lcdSetWindows(0, 0, width - 1, height - 1);
    lcdDcSet();
    for (uint32_t i = 0; i < height; i++)
    {
        for (uint32_t j = 0; j < width; j++)
        {
            lcdSendData16Bit((uint16_t)color);
        }
    }
}

/**
 * @brief 画一个点
 * 
 * @param x x轴坐标
 * @param y y轴坐标
 * @param color 绘制颜色
 */
void LCD::lcdDrawPoint(uint16_t x, uint16_t y, enum Lcd::Color color)
{
    lcdSetCursor(x, y);
    lcdSendData16Bit((uint16_t)color);
}

/**
 * @brief 填充区域
 * 
 * @param xStart x轴起始坐标
 * @param yStart y轴起始坐标
 * @param xEnd x轴结束坐标
 * @param yEnd y轴结束坐标
 * @param color 绘制颜色
 */
void LCD::lcdDrawFill(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color)
{
    uint16_t width = xEnd - xStart;
    uint16_t height = yEnd - yStart;
    lcdSetWindows(xStart, yStart, xEnd, yEnd);
    for (uint16_t i = 0; i <= height; i++)
    {
        for (uint16_t j = 0; j <= width; j++)
        {
            lcdSendData16Bit((uint16_t)color);
        }
    }
    lcdSetWindows(0, 0, width - 1, height - 1);
}

/**
 * @brief 画一条线
 * 
 * @param xStart x轴起始坐标
 * @param yStart y轴起始坐标
 * @param xEnd x轴结束坐标
 * @param yEnd y轴结束坐标
 * @param color 绘制颜色
 */
void LCD::lcdDrawLine(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color)
{
    int16_t delatX = xEnd - xStart;
    int16_t delatY = yEnd - yStart;
    int16_t xdistance, ydistance, distance, row = xStart, col = yStart, x = 0, y = 0;
    xdistance = delatX > 0 ? 1 : delatX == 0 ? 0 : (delatX *= -1, -1);
    ydistance = delatY > 0 ? 1 : delatY == 0 ? 0 : (delatY *= -1, -1);
    distance = delatX > delatY ? delatX : delatY;
    for (uint16_t i = 0; i < distance; i++)
    {
        lcdDrawPoint(row, col, color);
        x += delatX;
        y += delatY;
        if (x > distance)
        {
            x -= distance;
            row += xdistance;
        }
        if (y > distance)
        {
            y -= distance;
            col += ydistance;
        }
    }
}

/**
 * @brief 绘制空心矩形
 * 
 * @param xStart x轴起始坐标
 * @param yStart y轴起始坐标
 * @param xEnd x轴结束坐标
 * @param yEnd y轴结束坐标
 * @param color 绘制颜色
 */
void LCD::lcdDrawRectangle(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color)
{
    lcdDrawLine(xStart, yStart, xStart, yEnd, color);
    lcdDrawLine(xStart, yEnd, xEnd, yEnd, color);
    lcdDrawLine(xStart, yStart, xEnd, xStart, color);
    lcdDrawLine(xEnd, yStart, xEnd, yEnd, color);
}

/**
 * @brief 绘制实心矩形
 * 
 * @param xStart x轴起始坐标
 * @param yStart y轴起始坐标
 * @param xEnd x轴结束坐标
 * @param yEnd y轴结束坐标
 * @param color 绘制颜色
 */
void LCD::lcdDrawFillRectangle(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color)
{
    lcdDrawFill(xStart, yStart, xEnd, yEnd, color);
}

/**
 * @brief 8点对称画圆算法
 * 
 * @param xc 圆心x轴坐标
 * @param yc 圆心y轴坐标
 * @param x 相对于圆心x坐标
 * @param y 相对于圆心y坐标
 * @param color 绘制颜色
 */
void LCD::lcdDrawCircle8(int xc, int yc, int x, int y, enum Lcd::Color color)
{
    lcdDrawPoint(xc + x, yc + y, color);
    lcdDrawPoint(xc - x, yc + y, color);
    lcdDrawPoint(xc + x, yc - y, color);
    lcdDrawPoint(xc - x, yc - y, color);
    lcdDrawPoint(xc + y, yc + x, color);
    lcdDrawPoint(xc - y, yc + x, color);
    lcdDrawPoint(xc + y, yc - x, color);
    lcdDrawPoint(xc - y, yc - x, color);
}

/**
 * @brief 画圆
 * 
 * @param x 圆心x轴坐标
 * @param y 圆心y轴坐标
 * @param r 半径
 * @param fill 是否填充 1填充 0不填充
 * @param color 绘制颜色
 */
void LCD::lcdDrawCircle(uint16_t x, uint16_t y, uint16_t r, uint8_t fill, enum Lcd::Color color)
{
    int xi = 0, yi = r, d = 3 - 2 * r;

    if (fill)
    {
        // 如果填充（画实心圆）
        while (xi <= yi)
        {
            for (uint16_t i = xi; i < yi; i++)
            {
                lcdDrawCircle8(x, y, xi, i, color);
            }
            if (d < 0)
            {
                d = d + 4 * xi + 6;
            }
            else
            {
                d = d + 4 * (xi - yi) + 10;
                yi--;
            }
            xi++;
        }
    }
    else
    {
        // 如果不填充（画空心圆）
        while (xi <= yi)
        {
            lcdDrawCircle8(x, y, xi, yi, color);
            if (d < 0)
            {
                d = d + 4 * xi + 6;
            }
            else
            {
                d = d + 4 * (xi - yi) + 10;
                yi--;
            }
            xi++;
        }
    }
}

/**
 * @brief 交换
 * 
 * @param a 要交换的数
 * @param b 要交换的数
 */
static void _swap(u16 *a, u16 *b)
{
    u16 tmp;
    tmp = *a;
    *a = *b;
    *b = tmp;
}

/**
 * @brief 绘制空心三角形
 * 
 * @param x0 第一个点的x坐标
 * @param y0 第一个点的y坐标
 * @param x1 第二个点的x坐标
 * @param y1 第二个点的y坐标
 * @param x2 第三个点的x坐标
 * @param y2 第三个点的y坐标
 * @param color 绘制颜色
 */
void LCD::lcdDrawTriangel(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, enum Lcd::Color color)
{
    lcdDrawLine(x1, y1, x2, y2, color);
    lcdDrawLine(x2, y2, x0, y0, color);
    lcdDrawLine(x0, y0, x1, y1, color);
}

/**
 * @brief 绘制实心三角形
 * 
 * @param x0 第一个点的x坐标
 * @param y0 第一个点的y坐标
 * @param x1 第二个点的x坐标
 * @param y1 第二个点的y坐标
 * @param x2 第三个点的x坐标
 * @param y2 第三个点的y坐标
 * @param color 绘制颜色
 */
void LCD::lcdDrawFillTriangel(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, enum Lcd::Color color)
{
    u16 a, b, y;
    int dx01, dy01, dx02, dy02, dx12, dy12, last;
    long sa = 0;
    long sb = 0;
    if (y0 > y1)
    {
        _swap(&y0, &y1);
        _swap(&x0, &x1);
    }
    if (y1 > y2)
    {
        _swap(&y2, &y1);
        _swap(&x2, &x1);
    }
    if (y0 > y1)
    {
        _swap(&y0, &y1);
        _swap(&x0, &x1);
    }
    if (y0 == y2)
    {
        a = b = x0;
        if (x1 < a)
        {
            a = x1;
        }
        else if (x1 > b)
        {
            b = x1;
        }
        if (x2 < a)
        {
            a = x2;
        }
        else if (x2 > b)
        {
            b = x2;
        }
        lcdDrawFill(a, y0, b, y0, color);
        return;
    }
    dx01 = x1 - x0;
    dy01 = y1 - y0;
    dx02 = x2 - x0;
    dy02 = y2 - y0;
    dx12 = x2 - x1;
    dy12 = y2 - y1;

    if (y1 == y2)
    {
        last = y1;
    }
    else
    {
        last = y1 - 1;
    }
    for (y = y0; y <= last; y++)
    {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        if (a > b)
        {
            _swap(&a, &b);
        }
        lcdDrawFill(a, y, b, y, color);
    }
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++)
    {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        if (a > b)
        {
            _swap(&a, &b);
        }
        lcdDrawFill(a, y, b, y, color);
    }
}

/**
 * @brief 显示字符
 * 
 * @param x x轴坐标
 * @param y y轴坐标
 * @param ch 要显示的字符
 * @param size 字号
 * @param fontColor 字体颜色 
 * @param backColor 背景颜色
 * @param mode 绘制模式 0-覆盖模式 1-叠加模式
 * @note 叠加模式不会改变背景，可能导致改变显示时多个字重叠
 */
void LCD::lcdShowChar(uint16_t x, uint16_t y, char ch, enum Lcd::fontLibrary size, enum Lcd::Color fontColor, enum Lcd::Color backColor, uint8_t mode)
{
    using namespace Lcd;
    uint8_t temp, ignore = (size / 2) % 8, xrow = (size + ignore * 2) / 16;
    uint16_t x0 = x;
    uint16_t bitpos = 0, overbit = (x + size / 2) > width ? x + size / 2 > width : 0;

    ch = ch - ' ';                                       //得到偏移后的值
    lcdSetWindows(x, y, x + size / 2 - 1, y + size - 1); //设置单个文字显示窗口

    for (uint8_t pos = 0; pos < size * xrow; pos++)
    {
        if (pos / xrow + 1 + y > height)
        {
            return;
        }
        if (size == ascii3216)
            temp = asc2_3216[ch][pos]; //调用1206字体
        else if (size == ascii2412)
            temp = asc2_2412[ch][pos]; //调用1608字体
        else
            temp = asc2_1608[ch][pos];
        for (uint8_t t = 0; t < 8; t++)
        {

            if (!mode) //非叠加方式
            {
                if (overbit != 0)
                {
                    bitpos = ((pos % xrow) * 8 + t + 1);
                    if (x + bitpos > width)
                        break;
                }
                if (pos % xrow == xrow - 1 && t >= 8 - ignore)
                    break;
                if (temp & 0x01)
                {
                    lcdSendData16Bit((uint16_t)fontColor);
                }
                else
                {
                    lcdSendData16Bit((uint16_t)backColor);
                }
                temp >>= 1;
            }
            else //叠加方式
            {
                if (temp & 0x01)
                    lcdDrawPoint(x, y, fontColor); //画一个点
                temp >>= 1;
                x++;
                if (x - x0 == size / 2)
                {
                    x = x0;
                    y++;
                    break;
                }
            }
        }
    }
    lcdSetWindows(0, 0, width - 1, height - 1); //恢复窗口为全屏
}

#if CH_USE

/**
 * @brief 显示中文汉字
 * 
 * @param x x轴坐标
 * @param y y轴坐标
 * @param ch 要显示的汉字
 * @param size 字号
 * @param fontColor 字体颜色 
 * @param backColor 背景颜色
 * @param mode 绘制模式 0-覆盖模式 1-叠加模式
 * @note 叠加模式不会改变背景，可能导致改变显示时多个字重叠
 * @warning 只能显示字库中存在的汉字
 */
void LCD::lcdShowCH(uint16_t x, uint16_t y, char *ch, enum Lcd::fontLibrary size, enum Lcd::Color fontColor, enum Lcd::Color backColor, uint8_t mode)
{
    using namespace Lcd;
    uint8_t temp;
    int16_t addr = -1, x0 = x;
    uint16_t bitpos = 0, overbit = (x + size) > width ? x + size > width : 0;

    if (*ch <= 127)
        return;
    for (uint16_t i = 0; chFontIndex[i] != '\0'; i += 2)
    {
        if (chFontIndex[i] == *ch && chFontIndex[i + 1] == *(ch + 1))
        {
            addr = i / 2;
            break;
        }
    }
    lcdSetWindows(x, y, x + size - 1, y + size - 1); //设置单个文字显示窗口
    for (uint16_t pos = 0; pos < size * size / 8; pos++)
    {
        if (pos / (size / 8) + 1 + y > height)
        {
            return;
        }
        if (size == ch32x32)
            temp = chF32x32[addr][pos]; //调用1206字体
        else if (size == ch24x24)
            temp = chF24x24[addr][pos]; //调用1608字体
        else
        {
            temp = chF16x16[addr][pos];
            size = ch16x16;
        }

        for (uint8_t i = 0; i < 8; i++)
        {
            if (overbit != 0)
            {
                bitpos = ((pos % (size / 8)) * 8 + i + 1);
                if (x + bitpos > width)
                    break;
            }
            if (mode == 0) //非叠加方式
            {
                if (temp & (0x01))
                    lcdSendData16Bit((uint16_t)fontColor);
                else
                    lcdSendData16Bit((uint16_t)backColor);
            }
            else
            {
                if (temp & (0x01))
                {
                    lcdDrawPoint(x, y, fontColor);
                }
                x++;
                if (x - x0 == size)
                {
                    x = x0;
                    y++;
                    break;
                }
            }
            temp >>= 1;
        }
    }
    lcdSetWindows(0, 0, width - 1, height - 1); //恢复窗口为全屏
}
#endif

/**
 * @brief 显示字符串
 * 
 * @param x x轴坐标
 * @param y y轴坐标
 * @param ch 要显示的字符串
 * @param size 字号
 * @param fontColor 字体颜色 
 * @param backColor 背景颜色
 * @param mode 绘制模式 0-覆盖模式 1-叠加模式
 * @note 叠加模式不会改变背景，可能导致改变显示时多个字重叠
 */
void LCD::lcdShowString(uint16_t x, uint16_t y, char *str, enum Lcd::fontSize size, enum Lcd::Color fontColor, uint8_t mode, enum Lcd::Color backColor)
{
    using namespace Lcd;
    char *ch = str;
    enum fontLibrary zhfont, enfont;
    switch (size)
    {
    case small:
        enfont = ascii1608;
        zhfont = ch16x16;
        break;
    case middle:
        enfont = ascii2412;
        zhfont = ch24x24;
        break;
    case large:
        enfont = ascii3216;
        zhfont = ch32x32;
        break;
    default:
        return;
    }

    while (*ch != '\0' && x < width)
    {
        if (*ch <= 127)
        {
            lcdShowChar(x, y, *ch, enfont, fontColor, backColor, mode);
            ch += 1;
            x += size / 2;
        }
        else
        {
#if CH_USE
            lcdShowCH(x, y, ch, enfont, fontColor, backColor, mode);
#else
            lcdShowChar(x, y, ‘  ’, enfont, fontColor, backColor, mode);
#endif
            ch += 2;
            x += size;
        }
    }
}

/**
 * @brief 格式化打印字符串
 * 
 * @param x x轴位置
 * @param y y轴位置
 * @param size 字号
 * @param mode 模式
 * @param format 待输出字符串
 * @param ... 同printf
 * @warning 不要使用换行符与制表符
 */
void LCD::lcdPrintf(uint16_t x, uint16_t y, enum Lcd::fontSize size, uint8_t mode, const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    char str[60];
    vsprintf(str, format, arg);
    lcdShowString(x, y, str, size, this->fontColor, mode, this->backColor);
    va_end(arg);
}
