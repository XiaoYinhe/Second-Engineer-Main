/******************************
 @file TDT_Devce\inc\LCD.h
 @brief  LCD屏幕驱动
 @author 彭阳
 @version 2.3.0
 @date 20.12.19
 @history:
	——————————————————————————————————————————————————————————————————————————
	20.12.19 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************/
#ifndef _LCD_H
#define _LCD_H

#include "board.h"
#include "spi.h"

namespace Lcd
{
    enum lcdDirection //默认屏幕排针朝向下 按顺序为屏幕顺时针旋转0°、90°、180°、270°
    {
        up,
        right,
        down,
        left
    };

    enum Color : uint16_t //16位高彩色，采用R5B5G5A1格式
    {
        WHITE = 0xFFFF,
        BLACK = 0x0000,
        BLUE = 0x001F,
        BRED = 0XF81F,
        GRED = 0XFFE0,
        GBLUE = 0X07FF,
        RED = 0xF800,
        MAGENTA = 0xF81F,
        GREEN = 0x07E0,
        CYAN = 0x7FFF,
        YELLOW = 0xFFE0,
        BROWN = 0XBC40,      //棕色
        BRRED = 0XFC07,      //棕红色
        GRAY = 0X8430,       //灰色
        DARKBLUE = 0X01CF,   //深蓝色
        LIGHTBLUE = 0X7D7C,  //浅蓝色
        GRAYBLUE = 0X5458,   //灰蓝色
        LIGHTGREEN = 0X841F, //浅绿色
        LIGHTGRAY = 0XEF5B,  //浅灰色(PANNEL)
        LGRAY = 0XC618,      //浅灰色(PANNEL),
        LGRAYBLUE = 0XA651,  //浅灰蓝色(中间层颜色)
        LBBLUE = 0X2B12      //浅棕蓝色(选择条目的0反色)
    };

    enum Command : uint8_t // ILI9341芯片命令，未全部列出
    {
        nopCommand = 0x00,              //空指令
        softwareReset = 0x01,           //软件复位
        enterSleep = 0x10,              //进入睡眠模式
        exitSleep = 0x11,               //退出睡眠模式
        partialMode = 0x12,             //部分显示模式
        nomalMode = 0x13,               //正常显示模式
        displayInversionOn = 0x20,      //反色显示
        displayInversionOff = 0x20,     //取消反色显示
        gammaSet = 0x26,                //设定伽马值
        displayOff = 0x28,              //关闭显示屏，屏幕变为纯白
        displayOn = 0x29,               //打开显示屏
        setX = 0x2A,                    //设定x轴显示地址
        setY = 0x2B,                    //设定y轴显示地址
        writeRAM = 0x2C,                //开始写入缓存
        setColor = 0x2D,                //设置颜色格式
        partialArea = 0x30,             //设定部分区域
        verticalScrolling = 0x33,       //设置垂直滚动
        displayFunctionControl = 0xB6,  //显示功能控制
        powerControl1 = 0xC0,           //功率控制0
        powerControl2 = 0xC1,           //功率控制1
        vcomControl1 = 0xC5,            //VCOM控制1
        memoryAccessControl = 0x36,     //屏幕方向控制
        pixelFormatSet = 0x3A,          //像素格式控制
        vcomControl2 = 0xC7,            //VCOM控制2
        powerControlA = 0xCB,           //功率控制A
        powerControlB = 0xCF,           //功率控制B
        positiveGammaCorrection = 0xE0, //正极伽马校准
        negativeGammaCorrection = 0xE1, //负极伽马校准
        diverTimingControlA = 0xE8,     //驱动时序控制A
        diverTimingControlB = 0xEA,     //驱动时序控制B
        powerSequenceControl = 0xED,    //电源序列控制
        enable3Gamma = 0xF2,            //使能伽马
        pumpRatioControl = 0xF7,        //泵比控制

    };

    enum fontLibrary //字库
    {
        ascii1608 = 16,
        ascii2412 = 24,
        ascii3216 = 32,
        ch16x16 = 16,
        ch24x24 = 24,
        ch32x32 = 32,
    };

    enum fontSize //字号
    {
        small = 16,
        middle = 24,
        large = 32,
    };

    const int16_t LcdWidth = 240; //屏幕宽度
    const int16_t LcdHight = 320; //屏幕长度

} // namespace Lcd

struct Pin // 包含端口和引脚
{
    GPIO_TypeDef *port;
    uint16_t pin;
};

class LCD : public Spi
{
private:
    Pin dc;    //数据/命令控制信号
    Pin reset; //lcd复位信号

    uint16_t width;                   //LCD 宽度
    uint16_t height;                  //LCD 高度
    enum Lcd::lcdDirection direction; //屏幕方向

    enum Lcd::Color fontColor; //画笔颜色
    enum Lcd::Color backColor; //背景颜色

    void lcdPinInit(void); //lcd数据线和复位线引脚初始化

    inline void lcdDcSet(void);      //lcd数据线拉高
    inline void lcdDcClear(void);    //lcd数据线拉低
    inline void lcdResetSet(void);   //lcd复位线拉高
    inline void lcdResetClear(void); //lcd复位线拉低

    inline void lcdSendByte(uint8_t data); //对spi发送函数进行二次封装

    void lcdSendCommand(uint8_t data);    //发送命令
    void lcdSendData(uint8_t data);       //发送数据
    void lcdSendData16Bit(uint16_t data); //发送16位数据
    void lcdWriteRAMPrepare(void);        //开始写入缓存

    void lcdDrawCircle8(int xc, int yc, int x, int y, enum Lcd::Color color);                                //8点对称画圆算法
    void lcdDrawFill(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color); //填充区域

    void lcdShowChar(uint16_t x, uint16_t y, char cha, enum Lcd::fontLibrary size, enum Lcd::Color fontColor, enum Lcd::Color backColor, uint8_t mode); //显示字符
    void lcdShowCH(uint16_t x, uint16_t y, char *ch, enum Lcd::fontLibrary size, enum Lcd::Color fontColor, enum Lcd::Color backColor, uint8_t mode);   //显示中文汉字

public:
    LCD(GPIO_TypeDef *dcPort, uint16_t dcPin, GPIO_TypeDef *resetPort, uint16_t resetPin, SPI_TypeDef *spix);

    void lcdInit(void);                   //LCD屏幕初始化
    void lcdReset(void);                  //复位lcd屏
    void lcdClear(enum Lcd::Color color); //清屏

    void lcdSetDirection(enum Lcd::lcdDirection direction);                             //设置屏幕方向
    void lcdSetWindows(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd); //设定lcd屏幕绘制区域
    void lcdSetCursor(uint16_t xPos, uint16_t yPos);                                    //设定光标
    void lcdSetColor(enum Lcd::Color fontColor, enum Lcd::Color backColor);             //设置显示字符串颜色

    void lcdDrawPoint(uint16_t x, uint16_t y, enum Lcd::Color color);                                                              //画一个点
    void lcdDrawLine(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color);                       //画一条线
    void lcdDrawRectangle(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color);                  //绘制空心矩形
    void lcdDrawFillRectangle(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, enum Lcd::Color color);              //绘制实心矩形
    void lcdDrawCircle(uint16_t x, uint16_t y, uint16_t r, uint8_t fill, enum Lcd::Color color);                                   //画圆
    void lcdDrawTriangel(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, enum Lcd::Color color);     //绘制空心三角形
    void lcdDrawFillTriangel(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, enum Lcd::Color color); //绘制实心三角形

    //显示字符串
    void lcdShowString(uint16_t x, uint16_t y, char *str, enum Lcd::fontSize size, enum Lcd::Color fontColor, uint8_t mode = 1, enum Lcd::Color backColor = Lcd::BLACK);
    //格式化打印字符串
    void lcdPrintf(uint16_t x, uint16_t y, enum Lcd::fontSize size, uint8_t mode, const char *format, ...);
};

#endif
