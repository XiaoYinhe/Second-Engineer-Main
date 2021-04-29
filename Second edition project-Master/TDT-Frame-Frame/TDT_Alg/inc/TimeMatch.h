#ifndef __TIME_MATCH_H
#define __TIME_MATCH_H

//#define VISION
 #define CONTROL
#include "string.h"
#include <algorithm>
#ifdef VISION
#include "log.h"
#include <opencv2/opencv.hpp>
#endif
#ifdef CONTROL
#include "board.h"
#include "my_math.h"
#endif
namespace tdtusart {
    class timeSimulaneity {
    private:
        void *data_;      //储存数据
        int   max_size_;  //数据块数量
        int   tick_size_; //每个数据块最大大小
        int   bit_rate_;  //串口传输比特率
        int * rcnt_catch; //数组，储存每个数据块上次读取位置信息
        int * wcnt_catch; //数组，储存每个数据块上次写入位置信息
        int   begin = -1; //当前未删除的第一个数据
        int   end   = 0;  //当前未写入的第一个数据
#ifdef CONTROL
        float time_diff = 0;
#endif
        class node_ {
        private:
            void *buffer_; //数据块开始指针
            int   size_;   //数据块大小
            int * wcnt;    //写入位置信息
            int * rcnt;    //读取位置信息

        public:
            /**************************************************************************
             * @brief           向缓存里写入数据
             * @param           [in] 任意变量 用于写入的数据
             ***************************************************************************/
            template <typename T> node_ &operator<<(T a) {
                if (*wcnt + sizeof(T) > size_) {
#ifdef VISION
                    TDT_ERROR("[时间同步]缓存区写入溢出");
#elif defined CONTROL
                    // todo:电控错误处理
#endif
                    return *this;
                }
                memcpy((char *)buffer_ + *wcnt, &a, sizeof(T));
                *wcnt += sizeof(T);
				return *this;
            }

            /**************************************************************************
             * @brief           从缓存里读取数据
             * @param           [in] 任意变量 需要写入的变量
             ***************************************************************************/
            template <typename T> node_ &operator>>(T &a) {
                if (*rcnt + sizeof(T) > size_) {
#ifdef VISION
                    TDT_ERROR("[时间同步]缓存区读取溢出");
#elif defined CONTROL
                    // todo:电控错误处理
#endif
                    return *this;
                }
                memcpy(&a, (char *)buffer_ + *rcnt, sizeof(T));
                *rcnt += sizeof(T);
				return *this;
            }

            /**************************************************************************
             * @name            node_
             * @brief           构造函数，依次传入数据开始位置指针，该数据块大小，上次读取位置信息的指针，上次写入位置信息的指针
             *
             * @param           [in] buffer 数据开始位置指针
             * @param           [in] size 该数据块大小
             * @param           [in] r 上次读取位置信息的指针
             * @param           [in] w 上次写入位置信息的指针
             ***************************************************************************/
            node_(void *buffer, int size, int *r, int *w);

            /**************************************************************************
             * @name            clear
             * @brief           清空该节点的缓存与读写信息
             ***************************************************************************/
            void clear();

            /**************************************************************************
             * @name            begin
             * @brief           返回数据块的起始指针
             ***************************************************************************/
            char *begin() { return (char *)buffer_; };
        };

    public:
        /**************************************************************************
         * @name            timeSimulaneity
         * @brief           构造函数，依次传入数据块数量，每个数据块最大大小，串口比特率
         *
         * @param           [in] max_size 数据块数量
         * @param           [in] tick_max_size 每个数据块最大大小
         * @param           [in] bit_rate 串口比特率
         ***************************************************************************/
        timeSimulaneity(int max_size, int tick_max_size, int bit_rate = 460800);

        /**************************************************************************
         * @name            at
         * @brief           （at创建的数据并不会被top跳过）返回包含索引对应数据的可用于读写的节点
         *
         * @param           [in] index 数据索引值
         * @param           [in] time 若为正数则填写当前时间，用其打上时间戳；若为0.0f，则检测是否已打上时间戳，若未打上则自动获取时间打上时间戳；若为-1.0f则强行获取当前时间打上时间戳；其他数值则不打上时间戳
         * @note            at创建的数据并不会被top跳过,若未top也不会被pop释放，top若获取到at创建的数据不会清空读写信息
         ***************************************************************************/
        node_ at(int index, float time = 0.0f);

        /**************************************************************************
         * @name            readData
         * @brief           读取数据到缓存(会清空之间的缓存读写记录)，电控则同时比对时间
         *
         * @param           [in] data 串口收到的数据
         * @param           [in] dataSize 数据数组的字节数，对于静态数组可以调用sizeof(数组名)
         ***************************************************************************/
        void readData(char *data, int dataSize);

        /**************************************************************************
         * @name            writeData
         * @brief           将缓存中的数据写出，视觉会同时写出当前时间
         *
         * @param           [in] data 要将数据写入的数组
         * @param           [in] dataSize 数组的字节数，对于静态数组可以调用sizeof(数组名)
         * @param           [in] clear 是否在写入后清空缓存
         ***************************************************************************/
        void writeData(char *data, int dataSize, bool clear = 0);

        /**************************************************************************
         * @name            clearCatch
         * @brief           清空缓存以及读写情况与top,pop信息
         *
         ***************************************************************************/
        void clearCatch();

        /**************************************************************************
         * @name            pop
         * @brief           弹出一个当前存在的最早存入的数据
         *
         * @note            若不存在则直接return
         ***************************************************************************/
        void pop();

        /**************************************************************************
         * @name            top
         * @brief           占用一个新的数据块并返回其node节点
         *
         * @param           [in] time 若为正数则填写当前时间，用其打上时间戳；若为0.0f，则自动获取时间打上时间戳；若为负数则不打时间戳
         * @param           [in] cover 若为true则在无节点可用时pop一个节点
         * @return          返回该数据块node节点,若不存在则返回node_(NULL, 0, NULL, NULL)
         ***************************************************************************/
        node_ top(float time = 0.0f, bool cover = 1);

        /**************************************************************************
         * @name            getIndexByNode
         * @brief           通过节点获取索引
         *
         * @param           [in] node 数据块节点
         * @return          该数据块所在的索引
         ***************************************************************************/
        int getIndexByNode(node_ node);

        /**************************************************************************
         * @name            getTimeByIndex
         * @brief           获取索引所在数据块的时间戳
         *
         * @param           [in] index 索引
         * @return          该索引的数据块的时间戳
         ***************************************************************************/
        float getTimeByIndex(int index);

#ifdef VISION
        /**************************************************************************
         * @name            timeMatch
         * @brief           匹配最相近的数据
         *
         * @param           [in] time 用于匹配的时间
         * @return          匹配到的数据的索引
         ***************************************************************************/
        int timeMatch(float time);
#endif

#ifdef CONTROL
        /**************************************************************************
         * @name            timeFixed
         * @brief           获取执行函数时时间进行时间修正
         *
         * @param           [in] vision_time 视觉时间
         ***************************************************************************/
        void timeFixed(float vision_time);

        /**************************************************************************
         * @name            getFixedTime
         * @brief           获得修正后的视觉时间
         *
         * @param           [in] stm32_time 电控时间
         * @return          修正后的视觉时间
         ***************************************************************************/
        float getFixedTime(float stm32_time);
#endif

        /**************************************************************************
         * @name            ~timeSimulaneity
         * @brief           析构函数，释放内存
         *
         ***************************************************************************/
        ~timeSimulaneity();
    };
} // namespace tdtusart

#endif