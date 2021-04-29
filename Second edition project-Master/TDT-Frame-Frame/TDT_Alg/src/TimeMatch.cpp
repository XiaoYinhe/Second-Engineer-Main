#include "TimeMatch.h"
#if (!defined VISION) && (!defined CONTROL)
#error[TDT_COMPILE_ERROR]expect a define of VISION or CONTROL here
#endif
namespace tdtusart
{

    timeSimulaneity::node_::node_(void *buffer, int size, int *r, int *w) : buffer_(buffer), size_(size), rcnt(r), wcnt(w){};

    void timeSimulaneity::node_::clear()
    {
        if (buffer_ == 0)
            return;
        memset(buffer_, 0, size_);
        *wcnt = 0;
        *rcnt = 0;
    }

    timeSimulaneity::timeSimulaneity(int max_size, int tick_max_size, int bit_rate)
    {
        time_diff = 0;
        max_size_ = max_size;
        tick_size_ = tick_max_size;
        bit_rate_ = bit_rate;
        data_ = malloc(max_size * tick_max_size + max_size * sizeof(float));
        while (data_ == NULL)
        {
#ifdef VISION
            TDT_ERROR("[时间同步]申请内存失败，重新申请中");
#elif defined CONTROL
            // todo:电控错误处理
#endif
            data_ = malloc(max_size * tick_max_size + max_size * sizeof(float));
        }
        memset(data_, 0, max_size * tick_max_size + max_size * sizeof(float));
        rcnt_catch = new int[max_size_];
        wcnt_catch = new int[max_size_];
        memset(rcnt_catch, 0, max_size_ * sizeof(int));
        memset(wcnt_catch, 0, max_size_ * sizeof(int));
    };

    timeSimulaneity::node_ timeSimulaneity::at(int index, float time)
    {
        if (index >= max_size_)
        {
#ifdef VISION
            TDT_ERROR("[时间同步]索引值大于最大值,将会返回最最末尾节点");
            return timeSimulaneity::node_((char *)data_ + (max_size_ - 1) * tick_size_, tick_size_, rcnt_catch + (max_size_ - 1), wcnt_catch + (max_size_ - 1));
#elif defined CONTROL
            return timeSimulaneity::node_((char *)data_ + (max_size_ - 1) * tick_size_, tick_size_, rcnt_catch + (max_size_ - 1), wcnt_catch + (max_size_ - 1));
#endif
        }
        float time_in_cache;
        memcpy(&time_in_cache, (char *)data_ + max_size_ * tick_size_ + index * sizeof(float), sizeof(float));
        if ((time == 0.0f && time_in_cache != 0) || time == -1.0f)
        {
            float time_now;
#ifdef VISION
            time_now = cv::getTickCount() / cv::getTickFrequency();
#elif defined CONTROL
            time_now = getSysTimeUs() / 1e6f;
            time_now = getFixedTime(time_now);
#endif
            memcpy((char *)data_ + max_size_ * tick_size_ + index * sizeof(float), &time_now, sizeof(float));
        };
        if (time > 0)
        {
            memcpy((char *)data_ + max_size_ * tick_size_ + index * sizeof(float), &time, sizeof(float));
        }
        return timeSimulaneity::node_((char *)data_ + index * tick_size_, tick_size_, rcnt_catch + index, wcnt_catch + index);
    }

    void timeSimulaneity::readData(char *data, int dataSize)
    {
#ifdef CONTROL
        if (dataSize > max_size_ * tick_size_ + max_size_ * sizeof(float) + 1)
            dataSize = max_size_ * tick_size_ + max_size_ * sizeof(float);
        else
            dataSize--;
#else
        if (dataSize > max_size_ * tick_size_ + max_size_ * sizeof(float))
            dataSize = max_size_ * tick_size_ + max_size_ * sizeof(float);
#endif
        memcpy(data_, data, dataSize);
        memset(rcnt_catch, 0, max_size_ * sizeof(int));
        memset(wcnt_catch, 0, max_size_ * sizeof(int));
#ifdef CONTROL
        float vision_time;
        memcpy(&vision_time, (char *)data + dataSize, sizeof(float));
        timeFixed(vision_time);
#endif
    }

    void timeSimulaneity::writeData(char *data, int dataSize, bool clear)
    {
#ifdef VISION
        if (dataSize > max_size_ * tick_size_ + max_size_ * sizeof(float) + 1)
            dataSize = max_size_ * tick_size_ + max_size_ * sizeof(float);
        else
            dataSize--;
#else
        if (dataSize > max_size_ * tick_size_ + max_size_ * sizeof(float))
            dataSize = max_size_ * tick_size_ + max_size_ * sizeof(float);
#endif
        memcpy(data, data_, dataSize);
#ifdef VISION
        float time_now = cv::getTickCount() / cv::getTickFrequency();
        memcpy((char *)data + dataSize, &time_now, sizeof(float));
#endif
        if (clear)
            clearCatch();
    }

    void timeSimulaneity::clearCatch()
    {
        memset(data_, 0, max_size_ * tick_size_ + max_size_ * sizeof(float));
        memset(rcnt_catch, 0, max_size_ * sizeof(int));
        memset(wcnt_catch, 0, max_size_ * sizeof(int));
        begin = -1; //当前未删除的第一个数据
        end = 0;    //当前未写入的第一个数据
    }

    void timeSimulaneity::pop()
    {
        if (begin == -1)
            return;
        memset((char *)data_ + begin * tick_size_, 0, tick_size_);
        memset((char *)data_ + max_size_ * tick_size_ + begin * sizeof(float), 0, tick_size_);
        wcnt_catch[begin] = 0;
        rcnt_catch[begin] = 0;
        if (end == -1)
            end = begin;
        begin++;
        if (begin == max_size_)
            begin = 0;
        if (begin == end)
            begin = -1;
    }

    timeSimulaneity::node_ timeSimulaneity::top(float time, bool cover)
    {
        if (end == -1)
        {
            if (cover)
                pop();
            else
                return node_(NULL, 0, NULL, NULL);
        }
        if (time == 0.0f)
        {

            float time_now;
#ifdef VISION
            time_now = cv::getTickCount() / cv::getTickFrequency();
#elif defined CONTROL
            time_now = getSysTimeUs() / 1e6f;
            time_now = getFixedTime(time_now);
#endif
#ifdef VISION
            time_now = cv::getTickCount() / cv::getTickFrequency();
#elif defined CONTROL
            time_now = getSysTimeUs() / 1e6f;
            time_now = getFixedTime(time_now);
#endif
            memcpy((char *)data_ + max_size_ * tick_size_ + end * sizeof(float), &time_now, sizeof(float));
        };
        if (time > 0)
        {
            memcpy((char *)data_ + max_size_ * tick_size_ + end * sizeof(float), &time, sizeof(float));
        }
        if (begin == -1)
            begin = end;
        int t = end;
        end++;
        if (end == max_size_)
            end = 0;
        if (end == begin)
            end = -1;
        return node_((char *)data_ + t * tick_size_, tick_size_, rcnt_catch + t, wcnt_catch + t);
    }

    int timeSimulaneity::getIndexByNode(timeSimulaneity::node_ node)
    {
        int ptr_dis = node.begin() - (char *)data_;
        return ptr_dis / tick_size_;
    }

    float timeSimulaneity::getTimeByIndex(int index)
    {
        if (index >= max_size_)
        {
#ifdef VISION
            TDT_ERROR("[时间同步]索引值大于最大值,将会返回最末尾节点的时间");
            index = max_size_ - 1;
#elif defined CONTROL
            index = max_size_ - 1;
#endif
        }
        float time_in_cache;
        memcpy(&time_in_cache, (char *)data_ + max_size_ * tick_size_ + index * sizeof(float), sizeof(float));
        return time_in_cache;
    }

#ifdef VISION
    int timeSimulaneity::timeMatch(float time)
    {
        float timelist[max_size_];
        memcpy(timelist, (char *)data_ + max_size_ * tick_size_, max_size_ * sizeof(float));
        std::sort(timelist, timelist + max_size_);
        float *i = std::lower_bound(timelist, timelist + max_size_, time);
        int k = i - timelist;
        k = abs(time - timelist[k]) < abs(time - timelist[k - 1 >= 0 ? k - 1 : 0]) ? k : k - 1;
        TDT_INFO("[时间同步]时间差为%f", time - timelist[k]);
        return k;
    }
#endif

#ifdef CONTROL
    void timeSimulaneity::timeFixed(float vision_time)
    {
        float time_now;
        time_now = getSysTimeUs() / 1e6f;
        time_diff = time_now - vision_time - (float)10.0 * (max_size_ * tick_size_ + max_size_ * sizeof(float)) / bit_rate_;
    }

    float timeSimulaneity::getFixedTime(float stm32_time) { return stm32_time - time_diff; }
#endif
    timeSimulaneity::~timeSimulaneity()
    {
        free(data_);
        free(rcnt_catch);
        free(wcnt_catch);
    }
} // namespace tdtusart