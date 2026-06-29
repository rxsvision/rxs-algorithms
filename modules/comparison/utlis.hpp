#pragma once
#include<queue>
#include<vector>


class CircularBuffer {
public:
    float threshold;
private:
    std::vector<float> buffer;
    int capacity;       // 缓冲区的最大容量
    int count;          // 当前缓冲区中的元素数量
    int head;           // 指向最早插入的数据
    float sum;          // 缓冲区内所有元素的总和

public:
    // 构造函数
    CircularBuffer(int size) : capacity(size), count(0), head(0), sum(0.0f), threshold(0.1f){
        buffer.resize(capacity);
    }

    // 向缓冲区添加元素
    void add(float value) {
        if (count != 0 && abs(end()-value)>threshold)
        {
            buffer.clear();
            count = 0;
            head=0;
            sum = 0;
        }
        if (count < capacity) {
            buffer[count] = value;
            sum += value;
            count++;
        }
        else {
            // 当缓冲区已满，替换最早的元素
            sum -= buffer[head];
            buffer[head] = value;
            sum += value;
            head = (head + 1) % capacity; // 移动头指针
        }
    }

    float end()
    {
        if (count == 0) return 0;
        return buffer[count - 1];
    }

    // 计算缓冲区内所有元素的平均值
    float average() const {
        if (count == 0) return 0.0;  // 防止除以零
        return sum / count;
    }

    // 打印缓冲区的内容（调试用）
    void print() const {
        std::cout << "Buffer contents: ";
        for (int i = 0; i < count; ++i) {
            int index = (head + i) % capacity;
            std::cout << buffer[index] << " ";
        }
        std::cout << std::endl;
    }
};
