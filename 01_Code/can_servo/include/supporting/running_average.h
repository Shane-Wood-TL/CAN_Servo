#ifndef __RUNNING_AVERAGE_H__
#define __RUNNING_AVERAGE_H__

#include "../all_includes.h"


#define WINDOW_SIZE 10


class running_average{
    private:
        float buffer[WINDOW_SIZE] = {0};
        uint8_t index = 0;
        uint8_t count = 0;
        float sum = 0;
    public:
        void add_value(float new_value);
        float average();
};


#endif /* __RUNNING_AVERAGE_H__ */