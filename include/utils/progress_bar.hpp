#ifndef PROGRESS_BAR_H_
#define PROGRESS_BAR_H_

#include "pcheaders.h"

class ProgressBar
{
private:
    uint8_t m_barWidth;

public:
    ProgressBar() : m_barWidth(50)
    {
    }

    void show(double progress)
    {
        size_t pos = m_barWidth * progress / 100;

        CONSOLE_LOG("[");
        for (size_t i = 0; i < m_barWidth; ++i)
        {
            if (i < pos)
                CONSOLE_LOG("=");
            else if (i == pos)
                CONSOLE_LOG(">");
            else
                CONSOLE_LOG(" ");
        }
        CONSOLE_LOG("] " << int(progress) << " %\r");
        std::cout.flush();
    }

    void done(void)
    {
        CONSOLE_LOG(std::endl);
    }
};

#endif
