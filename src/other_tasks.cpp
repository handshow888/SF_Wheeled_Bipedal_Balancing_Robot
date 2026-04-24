#include "tasks/other_tasks.h"

void can_rx_task(void *pvParameters)
{
    twai_message_t rxFrame;
    while (true)
    {
        // 快速读取并丢弃
        twai_receive(&rxFrame, pdMS_TO_TICKS(1));
        // vTaskDelay(pdMS_TO_TICKS(1));
    }
}