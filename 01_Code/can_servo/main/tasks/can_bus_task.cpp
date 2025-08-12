#include "../../include/tasks/can_bus_task.h"


void can_bus(void *pv){

    #ifdef CAN_25K
        static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_25KBITS();
    #endif
    
    #ifdef CAN_50K
        static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_50KBITS();
    #endif
    
    #ifdef CAN_100K
        static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_100KBITS();
    #endif
    
    #ifdef CAN_125K
        static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    #endif
    
    #ifdef CAN_250K
        static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    #endif
    
    #ifdef CAN_500K
        static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    #endif
    
    #ifdef CAN_800K
        static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_800KBITS();
    #endif
    
    #ifdef CAN_1M
        static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    #endif



    static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


    // static const twai_filter_config_t f_config = {
    //     .acceptance_code = (node_id << 5),     // Match the node ID bits
    //     .acceptance_mask = (uint32_t)(~0x1F),            // Mask: only match top 6+ bits, ignore lower 5
    //     .single_filter = true
    // };


    static const twai_general_config_t g_config = {
        .controller_id = 0,
        .mode = TWAI_MODE_NORMAL,
        .tx_io = can_TX_pin,
        .rx_io = can_RX_pin,
        .clkout_io = TWAI_IO_UNUSED,
        .bus_off_io = TWAI_IO_UNUSED,
        .tx_queue_len = TWAI_TX_QUEUE_SIZE,
        .rx_queue_len = TWAI_RX_QUEUE_SIZE,
        .alerts_enabled = TWAI_ALERT_NONE,
        .clkout_divider = 0,
        .intr_flags = 0
    };

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK)
    {
        assert(false);
    }

    while (twai_start() != ESP_OK)
    {
        assert(false);
    }

    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR;
    twai_reconfigure_alerts(alerts_to_enable, NULL);

    can_servo can_bus_driver(node_id);
    for(;;){
        can_bus_driver.receive_message();
        printf("CAN Bus Task running...\n");
        vTaskDelay(pdMS_TO_TICKS(CAN_BUS_TASK_DELAY));
    }
}


