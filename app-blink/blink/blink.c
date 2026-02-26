
#include <rtems.h>
#include <stdio.h>
#include <stdbool.h>
#include <bsp.h>
#include <bsp/io.h>         


rtems_task Blink_Fast(rtems_task_argument pin) {
    while (1) {
        stm32f7_gpio_set_output((int)pin, true);
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() / 2);
        stm32f7_gpio_set_output((int)pin, false);
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() / 2);
    }
}

rtems_task Blink_Medium(rtems_task_argument pin) {
    while (1) {
        stm32f7_gpio_set_output((int)pin, true);
        rtems_task_wake_after(rtems_clock_get_ticks_per_second());
        stm32f7_gpio_set_output((int)pin, false);
        rtems_task_wake_after(rtems_clock_get_ticks_per_second());
    }
}

rtems_task Blink_Slow(rtems_task_argument pin) {
    while (1) {
        stm32f7_gpio_set_output((int)pin, true);
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() * 2);
        stm32f7_gpio_set_output((int)pin, false);
        rtems_task_wake_after(rtems_clock_get_ticks_per_second() * 2);
    }
}

rtems_task Init(rtems_task_argument argument) {
    rtems_id id1, id2, id3;

    stm32f7_gpio_config led1 = STM32F7_PIN_LED1;
    stm32f7_gpio_config led2 = STM32F7_PIN_LED2;
    stm32f7_gpio_config led3 = STM32F7_PIN_LED3;
    
    stm32f7_gpio_set_config(&led1);
    stm32f7_gpio_set_config(&led2);
    stm32f7_gpio_set_config(&led3);

    rtems_task_create(rtems_build_name('L', 'E', 'D', '1'), 10, 4096, RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &id1);
    rtems_task_create(rtems_build_name('L', 'E', 'D', '2'), 11, 4096, RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &id2);
    rtems_task_create(rtems_build_name('L', 'E', 'D', '3'), 12, 4096, RTEMS_DEFAULT_MODES, RTEMS_DEFAULT_ATTRIBUTES, &id3);

    rtems_task_start(id1, Blink_Fast,   (rtems_task_argument)LED1_PIN);
    rtems_task_start(id2, Blink_Medium, (rtems_task_argument)LED2_PIN);
    rtems_task_start(id3, Blink_Slow,   (rtems_task_argument)LED3_PIN);

    rtems_task_exit();
}

