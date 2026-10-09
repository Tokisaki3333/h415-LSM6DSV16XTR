/* USBHS CDC 双向串口（上行 EP2 bulk IN / 下行 EP2 bulk OUT），照搬 v2，仅键/LED 换脚 */

#include "USBCDC.h"
#include "ch32h417_usbhs_device.h"
#include "usbd_compatibility_hid.h"
#include "board_io.h"
#include "debug.h"

void usbhs_hid_enable(void)
{
    GPIO_InitTypeDef gpio = {0};

    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);     /* 让出 PB8/PB9 给 USBHS */

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOF | RCC_HB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin  = KEY2_PIN;                              /* PF14 下载模式键 */
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(KEY2_PORT, &gpio);

    gpio.GPIO_Pin   = LED1_PIN;                             /* PB11 指示 LED */
    gpio.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_High;
    GPIO_Init(LED1_PORT, &gpio);
    LED1_OFF();

    NVIC_SetAllocateIRQ(USBHS_IRQn, Core_ID_V5F);
    USBHS_Device_Init(ENABLE);
}

void usbhs_hid_poll(void)
{
    static uint8_t trig_armed = 1;                          /* 每次上电只切一次，单向不回 */
    static uint8_t key_prev   = 1;

    if (hid_up_avail())
        hid_up_flush();

    hid_cmd_poll();

    if (trig_armed)
    {
        uint8_t key_now = KEY2_DOWN() ? 0u : 1u;

        if (key_prev == 0u && key_now == 1u)                /* PF14 按下再松开 */
        {
            trig_armed = 0u;
            LED1_ON();

            USBHS_Device_Init(DISABLE);                     /* 先失能 USBHS 让出 PB8/PB9 */
            GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);
        }
        key_prev = key_now;
    }
}
