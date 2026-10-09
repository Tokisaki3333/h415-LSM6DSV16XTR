#include "USBCDC.h"
#include "ch32h417_usbhs_device.h"
#include "usbd_compatibility_hid.h"

/* USBHS-HID ʹ�ܣ��Ƚ��� SWJ���������жϹ������ʼ���豸 */
void usbhs_hid_enable(void)
{
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);   /* �رյ��Կڣ�USBHS ��ռ */

    RCC_HB2PeriphClockCmd(RCC_HB2Periph_GPIOC | RCC_HB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef gpio = {0};
    gpio.GPIO_Pin  = GPIO_Pin_9, gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &gpio);
    gpio.GPIO_Pin  = GPIO_Pin_11, gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_Low;
    GPIO_Init(GPIOB, &gpio);
    GPIO_SetBits(GPIOB, GPIO_Pin_11);    /* ��ʼ:�� = PF0 �� */

    NVIC_SetAllocateIRQ(USBHS_IRQn, Core_ID_V5F);
    USBHS_Device_Init(ENABLE);
}

/* ��ѭ����ѯ��HID ���й��� + PC9 �ɿ��ص��δ����л� SWJ */
void usbhs_hid_poll(void)
{
    static uint8_t trig_armed = 1;      /* ÿ���ϵ����һ��(�����ٻ�) */
    static uint8_t key_prev    = 1;

    if (hid_up_avail())
        hid_up_flush();

    hid_cmd_poll();                     /* ���У�������������������� */

    if (trig_armed)
    {
        uint8_t key_now = (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == RESET) ? 0 : 1;
        if (key_prev == 0 && key_now == 1)
        {
            trig_armed = 0;
            GPIO_WriteBit(GPIOB, GPIO_Pin_11, Bit_RESET);  /* PF0 ��(��) */

            /* �̶����У���ʧ�� USBHS����ʹ�� SWJ(����������) */
            USBHS_Device_Init(DISABLE);
            GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);
        }
        key_prev = key_now;
    }
}
