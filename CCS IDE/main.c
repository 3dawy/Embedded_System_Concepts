

/**
 * main.c
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "delay.h"


#define LED_RED   (1U << 1)
#define LED_BLUE  (1U << 2)
#define LED_GREEN (1U << 3)



uint32_t u32 = 0x11223344;
uint16_t u16 = 0x5566;
uint8_t  u8  = 0x77;
int main(void)
{
    uint32_t x = 0x12345678;
    uint32_t y = 0x12345678;
    u8  = 0x88;
    u16 = 0xBBCC;
    u32 = 0xAADDEEFF;

    SYSCTL_RCGCGPIO_R |= (1U << 5);  /* enable clock for GPIOF */
    SYSCTL_GPIOHBCTL_R |= (1U << 5); /* enable AHB for GPIOF */
    GPIO_PORTF_AHB_DIR_R |= (LED_RED | LED_BLUE | LED_GREEN);
    GPIO_PORTF_AHB_DEN_R |= (LED_RED | LED_BLUE | LED_GREEN);

    GPIO_PORTF_AHB_DATA_BITS_R[LED_RED] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_BLUE] = 0;
    GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = 0;
    while (1) {
        x = 0xFFFFFFFF;
        GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = LED_GREEN;
        y = 0x66666666;
        delay(500000);

        x = 0x55555555;
        y = 0x11111111;
        GPIO_PORTF_AHB_DATA_BITS_R[LED_GREEN] = 0;

        delay(490000);
    }
    return 0;
}
