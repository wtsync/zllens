#include "arduino_compact.h"
#include "gd32f1x0.h"

// PA0-PA15 ==> 1-15
// PB0-PB7  ==> 16-23
int digitalRead(int pin) {
    if (pin < 0) {
        return LOW;
    } else if (pin <= 15) {
        return (0 == ((GPIOA->DIR) & (1 << pin))) ? LOW : HIGH;
    } else if (pin <= 23) {
        return (0 == ((GPIOB->DIR) & (1 << (pin - 16)))) ? LOW : HIGH;
    } else {
        return LOW;
    }
}
void digitalWrite(int pin, int val) {
    if (pin < 0) {
        return;
    } else if (pin <= 15) {
        if (HIGH == val) {
            GPIOA->BOR = (1 << pin);
        } else {
            GPIOA->BCR = (1 << pin);
        }
    } else if (pin <= 23) {
        if (HIGH == val) {
            GPIOB->BOR = (1 << (pin - 16));
        } else {
            GPIOB->BCR = (1 << (pin - 16));
        }
    } else {
        return;
    }
}
void pinMode(int pin, int mod) {
    GPIO_InitPara GPIO_InitStructure;
    GPIO_TypeDef * pin_group;

    if (pin < 0) {
        return;
    } else if (pin <= 15) {
        pin_group = GPIOA;
        GPIO_InitStructure.GPIO_Pin     = (1 << pin);
    } else if (pin <= 23) {
        pin_group = GPIOB;
        GPIO_InitStructure.GPIO_Pin     = (1 << (pin - 16));
    } else {
        return;
    }
    GPIO_InitStructure.GPIO_Mode    = (OUTPUT == mod) ?  GPIO_MODE_OUT : GPIO_MODE_IN;

    GPIO_InitStructure.GPIO_OType   = GPIO_OTYPE_PP;
    GPIO_InitStructure.GPIO_PuPd    = (INPUT_PULLUP == mod) ? GPIO_PUPD_PULLUP : GPIO_PUPD_NOPULL;
    GPIO_InitStructure.GPIO_Speed   = GPIO_SPEED_50MHZ;
    GPIO_Init(pin_group, &GPIO_InitStructure );
}
