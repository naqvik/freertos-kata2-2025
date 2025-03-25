/* Top level source file */
/** A simple app, to demonstrate freertos */

/* standard includes */

/* freertos includes */
#include "FreeRTOS.h"
#include "task.h"

/* HW-specific includes (move to bsp area) */
#include "stm32f10x.h"

/**
   BLink the LED, using the lowest-level code possible
*/
#include <stdint.h>

// Where is the green LED?  connected to PB13 or PA5 with a 510 Ohm resistor
//   It looks like it's actually PA5, based on the note on p66.
//   So we need to put a High/Low on PA5 to turn LD2 On/Off.
// So we need to use GPIO port A, bit 5
// RCC block starts at 0x4002 1000
// APB2ENR is at RCC + 0x18, set bit 2
// base of GPIOA is 0x4001 0800 (is also CRL)
// want to set pin 5 of the ODR, so we need to configure pin 5
//  using CRL register
// The ODR of GPIOA is at GPIOA base address + 12 (decimal)

/*

   Full table of all possible CNF[1:0]MODE[1:0] patterns, and their
   meanings.  GPI=General Purpose Input, GPO=General Purpose Output,
   AFO=Alternate Function Output.

   |      |      |       |       | PxODR |                         |
   | CNF1 | CNF0 | MODE1 | MODE0 | bit   | Meaning                 |
   |------+------+-------+-------+-------+-------------------------|
   |    0 |    0 |     0 |     0 | x     | GPI, analog             |
   |    0 |    0 |     0 |     1 | 0/1   | GPO, push-pull, 10MHz   |
   |    0 |    0 |     1 |     0 | 0/1   | GPO, push-pull, 2MHz    |
   |    0 |    0 |     1 |     1 | 0/1   | GPO, push-pull, 50MHz   |
   |------+------+-------+-------+-------+-------------------------|
   |    0 |    1 |     0 |     0 | x     | GPI, floating (default) |
   |    0 |    1 |     0 |     1 | 0/1   | GPO, open-drain, 10MHz  |
   |    0 |    1 |     1 |     0 | 0/1   | GPO, open-drain, 2MHz   |
   |    0 |    1 |     1 |     1 | 0/1   | GPO, open-drain, 50MHz  |
   |------+------+-------+-------+-------+-------------------------|
   |    1 |    0 |     0 |     0 | 0/1   | GPI, pulldown/pullup    |
   |    1 |    0 |     0 |     1 | x     | AFO, push-pull, 10MHz   |
   |    1 |    0 |     1 |     0 | x     | AFO, push-pull, 2MHz    |
   |    1 |    0 |     1 |     1 | x     | AFO, push-pull, 50MHz   |
   |------+------+-------+-------+-------+-------------------------|
   |    1 |    1 |     0 |     0 | x     | forbidden               |
   |    1 |    1 |     0 |     1 | x     | AFO, open-drain, 10MHz  |
   |    1 |    1 |     1 |     0 | x     | AFO, open-drain, 2MHz   |
   |    1 |    1 |     1 |     1 | x     | AFO, open-drain, 50MHz  |
   |------+------+-------+-------+-------+-------------------------|

*/

/*  Added external LEDs as follows:

                                  510
    [D6, PB10]--[RedLED]--[2]-----vvvvv-----[1]---[GND]
    [D7,  PA8]--[YlwLED]--[4]-----vvvvv-----[1]---[GND]
    [D8,  PA9]--[GrnLED]--[6]-----vvvvv-----[1]---[GND]
    [D10, PB6]--[BluLED]--[8]-----vvvvv-----[1]---[GND]

    Resistor array, 9 RES
    510 Ohm, 10SIP                    Internal structure
    +-------------------------+    [1]-----+-----+--...---+
    |  CTSK1949770101511P     |            |     |        |
    |   1 2 3 4 5 6 7 8 9 10  |            \     \        \
    +-+-+-+-+-+-+-+-+-+-+-+-+-+            /     /        /
        | | | | | | | | | |             510\  510\ ... 510\
        1 2 3 4 5 6 7 8 9 10               |     |        |
                                          [2]   [3]     [10]
 */

using Reg32 = uint32_t volatile * const;

/** Configure GPIO port, make it hard to do this wrong */
void gpio_config(GPIO_TypeDef* base, uint32_t pin, uint32_t bits4) {
    configASSERT(base != nullptr);
    configASSERT(pin < 16);
    configASSERT(bits4 < 15);

    Reg32 CR =  (pin >= 8) ? &base->CRH : &base->CRL;
    pin  = (pin >= 8) ? pin - 8 : pin;
    *CR &= ~(0xfu << (pin*4));  // zero the nybble
    *CR |= bits4 << (pin*4);    // assign bits4 to the nybble
}

void gpio_on_off(GPIO_TypeDef * base, uint32_t pin, bool on) {
    configASSERT(base != nullptr);
    configASSERT(pin < 16);

    uint32_t mask = 1u << pin;
    if (on)
        base->ODR |= mask;
    else
        base->ODR &= ~mask;
}

void blinkPA5(void * blah) {
    // turn on clock for GPIOA
    RCC->APB2ENR |= 1u<<2;

    // configure PA5 to be output, push-pull, 50MHz
    gpio_config(GPIOA, 5u, 0b0011u);

    while (1) {
        gpio_on_off(GPIOA, 5u, 1);
        vTaskDelay(400);

        gpio_on_off(GPIOA, 5u, 0);
        vTaskDelay(400);
    }
}
void display(void * blah) {
    // turn on clock for GPIOA
    RCC->APB2ENR |= 1u<<2;

    // configure PA8 and PA9 to be output, push-pull, 50MHz
    gpio_config(GPIOA, 8u, 0b0011u);
    gpio_config(GPIOA, 9u, 0b0011u);

    while (1) {
        gpio_on_off(GPIOA, 8u, 1);
        vTaskDelay(500);

        gpio_on_off(GPIOA, 8u, 0);
        vTaskDelay(500);

        gpio_on_off(GPIOA, 9u, 1);
        vTaskDelay(500);

        gpio_on_off(GPIOA, 9u, 0);
        vTaskDelay(500);
    }
}

int main() {
    BaseType_t retval = xTaskCreate(
        blinkPA5,    // task function
        "blink pa5", // task name
        50,          // stack in words
        nullptr,     // optional parameter
        4,           // priority
        nullptr      // optional out: task handle
        );
    configASSERT(retval==pdPASS);

    retval = xTaskCreate(
        display,    // task function
        "blink pa8", // task name
        50,          // stack in words
        nullptr,     // optional parameter
        4,           // priority
        nullptr      // optional out: task handle
        );
    configASSERT(retval==pdPASS);

    vTaskStartScheduler();
        
    // deadloop in case we fall through
    while (1) {}
}

