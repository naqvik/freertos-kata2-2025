/* Top level source file */
/** A simple app, to demonstrate freertos */

/* standard includes */
#include <stdio.h>

/* freertos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* HW-specific includes (move to bsp area) */
#include "stm32f10x.h"

// project specific includes
#include "serial-io.h"

/**
   BLink the LED, using the lowest-level code possible
*/
#include <stdint.h>

// Do this if you're using C
// typedef enum pin_tag {
//     Pin0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7,
//     Pin8, Pin9, Pin10, Pin11, Pin12, Pin13, Pin14, Pin15,
// } Pin;

enum Pin {
    Pin0, Pin1, Pin2, Pin3, Pin4, Pin5, Pin6, Pin7,
    Pin8, Pin9, Pin10, Pin11, Pin12, Pin13, Pin14, Pin15,
};

// global objects
static SemaphoreHandle_t gl_sequence_tasks_sem = nullptr;

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
void gpio_config(GPIO_TypeDef* base, Pin p, uint32_t bits4) {
    configASSERT(base != nullptr);
    // configASSERT(p < 16);
    configASSERT(bits4 < 15);

    int32_t pin = (int32_t)p;
    Reg32 CR =  (pin >= 8) ? &base->CRH : &base->CRL;
    pin  = (pin >= 8) ? pin - 8 : pin;
    *CR &= ~(0xfu << (pin*4));  // zero the nybble
    *CR |= bits4 << (pin*4);    // assign bits4 to the nybble
}



void gpio_on_off(GPIO_TypeDef * base, Pin p, bool on) {
    configASSERT(base != nullptr);
    // configASSERT(pin < 16);

    int32_t pin = (int32_t)p;

    // change bit state atomically
    uint32_t mask = on ? 1u << pin : 1u << (pin+16u);
    base->BSRR |= mask;
}


////////////////////////////////////////////////////////////////
// Allow hand-off from task display to blinkGrn.  I want the PA8 task
// to finish its work *before* the Grn task runs.  This semaphore is
// used to enforce this.  The PA8 task's period is 1000 ms, while
// the Grn task's period is 200 ms.  When a falling edge occurs on
// PA8, the Grn task is released, runs to completion, then
// blocks until PA8 emits another falling edge.
//
//  Grn....................--__................--__.......... etc
//  PA8__________----------__________----------__________----  etc

// Blinks LD2 on the nucleo board
void blinkGrn(void * blah) {
    // turn on clock for GPIOA
    RCC->APB2ENR |= 1u<<2;

    // configure Grn to be output, push-pull, 50MHz
    gpio_config(GPIOA, Pin9, 0b0011u);

    while (1) {
        xSemaphoreTake(gl_sequence_tasks_sem, portMAX_DELAY);  // wait
        gpio_on_off(GPIOA, Pin9, 1);
        vTaskDelay(100);

        gpio_on_off(GPIOA, Pin9, 0);
        vTaskDelay(100);
    }
}
void display(void * blah) {
    // turn on clock for GPIOA
    RCC->APB2ENR |= 1u<<2;

    // configure PA8 and PA9 to be output, push-pull, 50MHz
    gpio_config(GPIOA, Pin8, 0b0011u);
    gpio_config(GPIOA, Pin9, 0b0011u);

    while (1) {
        gpio_on_off(GPIOA, Pin8, 1);
        vTaskDelay(500);

        gpio_on_off(GPIOA, Pin8, 0);
        xSemaphoreGive(gl_sequence_tasks_sem);  // release blinkGrn task
        vTaskDelay(500);

    }
}

int main() {
    openUsart2();

    // initialize tasks
    BaseType_t retval = xTaskCreate(
        blinkGrn,    // task function
        "blink Grn", // task name
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

    // initialize semaphore for sequencing
    gl_sequence_tasks_sem = xSemaphoreCreateBinary();
    configASSERT(gl_sequence_tasks_sem != nullptr);

    printf("Starting tasks..\r\n");

    vTaskStartScheduler();
        
    // deadloop in case we fall through
    while (1) {}
}

