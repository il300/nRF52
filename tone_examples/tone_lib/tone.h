#ifndef TONE_H_
#define TONE_H_

//#define LOG_INFOR_ENABLE (1)


#include <variant.h>

#define TONE_USED_GPIOTE_NUM          7
#define TONE_USED_PPI_CHANNAL         0
#define TONE_USED_TIMER               NRF_TIMER1
#define TONE_USED_TIMER_IRQn          TIMER1_IRQn
#define TONE_USED_TIMER_IRQHandler    TIMER1_IRQHandler

#if defined(NRF51)
#define _PRIO_SD_HIGH       0
#define _PRIO_APP_HIGH      1
#define _PRIO_APP_MID       1
#define _PRIO_SD_LOW        2
#define _PRIO_APP_LOW       3
#define _PRIO_APP_LOWEST    3
#define _PRIO_THREAD        4
#elif defined(NRF52)
#define _PRIO_SD_HIGH       0
#define _PRIO_SD_MID        1
#define _PRIO_APP_HIGH      2
#define _PRIO_APP_MID       3
#define _PRIO_SD_LOW        4
#define _PRIO_SD_LOWEST     5
#define _PRIO_APP_LOW       6
#define _PRIO_APP_LOWEST    7
#define _PRIO_THREAD        15
#else
    #error "No platform defined"
#endif

typedef enum
{
#ifdef SOFTDEVICE_PRESENT
    APP_IRQ_PRIORITY_HIGHEST = _PRIO_SD_HIGH,
#else
    APP_IRQ_PRIORITY_HIGHEST = _PRIO_APP_HIGH,
#endif
    APP_IRQ_PRIORITY_HIGH    = _PRIO_APP_HIGH,
#ifndef SOFTDEVICE_PRESENT
    APP_IRQ_PRIORITY_MID     = _PRIO_SD_LOW,
#else
    APP_IRQ_PRIORITY_MID     = _PRIO_APP_MID,
#endif
    APP_IRQ_PRIORITY_LOW     = _PRIO_APP_LOW,
    APP_IRQ_PRIORITY_LOWEST  = _PRIO_APP_LOWEST,
    APP_IRQ_PRIORITY_THREAD  = _PRIO_THREAD     /**< "Interrupt level" when running in Thread Mode. */
} app_irq_priority_t;


#if LOG_INFOR_ENABLE
#include "SEGGER_RTT.h"
#define log_info(format, ...)  SEGGER_RTT_printf(0, format, ## __VA_ARGS__)
#else
#define log_info(format, ...)
#endif

void tone(uint8_t pin, uint16_t freq, uint32_t duration);
void noTone(uint8_t pin);


#endif
