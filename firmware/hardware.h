/*
 * Copyright (C) 2024 saybur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef __HARDWARE_H__
#define __HARDWARE_H__

/*
 * ----------------------------------------------------------------------------
 *     PIN DEFINITIONS
 *   Change these to match your hardware.
 * ----------------------------------------------------------------------------
 *
 * The RP2040 has fewer "gotchas" than other MCUs but there still are some
 * limitations to be aware of if you're trying a new board spin:
 *
 * - LEDs should not be on PWMs shared with other periphals that use the PWM
 *   system, such as the buzzer, or you will need to adjust the LED driving
 *   code.
 * - Generally try to have the _DO lines on PWM_CHB (odd GPIOs), they are the
 *   only ones with duty cycle / frequency measuring. This is not currently
 *   implemented, PIO handles all that stuff, but it might be useful in the
 *   future.
 */

#define LED_ERR_PIN         0
#define LED_ACT_PIN         1
#define LED_C1_PIN          2
#define LED_C2_PIN          3
#define LED_C3_PIN          4
#define LED_C4_PIN          5
#define SWITCH_PIN          6
#define BUZZER_PIN          7
#define A_DO_PIN            26
#define A_DI_PIN            27
#define C1_DO_PIN           18
#define C1_DI_PIN           19
#define C1_PSW_PIN          17
#define C2_DO_PIN           15
#define C2_DI_PIN           16
#define C2_PSW_PIN          14
#define C3_DO_PIN           12
#define C3_DI_PIN           13
#define C3_PSW_PIN          11
#define C4_DO_PIN           9
#define C4_DI_PIN           10
#define C4_PSW_PIN          8

/*
 * ----------------------------------------------------------------------------
 *     RESOURCE DEFINITIONS / ALIASES
 *   Use care changing anything past this line!
 * ----------------------------------------------------------------------------
 */

// "computer" refers to the emulated device ports on the back
#define COMPUTER_PIO        pio0
#define COMPUTER_PIO_IRQ0   PIO0_IRQ_0
#define COMPUTER_COUNT      4

// device DMA units, used to handle initial signal detection and Listen data
#define COMPUTER_DMA_IRQ    DMA_IRQ_1
#define COMPUTER_DMA_IDX    1
#define COMPUTER_DMA_INTS   dma_hw->ints1
#define COMPUTER_0_DMA      6
#define COMPUTER_1_DMA      7
#define COMPUTER_2_DMA      8
#define COMPUTER_3_DMA      9

// "host" refers to the connection with the native peripherals on the front
#define HOST_PIO            pio1
#define HOST_PIO_IRQ0       PIO1_IRQ_0

#define BUZZER_TIMER        2
#define BUZZER_TIMER_IRQ    TIMER_IRQ_2
#define COMPUTER_TIMER      1
#define COMPUTER_TIMER_IRQ  TIMER_IRQ_1
#define HOST_TIMER          0
#define HOST_TIMER_IRQ      TIMER_IRQ_0

#define A_DO_PIN_bm         (1U << A_DO_PIN)
#define C1_DO_PIN_bm        (1U << C1_DO_PIN)
#define C2_DO_PIN_bm        (1U << C2_DO_PIN)
#define C3_DO_PIN_bm        (1U << C3_DO_PIN)
#define C4_DO_PIN_bm        (1U << C4_DO_PIN)

#define A_DO_PIN_bm         (1U << A_DO_PIN)
#define C1_DO_PIN_bm        (1U << C1_DO_PIN)
#define C2_DO_PIN_bm        (1U << C2_DO_PIN)
#define C3_DO_PIN_bm        (1U << C3_DO_PIN)
#define C4_DO_PIN_bm        (1U << C4_DO_PIN)

#endif /* __HARDWARE_H__ */
