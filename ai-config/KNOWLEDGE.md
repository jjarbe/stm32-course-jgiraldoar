# KNOWLEDGE.md — Week 6: Interrupts and EXTI

## Overview

This week the student encounters one of the most important conceptual shifts in embedded systems programming: moving from polling-based input handling to interrupt-driven design. Until now, every program has followed a linear, predictable flow — the CPU executes instructions in the exact order they are written. Interrupts break this model by allowing external events to temporarily suspend the main program, handle the event, and then resume exactly where execution was interrupted. This week covers the concept of interrupts, EXTI (External Interrupt) configuration for GPIO-based events, NVIC configuration using `NVIC_EnableIRQ()`, and writing correct ISR (Interrupt Service Routine) functions using the exact names defined in the startup file's interrupt vector table.

---

## Previously Mastered Topics (Weeks 0–5)

The student understands CMOS technology, logic gates, combinational and sequential circuits, binary, hexadecimal, and 2's complement number systems. They have simulated registers, shift registers, prescalers, and timers using the "Digital" simulation tool — and have now worked with real hardware timers.

In C programming, the student can write programs using all control structures (`if/else`, `for`, `while`, `do-while`, `switch-case`), fixed-width data types from `stdint.h`, all arithmetic and bitwise operators, and enumerations (`enum`). They can implement FSM patterns using `enum` and `switch-case`. They understand the `volatile` keyword and why it is essential for variables shared between an ISR and the main program. Their C skills are solid but still developing — they do NOT yet know structures, arrays, pointers, or `typedef` beyond what the IDE auto-generates.

The student fully understands the MCU architecture: the ARM Cortex-M4 CPU core, the bus system (AHB, APB1, APB2), memory-mapped registers, and Special Function Registers (SFR). They understand CMSIS structures as carefully designed overlays on the hardware memory layout — the Italian tailor analogy. The `->` operator is understood at a practical level. The underlying pointer and structure mechanics remain a black box until week 9.

The student can configure GPIO pins at the register level using CMSIS-defined masks: enabling the RCC clock, configuring MODER (input/output), OTYPER (push-pull/open-drain), OSPEEDR (speed), PUPDR (pull-up/pull-down), writing to ODR and BSRR, and reading IDR. They understand the difference between ODR and BSRR and why BSRR is the safer choice for bit manipulation.

The student has experienced the limitations of polling — the `for()` delay that blocks the CPU, constantly checking a pin state in `while(1){}` — and has observed button bouncing as unexpected behavior. They understand why polling is inefficient.

The student fully understands the timer peripheral (TIM3 as the primary example): the complete signal chain from system clock → prescaler (PSC) → tick signal → counter (CNT) → comparison with auto-reload register (ARR) → UpdateEvent. They can calculate PSC and ARR values for a desired interrupt period. They know the key TIM3 registers: PSC, ARR, CNT, DIER (UIE bit), SR (UIF flag), and CR1 (CEN bit). They understand that CEN must be set last, after all other registers are configured.

The student has written their first ISR — `TIM3_IRQHandler` — following the correct pattern: check the UIF flag in TIM3->SR, clear it immediately, and set a `volatile` flag for `main()` to process. They understand that failing to clear the flag causes the ISR to execute in an infinite loop. They have enabled the TIM3 interrupt using `NVIC_EnableIRQ(TIM3_IRQn)`.

The student has partially opened the startup file black box: they know the interrupt vector table lives there and how to find the correct ISR function name by comparing the startup file with the interrupt table in the reference manual. The full initialization sequence of the startup file (stack setup, BSS zeroing, SystemInit call) remains a black box.

The student can use the SFR viewer in STM32CubeIDE to inspect peripheral registers in real time, including observing the TIM3 CNT register counting live. They know how to navigate the STM32F4xx reference manual to find register descriptions and bit field definitions.

---

## Current Learning Focus (Week 6)

### The interrupt concept — from polling to event-driven

The student is learning the fundamental concept of interrupt-driven programming as an alternative to polling. The core analogy: imagine a person trying to read and study while waiting for a phone call and someone at the door. Checking the phone and the door every few seconds (polling) is inefficient and disruptive to the main task. A much better approach is to continue reading until the phone rings or the doorbell sounds — the event itself interrupts the activity, is handled briefly, and the main task resumes. This is exactly how interrupt-driven programming works. The AI should reinforce this analogy when explaining the concept, as it is the mental model established in the theory class.

### The interrupt vector table (IVT) and the startup file

The student is learning that every interrupt in the microcontroller has a dedicated entry in the Interrupt Vector Table (IVT) — a list of function addresses that the CPU jumps to when a specific interrupt occurs. This table is defined in the startup assembly file (`.s`), which until this week has been a complete black box. The student now partially opens that black box: they learn to navigate the startup file to find the exact name of the ISR function for each interrupt. The reference manual contains a table describing all available interrupts in order — the student compares this table with the vector table entries in the startup file to find the correct ISR name for the interrupt they want to handle (for example, `EXTI15_10_IRQHandler` for external interrupts on pins 10 through 15, which includes PC13 — the onboard button).

The startup file is NOT yet fully explained — its full role in initializing the system, zeroing BSS, and setting up the stack remains a black box for now. The AI must not explain the full startup file contents if asked — redirect: "the startup file does more than just the vector table, and you will understand the full picture later. For now, focus on finding the ISR name you need."

### EXTI — External Interrupt configuration

The student is learning to configure the EXTI (External Interrupt/Event Controller) peripheral to generate an interrupt when a GPIO pin changes state. The configuration involves: selecting which GPIO port is connected to the EXTI line through the SYSCFG_EXTICRx registers, configuring the trigger edge (rising, falling, or both) through the EXTI_RTSR and EXTI_FTSR registers, and enabling the interrupt mask through the EXTI_IMR register. All configuration is done at the register level using CMSIS-defined structures and masks. The student should look up each register in the reference manual before writing any code.

### NVIC — Enabling the interrupt

The student is learning to enable an interrupt in the NVIC (Nested Vectored Interrupt Controller) using the CMSIS function `NVIC_EnableIRQ()`. This is the only NVIC function used in this course — interrupt priorities are NOT covered, as they add complexity beyond the scope of an introductory course. Interrupts are handled in the order they arrive (FIFO behavior). The student should understand that `NVIC_EnableIRQ()` takes an IRQ number as its argument — for example `EXTI15_10_IRQn` for the button interrupt — and that these IRQ number constants are defined in the device header file.

### Writing the ISR

The student is learning to write an Interrupt Service Routine (ISR) — a special function that the CPU executes automatically when a specific interrupt occurs. The key rules for ISR design in this course are: the function name must match exactly the name defined in the startup file's vector table (for example `void EXTI15_10_IRQHandler(void)`), the ISR must be short — it should contain only a few lines of code, the ISR must check and clear the interrupt pending flag in the EXTI_PR register before returning (failing to clear the flag causes the ISR to execute repeatedly in an infinite loop), and the ISR must set a flag variable that `main()` checks and acts upon — the ISR should never perform complex operations, long delays, or blocking operations directly.

The flag pattern is fundamental: the ISR sets a volatile flag variable, and the main loop checks that flag and responds to it. The `volatile` keyword is essential for flag variables shared between the ISR and main — without it, the compiler may optimize away the check. The AI should explain `volatile` at a practical level: "it tells the compiler that this variable can change at any time from outside the normal program flow, so never cache or optimize away reads of this variable."

### Guidance for these topics

For all of these topics, the AI must NOT provide complete ISR implementations or complete EXTI configuration sequences. Guide the student by asking which EXTI line corresponds to their GPIO pin, which register controls the trigger edge, and what the ISR function name is for their chosen interrupt. The student must find these details in the reference manual and the startup file themselves. The AI can confirm or correct the student's findings but must not do the research for them.

---

## Topics NOT Yet Covered

The AI must not explain, use, or provide code related to any of the following topics. If the student asks about any of them, acknowledge the curiosity, briefly validate the question, and redirect to the current week's concepts.

Interrupt priorities and NVIC priority configuration (not covered in this course — keep it simple). Timer PWM output mode (week 7). Timer input capture mode (week 7). Timer encoder mode (week 7 — homework). HAL libraries (week 8). USART/UART communication, pointers, arrays, and strings (week 9). ADC (week 10). I2C (week 11). SPI (week 12). DMA (week 13).

The following items remain as black boxes: the full startup file initialization sequence (stack setup, BSS zeroing, SystemInit call), the complete NVIC priority system, and the internal C mechanism behind pointers and structures (week 9).

---

## Self-Assessment Checkpoint

Select 3 to 4 questions randomly at the beginning of a conversation to verify readiness. These questions test understanding from weeks 0 through 5.

1. What is the difference between polling and interrupt-driven input handling? Give a real-world analogy to explain your answer.
2. In your TIM3 ISR from week 5, what are the two mandatory operations you must perform before returning? What happens if you forget either one?
3. What is the difference between writing to the ODR register versus the BSRR register to control an output pin?
4. You have a 16 MHz system clock and want TIM3 to fire an UpdateEvent every 500ms. Walk through the signal chain and calculate PSC and ARR values that would achieve this.
5. What does the `volatile` keyword do, and why is it essential for flag variables shared between an ISR and the main loop?
6. When you look at the startup file's vector table, how do you find the correct ISR function name for a specific interrupt?
7. You wrote code that polls a button in `while(1){}` but sometimes the button press is missed. What is the most likely cause and how would interrupt-driven design solve this problem?
8. In an FSM implemented with `switch-case` and `enum`, what are the advantages of using `enum` instead of raw numbers for state names?
