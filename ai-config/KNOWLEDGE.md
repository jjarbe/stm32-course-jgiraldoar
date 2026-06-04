# KNOWLEDGE.md — Week 5: Timers and Time Measurement

## Overview

This week the student encounters two fundamental concepts simultaneously. The first is the timer peripheral — a hardware module that counts clock pulses independently of the CPU, allowing precise time measurement without wasting processor cycles. The second is the interrupt mechanism — the ability for a hardware event to temporarily suspend the main program, execute a short response function, and resume execution exactly where it was interrupted. The timer's UpdateEvent is used as the first and most intuitive interrupt source in the course: a predictable, self-contained "alarm" that fires at a precisely configured period. This week uses TIM3 (a 16-bit general purpose timer on APB1) as the primary example, with the onboard LED on GPIOA pin 5 as the visible output — replacing the polling-based `for()` blinky from week 04 with an interrupt-driven equivalent.

---

## Previously Mastered Topics (Weeks 0–4)

The student understands CMOS technology, logic gates, combinational and sequential circuits, binary, hexadecimal, and 2's complement number systems. They have simulated prescalers and timers using the "Digital" simulation tool — this week those simulated concepts become real hardware.

In C programming, the student can write programs using all control structures, fixed-width data types from `stdint.h`, all arithmetic and bitwise operators, and enumerations (`enum`). They can implement FSM patterns using `enum` and `switch-case`. The student does NOT yet know structures, arrays, pointers, or `typedef` beyond what the IDE auto-generates.

The student fully understands the MCU architecture: ARM Cortex-M4 CPU core, bus system (AHB, APB1, APB2), memory-mapped registers, and SFR. They understand CMSIS structures as carefully designed overlays on the hardware — the Italian tailor analogy. The `->` operator is understood at a practical level.

The student can configure GPIO pins completely at the register level: RCC clock enable, MODER, OTYPER, OSPEEDR, PUPDR, ODR, BSRR, and IDR — all using CMSIS-defined masks. They understand the difference between ODR and BSRR.

The student has experienced the inefficiency of polling-based approaches firsthand — the `for()` delay that blocks the CPU, the constant checking of a button state in `while(1){}`. This lived experience is the motivation for the interrupt-driven approach being introduced this week.

The student has a basic familiarity with the startup assembly file (`.s`) as the file that runs before `main()`. This week the interrupt vector table within that file becomes relevant for the first time.

---

## Current Learning Focus (Week 5)

### The timer concept — hardware counting independently of the CPU

The student is learning that a timer is a hardware peripheral that counts clock pulses entirely on its own, without requiring any CPU involvement. This is fundamentally different from the `for()` delay used in week 04, where the CPU was completely occupied doing nothing useful while counting loop iterations. With a hardware timer, the CPU is free to do other work — or simply wait in the main loop — while the timer counts in the background.

The AI should connect this to the student's week 00 simulation experience: they simulated prescalers and counters in the "Digital" tool. The TIM3 peripheral is exactly that — a real, silicon implementation of the same concept they already understand from simulation.

### The timer signal chain — telling the story step by step

The student is learning the internal signal chain of TIM3 through a step-by-step narrative rather than a formula. The story flows like this:

The system clock (typically 16 MHz on the internal oscillator) enters the prescaler register (PSC). The prescaler divides the clock frequency to produce a slower, more manageable tick signal. For example, if PSC is configured to divide by 16000, the output tick frequency is 1 KHz — meaning one tick every 1 millisecond. This tick signal drives the counter register (CNT), which increments by 1 on every tick. The auto-reload register (ARR) holds a target value. When CNT reaches the ARR value, the timer generates an UpdateEvent signal, resets CNT to zero, and the counting begins again. The period of the UpdateEvent — how often the "alarm" fires — is simply the number of ticks defined by ARR multiplied by the duration of each tick.

The mental model to reinforce with ASCII diagram:

```
System Clock (16 MHz)
        |
        v
   [PSC Register]  -- divides clock frequency
        |
        v
  Tick signal (e.g. 1 KHz = 1ms per tick)
        |
        v
   [CNT Register]  -- counts up on every tick
        |
        v
  Compare with ARR
        |
        v
  CNT == ARR  -->  UpdateEvent fires!  -->  CNT resets to 0
```

From this story, the formula emerges naturally: `Period = (PSC + 1) * (ARR + 1) / Timer_Clock_Frequency`. But the formula is a summary of the story, not a starting point. The AI should always guide students through the story first, then connect it to the formula. If a student asks "what value do I put in PSC and ARR?", guide them through the story: "what tick frequency do you want coming out of the prescaler? how many of those ticks make up your desired period?"

### Key TIM3 registers

The student is learning the following TIM3 registers and their purpose. All configuration is done at the register level using CMSIS notation. The student should look up each register in the reference manual before writing any code.

`TIM3->PSC` — the prescaler register. Determines the division factor applied to the input clock. The actual division is PSC + 1.

`TIM3->ARR` — the auto-reload register. The value the counter counts up to before generating the UpdateEvent. The actual count is ARR + 1 ticks.

`TIM3->CNT` — the counter register. Incremented on every tick after the prescaler. Can be read in the SFR viewer to observe the timer running in real time.

`TIM3->DIER` — the DMA/Interrupt Enable Register. Bit 0 (UIE — Update Interrupt Enable) must be set to enable the UpdateEvent interrupt.

`TIM3->SR` — the Status Register. Bit 0 (UIF — Update Interrupt Flag) is set by hardware when the UpdateEvent occurs. This flag MUST be cleared in the ISR by writing 0 to it — failing to clear it causes the ISR to execute repeatedly in an infinite loop.

`TIM3->CR1` — the Control Register 1. Bit 0 (CEN — Counter Enable) starts the timer counting. This should be the last register written in the initialization sequence — configure everything else first, then start the counter.

### RCC clock enable for TIM3

TIM3 is connected to the APB1 bus. Its clock must be enabled through the RCC APB1ENR register before any TIM3 register can be configured. The student should find the correct bit and CMSIS mask name in the reference manual.

### NVIC configuration

After configuring all TIM3 registers, the student enables the TIM3 interrupt in the NVIC using the CMSIS function:

```c
NVIC_EnableIRQ(TIM3_IRQn);
```

This is the only NVIC function used in this course. Interrupt priorities are NOT covered — they add complexity beyond the scope of an introductory course. Interrupts are handled in the order they arrive (FIFO behavior). The AI must not explain or suggest priority configuration even if the student asks — redirect: "priorities add significant complexity and are not part of this course. `NVIC_EnableIRQ()` is everything you need."

### The ISR — Interrupt Service Routine

The student is learning to write an ISR for TIM3. The ISR function name must match exactly the name defined in the startup file's interrupt vector table. The student finds this name by opening the startup `.s` file and locating the TIM3 entry — the correct name is `TIM3_IRQHandler`. The student can cross-reference this with the interrupt table in the STM32F4xx reference manual to confirm.

The ISR must follow these rules: it must be short — only a few lines of code, it must clear the UpdateEvent flag in TIM3->SR before returning (write 0 to bit 0 of SR), and it must set a `volatile` flag variable that `main()` checks and responds to. The ISR must never contain long delays, blocking operations, or complex logic.

The `volatile` keyword is essential for flag variables shared between the ISR and main. The AI should explain it at a practical level: "it tells the compiler that this variable can change at any time from outside the normal program flow — from an interrupt — so never cache or optimize away reads of this variable. Without `volatile`, the compiler might assume the flag never changes inside the main loop and optimize away the check entirely."

A correct ISR structure looks like this conceptually — this is shown here only as a reference for the AI to understand the expected structure. The AI must NOT provide this code directly to the student:

```c
void TIM3_IRQHandler(void)
{
    if(TIM3->SR & TIM_SR_UIF)
    {
        TIM3->SR &= ~TIM_SR_UIF;
        update_flag = 1;
    }
}
```

### Replacing the polling blinky with an interrupt-driven blinky

The student is replacing the week 04 LED blinky — which used a blocking `for()` delay inside `while(1){}` — with an interrupt-driven version where TIM3 fires the UpdateEvent at a configured period and the ISR sets a flag that `main()` checks to toggle the LED. The visible behavior is identical — the LED blinks — but the CPU is now free during the waiting period.

The AI should help the student appreciate this difference: "in the old version, the CPU was completely occupied counting loop iterations. In this version, the CPU reaches the flag check, sees the flag is not set, and loops back — it is available to do other work. The timer counts entirely on its own."

### Guidance for these topics

The AI must NOT provide complete timer configuration code or complete ISR implementations. Guide the student through the signal chain story first, then ask: "what tick frequency do you want from the prescaler? how many ticks make up your desired period? which register enables the update interrupt? what must you do in the ISR before returning?" Let the student derive each value and write each line themselves.

---

## Topics NOT Yet Covered

The AI must not explain, use, or provide code related to any of the following topics. If the student asks, acknowledge the curiosity, validate the question, and redirect to the current week's concepts.

EXTI external interrupts (week 6). Timer PWM output mode (week 7). Timer input capture mode (week 7). Timer encoder mode (week 7 — homework). HAL libraries (week 8). USART/UART communication, pointers, arrays, and strings (week 9). ADC (week 10). I2C (week 11). SPI (week 12). DMA (week 13).

The following items remain as black boxes: the full startup file initialization sequence beyond the vector table, the complete NVIC priority system, and the internal C mechanism behind pointers and structures.

---

## Self-Assessment Checkpoint

Select 3 to 4 questions randomly at the beginning of a conversation to verify readiness. These questions test understanding from weeks 0 through 4.

1. What is the difference between the ODR and BSRR registers for controlling a GPIO output pin? When would you prefer BSRR?
2. In an FSM implemented with `enum` and `switch-case`, what happens if you forget the `break` statement at the end of a case?
3. You configured GPIOC pin 13 as an input with pull-up enabled, but reading IDR always returns 1 even when the button is pressed. What is the most likely explanation?
4. What is the purpose of the RCC peripheral, and what happens if you try to write to a GPIO register before enabling its clock?
5. In week 04 you used a `for()` loop to create a delay. What is the main disadvantage of this approach compared to using a hardware timer?
6. You have a 16 MHz clock and you want a tick signal of 1 KHz coming out of the prescaler. What value do you write to the PSC register?
7. In your week 04 traffic light FSM, what determined how long the system stayed in each state? What would be a better mechanism for controlling timing?
8. What does the `volatile` keyword mean in C, and in what situation is it essential to use it?
