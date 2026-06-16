# Homework Exercises — Week 06
## STM32F4xx Embedded Systems Course
### External Interrupts (EXTI) and 7-Segment Display

This week combines two powerful new capabilities: EXTI-driven external interrupts for responding to button events asynchronously, and a 4-digit 7-segment display driven by a dedicated multiplexing timer. Both build directly on the timer and interrupt skills from week 05.

The timer assignment policy — one task, one timer — is strictly enforced this week with four active timers running simultaneously in most exercises:

- **TIM2** — dedicated to LED_OK at 1 Hz. Never reassigned.
- **TIM3** — main exercise logic (counting, time measurement, etc.)
- **TIM4** — 7-segment display multiplexing at approximately 120 Hz (4 digits × 30 frames per second)
- **EXTI lines** — button events, handled asynchronously through ISRs

Every button in this week's exercises is hardware-debounced using a Schmitt Trigger circuit. No software debouncing is needed or expected — trust the hardware. Every ISR must still follow the mandatory pattern: check the flag, clear it immediately, set a `volatile` flag for `main()`.

The 7-segment display uses 7 segment pins (a through g) and 4 digit selection pins connected through transistors. At any moment, only one digit is physically active. The display timer cycles through all four digits fast enough that the human eye perceives them as simultaneously lit — this is hardware multiplexing. Your display driver must maintain a 4-element array holding the current value for each digit position, and the TIM4 ISR activates one digit per interrupt, cycling through all four in sequence.

Before starting any exercise, make sure your display driver is working correctly with a static test value. A broken display driver will make every exercise impossible to debug.

---

### Exercise 6.1 — Button press counter on 7-segment display

Configure one button on an EXTI line. Every time the button is pressed, increment a counter variable. Display the current counter value on the 4-digit 7-segment display in decimal format. LED_OK blinks on TIM2 at 1 Hz throughout.

The EXTI line should trigger on the falling edge (button press) or rising edge (button release) — choose one and be consistent. Find the correct ISR name in the startup file's vector table, verify it against the interrupt table in the reference manual, and implement the ISR following the mandatory flag pattern.

The counter display requires converting a binary number into individual decimal digits — for example, the value 1234 must be split into digits 1, 2, 3, and 4 to drive the four display positions. Think about how to extract each decimal digit using division and modulus operations. This is a direct application of the arithmetic skills from week 01.

After the counter is working, test the behavior carefully: press the button rapidly several times. Does the counter increment correctly every time, or does it sometimes skip or double-count? Remember the button is hardware-debounced — if you still see unexpected behavior, the issue is in your ISR or flag handling, not in the button hardware. Use the debugger to investigate.

---

### Exercise 6.2 — Up/down counter with two EXTI buttons

Extend exercise 6.1 by adding a second button on a separate EXTI line. One button increments the counter, the other decrements it. The display shows the current value at all times. Define minimum and maximum bounds for the counter — for example 0 to 9999 — and prevent it from going below the minimum or above the maximum.

Each button needs its own EXTI line and its own ISR. Think carefully about the naming: each ISR name comes from the startup file's vector table and depends on which GPIO pin you use for each button. Two buttons on pins in the same EXTI group (for example EXTI0 and EXTI1) have separate ISRs. Two buttons sharing the same EXTI group (for example EXTI0 and EXTI5 — both handled by different IRQs) also have separate ISRs. Check the reference manual carefully.

After the system is working, observe what happens when you press both buttons simultaneously. Does the counter behave predictably? Write your observation as a comment — this edge case connects to interrupt priority concepts that are beyond this course but worth being aware of.

---

### Exercise 6.3 — Elapsed time measurement between two button presses

Configure two buttons on separate EXTI lines. When the first button is pressed, start TIM3 counting from zero. When the second button is pressed, stop TIM3 and display the elapsed time on the 7-segment display in milliseconds. LED_OK continues on TIM2 at 1 Hz.

Configure TIM3 with a tick frequency of 1 KHz — one tick per millisecond — so that the CNT register directly represents elapsed milliseconds. When the second button fires its EXTI interrupt, read TIM3->CNT immediately and store the value. This is a time-critical read — the sooner you capture CNT after the interrupt fires, the more accurate your measurement will be.

Think carefully about the system states: what happens if the second button is pressed before the first? What happens if the first button is pressed twice? Design your FSM to handle these cases sensibly. Draw the state diagram before writing any code.

After the system is working, test it by measuring a known time interval — for example, press the first button and count to one second before pressing the second. Does the display show approximately 1000ms? What factors might affect the accuracy of your measurement? Think about interrupt latency, the time between the physical button press and the ISR execution, and how the Schmitt Trigger debounce circuit might introduce a small delay.

---

### Exercise 6.4 — Display refresh rate explorer

This exercise deliberately breaks the display to teach the concept of persistence of vision and multiplexing. Configure the system with LED_OK on TIM2, a static value displayed on the 4-digit display using TIM4, and two buttons on EXTI lines — one to decrease the display refresh rate and one to increase it back.

Start with TIM4 configured at the normal 120 Hz refresh rate. Each press of the decrease button reduces the refresh rate significantly — for example halving it each time. Each press of the increase button restores the previous rate or increases toward the normal rate. Display the current refresh rate on the display itself so the student can see what frequency is active.

As the refresh rate decreases, observe carefully what happens to the display appearance. At what frequency does the display start to flicker noticeably? At what frequency can you clearly see only one digit active at a time? At what frequency does the display become completely unreadable? Record your observations at each step as comments in the code.

This exercise has no "correct" hardware behavior — the goal is observation and understanding. The persistence of vision threshold is approximately 50 Hz for most people, but individual variation exists. What you are observing is the same phenomenon that makes cinema and television work — a sequence of still images played fast enough to appear as continuous motion.

After completing the observations, restore the normal refresh rate and verify that the display returns to normal operation. Then think about this question: what is the minimum acceptable refresh rate for a display that will be used in a well-lit environment versus a dimly lit environment? Write your reasoning as a comment.

---

### Exercise 6.5 — Reaction time meter

Build a complete reaction time measurement system using everything learned this week. The system works as follows: after the user presses a start button, the system waits for a random delay between 2 and 5 seconds, then lights a reaction LED. The user must press a response button as quickly as possible after seeing the LED. The system measures the time between the LED turning on and the button press, and displays the reaction time in milliseconds on the 4-digit display. LED_OK blinks on TIM2 at 1 Hz throughout.

Timer and EXTI assignment:
- TIM2 — LED_OK at 1 Hz
- TIM3 — random delay countdown and reaction time measurement
- TIM4 — display multiplexing at 120 Hz
- EXTI — start button and response button on separate lines

The random delay does not need to be truly random — a simple pseudo-random approach is acceptable. For example, use a counter variable that increments every TIM3 tick while waiting for the start button press. When the start button is pressed, the current counter value modulo a suitable range gives the waiting period. This is not cryptographically random but is unpredictable enough for a reaction time test.

Design your complete FSM before writing any code. The system has at least four states: IDLE (waiting for start button), WAITING (counting down the random delay), ACTIVE (reaction LED on, waiting for response button), and RESULT (displaying the reaction time). Draw the full state diagram with all transitions and outputs for each state. Consider what happens in edge cases: what if the response button is pressed during the WAITING state (before the LED turns on)? This is called a "false start" — define how your system handles it.

After the system is working, test it multiple times and observe your own reaction times. A typical human reaction time to a visual stimulus is between 150ms and 300ms. If your measurements are consistently outside this range, investigate whether the issue is in your time measurement, your display conversion, or genuinely in your reaction speed. This self-verification habit — questioning whether your measurements make physical sense — is one of the most important skills in embedded systems development.
