# Ejercicios de Practica — Semana 06
## Curso de Sistemas Embebidos STM32F4xx
### Interrupciones Externas (EXTI) y Display de 7 Segmentos

Esta semana combina dos nuevas capacidades poderosas: interrupciones externas impulsadas por EXTI para responder a eventos de botones de forma asíncrona, y un display de 7 segmentos de 4 dígitos impulsado por un temporizador de multiplexión dedicado. Ambas se construyen directamente sobre las habilidades de temporizadores e interrupciones de la semana 05.

La política de asignación de temporizadores — una tarea, un temporizador — se aplica estrictamente esta semana con cuatro temporizadores activos funcionando simultáneamente en la mayoría de los ejercicios:

- **TIM2** — dedicado a LED_OK a 1 Hz. Nunca reasignado.
- **TIM3** — lógica principal del ejercicio (conteo, medición de tiempo, etc.)
- **TIM4** — multiplexión del display de 7 segmentos a aproximadamente 120 Hz (4 dígitos × 30 fotogramas por segundo)
- **Líneas EXTI** — eventos de botones, manejados asíncronamente a través de ISRs

Cada botón en los ejercicios de esta semana está eliminado el rebote por hardware mediante un circuito Schmitt Trigger. No se necesita ni se espera ninguna eliminación de rebote por software — confía en el hardware. Cada ISR debe seguir el patrón obligatorio: verificar el flag, limpiarlo inmediatamente, establecer un flag `volatile` para que `main()` lo procese.

El display de 7 segmentos usa 7 pines de segmento (a hasta g) y 4 pines de selección de dígito conectados a través de transistores. En cualquier momento, solo un dígito está físicamente activo. El temporizador del display cicla por los cuatro dígitos lo suficientemente rápido como para que el ojo humano los perciba como simultáneamente encendidos — esto es multiplexión por hardware. Tu driver del display debe mantener un arreglo de 4 elementos que contenga el valor actual para cada posición de dígito, y la ISR de TIM4 activa un dígito por interrupción, ciclando por los cuatro en secuencia.

Antes de comenzar cualquier ejercicio, asegúrate de que tu driver del display funciona correctamente con un valor de prueba estático. Un driver de display roto hará que cada ejercicio sea imposible de depurar.

---

### Ejercicio 6.1 — Contador de pulsaciones de botón en display de 7 segmentos

Configura un botón en una línea EXTI. Cada vez que se presiona el botón, incrementa una variable contadora. Muestra el valor actual del contador en el display de 7 segmentos de 4 dígitos en formato decimal. LED_OK parpadea en TIM2 a 1 Hz durante todo el ejercicio.

La línea EXTI debe dispararse en el flanco de bajada (presión del botón) o en el flanco de subida (liberación del botón) — elige uno y sé consistente. Encuentra el nombre correcto de la ISR en la tabla de vectores del archivo de inicio, verifícalo con la tabla de interrupciones en el manual de referencia, e implementa la ISR siguiendo el patrón obligatorio de flag.

La visualización del contador requiere convertir un número binario en dígitos decimales individuales — por ejemplo, el valor 1234 debe dividirse en los dígitos 1, 2, 3 y 4 para controlar las cuatro posiciones del display. Piensa en cómo extraer cada dígito decimal usando operaciones de división y módulo. Esta es una aplicación directa de las habilidades aritméticas de la semana 01.

Después de que el contador funcione, prueba el comportamiento cuidadosamente: presiona el botón rápidamente varias veces. ¿El contador se incrementa correctamente cada vez, o a veces salta o cuenta doble? Recuerda que el botón tiene eliminación de rebote por hardware — si aún ves comportamientos inesperados, el problema está en tu ISR o en el manejo del flag, no en el hardware del botón. Usa el depurador para investigar.

---

### Ejercicio 6.2 — Contador ascendente/descendente con dos botones EXTI

Extiende el ejercicio 6.1 añadiendo un segundo botón en una línea EXTI separada. Un botón incrementa el contador, el otro lo decrementa. El display muestra el valor actual en todo momento. Define límites mínimo y máximo para el contador — por ejemplo de 0 a 9999 — y evita que vaya por debajo del mínimo o por encima del máximo.

Cada botón necesita su propia línea EXTI y su propia ISR. Piensa cuidadosamente en la nomenclatura: cada nombre de ISR proviene de la tabla de vectores del archivo de inicio y depende de qué pin GPIO uses para cada botón. Dos botones en pines del mismo grupo EXTI tienen ISRs separadas. Verifica el manual de referencia cuidadosamente.

Después de que el sistema funcione, observa qué ocurre cuando presionas ambos botones simultáneamente. ¿Se comporta el contador de forma predecible? Escribe tu observación como un comentario — este caso límite se conecta con conceptos de prioridad de interrupciones que están más allá de este curso pero que vale la pena conocer.

---

### Ejercicio 6.3 — Medición de tiempo transcurrido entre dos pulsaciones de botón

Configura dos botones en líneas EXTI separadas. Cuando se presiona el primer botón, inicia TIM3 contando desde cero. Cuando se presiona el segundo botón, detén TIM3 y muestra el tiempo transcurrido en el display de 7 segmentos en milisegundos. LED_OK continúa en TIM2 a 1 Hz.

Configura TIM3 con una frecuencia de tick de 1 KHz — un tick por milisegundo — de modo que el registro CNT represente directamente los milisegundos transcurridos. Cuando el segundo botón dispara su interrupción EXTI, lee TIM3->CNT inmediatamente y almacena el valor. Esta es una lectura crítica en tiempo — cuanto antes captures CNT después de que la interrupción dispare, más precisa será tu medición.

Piensa cuidadosamente en los estados del sistema: ¿qué ocurre si el segundo botón se presiona antes que el primero? ¿Qué ocurre si el primer botón se presiona dos veces? Diseña tu FSM para manejar estos casos de manera sensata. Dibuja el diagrama de estados antes de escribir cualquier código.

Después de que el sistema funcione, pruébalo midiendo un intervalo de tiempo conocido — por ejemplo, presiona el primer botón y cuenta hasta un segundo antes de presionar el segundo. ¿Muestra el display aproximadamente 1000ms? ¿Qué factores podrían afectar la precisión de tu medición? Piensa en la latencia de la interrupción, el tiempo entre la presión física del botón y la ejecución de la ISR, y cómo el circuito Schmitt Trigger podría introducir un pequeño retardo.

---

### Ejercicio 6.4 — Explorador de frecuencia de refresco del display

Este ejercicio rompe deliberadamente el display para enseñar el concepto de persistencia de la visión y la multiplexión. Configura el sistema con LED_OK en TIM2, un valor estático mostrado en el display de 4 dígitos usando TIM4, y dos botones en líneas EXTI — uno para disminuir la frecuencia de refresco del display y otro para aumentarla de vuelta.

Comienza con TIM4 configurado a la frecuencia de refresco normal de 120 Hz. Cada pulsación del botón de disminución reduce significativamente la frecuencia de refresco — por ejemplo dividiéndola a la mitad cada vez. Cada pulsación del botón de aumento restaura la velocidad anterior o aumenta hacia la frecuencia normal. Muestra la frecuencia de refresco actual en el display para que puedas ver qué frecuencia está activa.

A medida que la frecuencia de refresco disminuye, observa cuidadosamente qué le ocurre a la apariencia del display. ¿A qué frecuencia empieza a parpadear el display notablemente? ¿A qué frecuencia puedes ver claramente solo un dígito activo a la vez? ¿A qué frecuencia el display se vuelve completamente ilegible? Registra tus observaciones en cada paso como comentarios en el código.

Este ejercicio no tiene un "comportamiento de hardware correcto" — el objetivo es la observación y la comprensión. El umbral de persistencia de la visión es de aproximadamente 50 Hz para la mayoría de las personas, pero existe variación individual. Lo que estás observando es el mismo fenómeno que hace funcionar el cine y la televisión — una secuencia de imágenes fijas reproducidas lo suficientemente rápido como para parecer movimiento continuo.

Después de completar las observaciones, restaura la frecuencia de refresco normal y verifica que el display vuelva a la operación normal. Luego piensa en esta pregunta: ¿cuál es la frecuencia de refresco mínima aceptable para un display que se usará en un entorno bien iluminado versus uno con poca luz? Escribe tu razonamiento como un comentario.

---

### Ejercicio 6.5 — Medidor de tiempo de reacción

Construye un sistema completo de medición de tiempo de reacción usando todo lo aprendido esta semana. El sistema funciona así: después de que el usuario presiona un botón de inicio, el sistema espera un retardo aleatorio entre 2 y 5 segundos, luego enciende un LED de reacción. El usuario debe presionar un botón de respuesta lo más rápido posible después de ver el LED. El sistema mide el tiempo entre el encendido del LED y la pulsación del botón, y muestra el tiempo de reacción en milisegundos en el display de 4 dígitos. LED_OK parpadea en TIM2 a 1 Hz durante todo el proceso.

Asignación de temporizadores y EXTI:
- TIM2 — LED_OK a 1 Hz
- TIM3 — cuenta regresiva del retardo aleatorio y medición del tiempo de reacción
- TIM4 — multiplexión del display a 120 Hz
- EXTI — botón de inicio y botón de respuesta en líneas separadas

El retardo aleatorio no necesita ser verdaderamente aleatorio — un enfoque pseudo-aleatorio simple es aceptable. Por ejemplo, usa una variable contadora que se incrementa en cada tick de TIM3 mientras espera la pulsación del botón de inicio. Cuando se presiona el botón de inicio, el valor actual del contador módulo un rango adecuado da el período de espera. Esto no es aleatorio criptográficamente pero es suficientemente impredecible para una prueba de tiempo de reacción.

Diseña tu FSM completa antes de escribir cualquier código. El sistema tiene al menos cuatro estados: IDLE (esperando el botón de inicio), WAITING (contando el retardo aleatorio), ACTIVE (LED de reacción encendido, esperando el botón de respuesta), y RESULT (mostrando el tiempo de reacción). Dibuja el diagrama de estados completo con todas las transiciones y salidas para cada estado. Considera qué ocurre en casos límite: ¿qué pasa si el botón de respuesta se presiona durante el estado WAITING (antes de que el LED se encienda)? Esto se llama "salida en falso" — define cómo tu sistema lo maneja.

Después de que el sistema funcione, pruébalo varias veces y observa tus propios tiempos de reacción. Un tiempo de reacción humano típico a un estímulo visual está entre 150ms y 300ms. Si tus mediciones están consistentemente fuera de este rango, investiga si el problema está en tu medición de tiempo, en tu conversión del display, o genuinamente en tu velocidad de reacción. Este hábito de auto-verificación — cuestionar si tus mediciones tienen sentido físico — es una de las habilidades más importantes en el desarrollo de sistemas embebidos.
