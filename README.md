# VAE - TRIPLE-R: Rover Autónomo Ackermann con Control en Cascada

Sistema de control autónomo para un vehículo RC escala 1/10 con geometría Ackermann. La arquitectura integra simulación en MATLAB/Simulink, visión artificial y control de dirección discreto en una cámara OpenMV H7, y control de velocidad PI con estimación espectral (FFT) en una ESP32-S3.

---

## 1. Arquitectura del Sistema

El sistema opera bajo un esquema de control en cascada distribuido en dos nodos:

1. **Lazo externo (Dirección y Visión - OpenMV H7):** Captura la pista en escala de grises ($160 \times 60$ px), segmenta la cinta negra mediante tres franjas horizontales con validación de contraste local, calcula el error geométrico Ackermann $y_k = \operatorname{atan2}(e_{\text{lat}}, L_{\text{rover}})$, ejecuta el controlador discreto seleccionado (`P`, `PI`, `PD` o `PID`) para comandar el servo de dirección y transmite el estado por UART.
2. **Lazo interno (Velocidad - ESP32-S3):** Recibe la trama UART para modular continuamente la velocidad de referencia entre `V_RECTA` y `V_CURVA` en función del ángulo de giro y la curvatura anticipada. Mide la velocidad real del motor BLDC mediante una FFT Radix-2 sobre la señal del comparador de corriente y ejecuta un controlador PI discreto a $2\text{ kHz}$ hacia el ESC.

---

## 2. Requisitos de Hardware y Software

### Hardware
* Chasis RC 1/10 Ackermann ($L = 0.23965\text{ m}$, diámetro de rueda $D = 0.109\text{ m}$, relación de transmisión $10:1$).
* Motor BLDC BS214-008 (4 polos, $p = 2$) y ESC Hobbywing WP-10BL50-RTR.
* Circuito comparador de corriente (salida digital $3.3\text{ V}$) para medición de frecuencia eléctrica.
* Microcontrolador ESP32-S3.
* Cámara OpenMV H7 (firmware **v5.0.1**).

### Software
* MATLAB y Simulink (para validación previa y cálculo de índices de error).
* OpenMV IDE (compatible con MicroPython v1.28 / OpenMV v5.0.1).
* Arduino IDE o PlatformIO con soporte para ESP32-S3 y librería `ESP32Servo`.

---

## 3. Mapa de Conexiones

Asegure una referencia de tierra (`GND`) común entre la OpenMV H7, la ESP32-S3, el comparador y la señal del ESC/servo.

| Origen | Pin | Destino | Pin | Función |
| :--- | :--- | :--- | :--- | :--- |
| OpenMV H7 | `P4` (UART3 TX) | ESP32-S3 | `GPIO 18` (RX1) | Telemetría en cascada ($115200$ baudios) |
| OpenMV H7 | `P7` (Timer4 CH1) | Servo de dirección | Señal PWM | Control de ángulo Ackermann ($50\text{ Hz}$) |
| ESP32-S3 | `GPIO 13` | ESC WP-10BL50-RTR | Señal PWM | Control de velocidad ($50\text{ Hz}$, $1000\text{--}2000\ \mu\text{s}$) |
| Comparador | Salida $3.3\text{ V}$ | ESP32-S3 | `GPIO 4` (`TACHO_PIN`) | Pulsos de fase para estimación FFT |
| ESP32-S3 | `GPIO 17` (TX1) | OpenMV H7 | `P5` (UART3 RX) | Canal de retorno opcional |

---

## 4. Procedimiento de Implementación

### Paso 1: Validación en Simulación (MATLAB/Simulink)
1. Coloque `ejecutar_rover.m` y `Simulacion_Rover.slx` en el mismo directorio de trabajo de MATLAB.
2. En `ejecutar_rover.m`, seleccione los controladores en `modo_control` (velocidad) y `modo_dir` (dirección): `1 = P`, `2 = PD`, `3 = PI`, `4 = PID`, `5 = Relé`.
3. Configure el tipo de trayectoria (`1 = Circular`, `2 = Infinito`) y las ganancias 2-DOF.
4. Ejecute `ejecutar_rover.m`. El script corre el modelo y muestra en consola la tabla comparativa de desempeño (`RMSE`, `IAE`, `ISE`, `ITAE`) para ambos lazos.

### Paso 2: Programación de la OpenMV H7
1. Verifique que la cámara cuente con el firmware **v5.0.1** (el script utiliza la API `csi.CSI()` y coordenadas en tupla para `get_pixel` y `draw_string`).
2. Abra el script de dirección en OpenMV IDE y seleccione la ley de control en la cabecera:
   ```python
   CONTROLADOR = "PID"  # Opciones: "P", "PI", "PD", "PID"
   ```
3. Verifique la calibración espacial (`METROS_POR_PIXEL = 0.0022`) y el sentido del servo (`INVERTIR_GIRO = 1` o `-1`).
4. Guarde el archivo en la memoria interna de la cámara como `main.py` para ejecución autónoma al energizar.

### Paso 3: Programación de la ESP32-S3
1. Instale la librería `ESP32Servo` en su entorno de desarrollo.
2. Abra el firmware de control de velocidad y verifique:
   * `SENTIDO_AVANCE`: `-1` si el ESC avanza con pulsos $< 1500\ \mu\text{s}$ (o `+1` en caso contrario).
   * Límites de referencia: `V_RECTA` ($1.5\text{ m/s}$) y `V_CURVA` ($0.34\text{ m/s}$).
   * Ganancias del PI discreto: `KP_VEL = 2.4f`, `TI_VEL = 0.090f`, `TS_VEL = 0.0005f`.
3. Compile y cargue el firmware en la ESP32-S3.

### Paso 4: Protocolo de Comunicación UART
La OpenMV transmite en cada fotograma una trama ASCII a $115200$ baudios con el formato:

```text
detectado,u_dir,y_rad,curvatura\n
```

* `detectado`: `1` si la cinta es válida; `0` si se perdió la línea (la OpenMV mantiene el giro en `ultima_dir_giro` y la ESP32 reduce la referencia a `V_CURVA`).
* `u_dir`: Señal de control de dirección saturada en $[-1.65, +1.65]$.
* `y_rad`: Error angular geométrico Ackermann en radianes.
* `curvatura`: Curvatura normalizada en $[0.0, 1.0]$ para anticipación de desaceleración.

Si la ESP32-S3 deja de recibir tramas por más de $1000\text{ ms}$, el *watchdog* detiene el motor enviando $1500\ \mu\text{s}$ al ESC.

---

## 5. Secuencia de Puesta en Marcha y Pruebas

1. **Eleve las ruedas motrices** antes de energizar el ESC para pruebas iniciales de banco.
2. Energice la OpenMV H7 y la ESP32-S3. Espere $5\text{ s}$ mientras la ESP32 mantiene el pulso neutro ($1500\ \mu\text{s}$) para completar la secuencia de armado del ESC WP-10BL50-RTR.
3. Confirme en el Monitor Serie ($115200$ baudios) la recepción de telemetría cada $100\text{ ms}$:
   ```text
   Vref:1.50 | Vreal:0.00 m/s (FFT:0.0Hz) | u:0.00V | ESC:1500us || OMV[Det:1 u_dir:0.00 curv:0.00]
   ```
4. Utilice los comandos por consola serie para validar el actuador antes de colocar el rover en pista:
   * `s` o `0`: Detiene el motor de inmediato ($1500\ \mu\text{s}$).
   * `p <valor>`: Modo manual en porcentaje $[-100, 100]$ (ejemplo: `p -8` aplica $1460\ \mu\text{s}$). Útil para verificar `SENTIDO_AVANCE` y la lectura `FFT` del tacómetro.
   * `g`: Reactiva el modo automático en cascada con la OpenMV.
5. Coloque el vehículo sobre la pista blanca con cinta negra centrada. Para pruebas de ajuste inicial, reduzca `V_RECTA` y `PCT_MAX_MOTOR` hasta validar la estabilidad en las curvas de menor radio.
