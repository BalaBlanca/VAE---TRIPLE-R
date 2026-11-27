#include <Arduino.h>
#include <ESP32Servo.h>
#include "esp_timer.h"
#include <math.h>

// =========================================================================
// 1. PARÁMETROS DE VELOCIDAD POR TRAMO (MODULACIÓN CONTINUA POR ÁNGULO)
// =========================================================================
#define ARRANCAR_AUTOMATICO true

// Sentido de avance del ESC (-1 avanza con <1500 us, +1 avanza con >1500 us)
const int SENTIDO_AVANCE = -1;

// Velocidades objetivo [m/s] (Ajustables según pista)
const float V_RECTA        = 1.5f;  // [m/s] Velocidad máxima en recta (ángulo ~ 0)
const float V_CURVA        = 0.34f;  // [m/s] Velocidad mínima en curva cerrada
const float ZONA_RECTA_DIR = 0.06f;  // [rad] Pequeña tolerancia para ruido en recta
const float ANGULO_MAX_DIR = 1.35f;  // [rad] Ángulo a partir del cual aplica V_CURVA
const float PESO_ANTICIP   = 0.35f;  // Influencia de la curvatura lejana (0.0 a 1.0)

// Mapeo PWM hacia el ESC Hobbywing WP-10BL50-RTR (1% = 5 us)
const float PCT_COAST_MIN   = 3.5f;  // 17 us (1483 us): giro libre suave sin activar drag-brake
const float PCT_ZONA_MUERTA = 5.2f;  // 26 us (1474 us): mínimo pulso de tracción efectiva
const float PCT_MAX_MOTOR   = 23.0f; // 115 us (1385 us): techo de potencia del controlador PI
const int   MAX_PASO_ACEL_US  = 6;   // Rampa suave al acelerar (cada 20 ms)
const int   MAX_PASO_DECEL_US = 14;  // Rampa progresiva al desacelerar (evita frenazo brusco)

// =========================================================================
// 2. PINES Y PARÁMETROS DEL ESC / MOTOR BS214-008
// =========================================================================
#define ESC_PIN   13  // Señal PWM hacia el ESC
#define TACHO_PIN 4   // Salida 3.3V del comparador de corriente
#define RX_PIN    18  // Recibe TX (Pin P4) de la OpenMV H7
#define TX_PIN    17  // Opcional hacia RX (Pin P5) de la OpenMV H7

const int      ESC_NEUTRO_US    = 1500;
const int      ESC_MIN_US       = 1000;
const int      ESC_MAX_US       = 2000;
const uint32_t TIEMPO_ARMADO_MS = 5000;

// Motor 4 polos (p = 2), transmisión 1/10, diámetro rueda = 0.109 m
const float PARES_POLOS          = 2.0f;
const float RELACION_TRANSMISION = 10.0f;
const float DIAMETRO_RUEDA_M     = 0.109f;
const float FACTOR_RPM_A_MS      = (PI * DIAMETRO_RUEDA_M) / 60.0f;

// =========================================================================
// 3. PARÁMETROS DEL CONTROLADOR PI DISCRETO (SINTONIZADO PARA HARDWARE REAL)
// =========================================================================
const float KP_VEL    = 2.4f;    // Ganancia proporcional suavizada
const float TI_VEL    = 0.090f;  // Tiempo integral ajustado al retardo de FFT/ESC (90 ms)
const float TS_VEL    = 0.0005f; // 500 us -> Fs = 2000 Hz
const float B_PESO    = 1.0f;
const float U_VEL_MIN = 0.0f;
const float U_VEL_MAX = 3.3f;

// =========================================================================
// 4. CONFIGURACIÓN DE LA FFT (Fs = 2000 Hz, N = 256 -> df = 7.8125 Hz)
// =========================================================================
#define FFT_N 256
const float FS_HZ = 1.0f / TS_VEL;

const int BIN_MIN = 1;
const int BIN_MAX = 13;

portMUX_TYPE mux_fft = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t  ring_buf[FFT_N];
volatile uint16_t ring_idx        = 0;
volatile uint16_t muestras_nuevas = 0;
const uint16_t    FFT_HOP         = 32;

float vReal[FFT_N];
float vImag[FFT_N];
float ventana_hann[FFT_N];
uint8_t ventanas_sin_pulsos = 0;

// =========================================================================
// 5. VARIABLES DE ESTADO COMPARTIDAS
// =========================================================================
Servo escMotor;
esp_timer_handle_t timer_control;

volatile int      omv_linea_ok  = 0;
volatile float    omv_u_dir     = 0.0f;
volatile float    omv_y_rad     = 0.0f;
volatile float    omv_curvatura = 0.0f;
volatile uint32_t ultimo_rx_ms  = 0;
String buffer_uart = "";

volatile bool  motor_habilitado   = ARRANCAR_AUTOMATICO;
volatile bool  modo_manual        = false;
volatile float pct_manual         = 0.0f;

volatile float V_ref_actual       = 0.0f;
volatile float V_actual_ms        = 0.0f;
volatile float freq_fund_hz       = 0.0f;
volatile float I_vel              = 0.0f;
volatile float u_vel_actual       = 0.0f;
volatile int   pulso_esc_objetivo = ESC_NEUTRO_US;
int            pulso_esc_suave    = ESC_NEUTRO_US;

inline int porcentajeAMicrosegundos(float pct) {
  float pct_lim = constrain(pct, -100.0f, 100.0f);
  int pulso = ESC_NEUTRO_US + (int)(pct_lim * 5.0f);
  return constrain(pulso, ESC_MIN_US, ESC_MAX_US);
}

// Modulación continua: ángulo pequeño -> V_RECTA | ángulo grande -> V_CURVA
inline float calcularReferenciaVelocidad(float u_dir, float curvatura) {
  float angulo = fabsf(u_dir);
  float factor_angulo = 0.0f;

  if (angulo > ZONA_RECTA_DIR) {
    factor_angulo = (angulo - ZONA_RECTA_DIR) / (ANGULO_MAX_DIR - ZONA_RECTA_DIR);
    factor_angulo = constrain(factor_angulo, 0.0f, 1.0f);
  }

  // Combina el ángulo instantáneo con la curvatura lejana anticipada
  float factor_reduccion = constrain(factor_angulo + (PESO_ANTICIP * curvatura * (1.0f - factor_angulo)), 0.0f, 1.0f);

  return V_RECTA - factor_reduccion * (V_RECTA - V_CURVA);
}

// =========================================================================
// 6. CALLBACK PERIÓDICO EXACTO CADA Ts = 500 us (MUESTREO FFT + CONTROL PI)
// =========================================================================
void onTimerControl(void* arg) {
  uint8_t muestra = (uint8_t)digitalRead(TACHO_PIN);
  portENTER_CRITICAL_ISR(&mux_fft);
  ring_buf[ring_idx] = muestra;
  ring_idx = (ring_idx + 1) & (FFT_N - 1);
  if (muestras_nuevas < FFT_N) {
    muestras_nuevas++;
  }
  portEXIT_CRITICAL_ISR(&mux_fft);

  if (modo_manual) {
    pulso_esc_objetivo = porcentajeAMicrosegundos(pct_manual);
    return;
  }

  bool rx_valido = ((millis() - ultimo_rx_ms) <= 1000UL);
  if (!motor_habilitado || !rx_valido) {
    V_ref_actual       = 0.0f;
    I_vel              = 0.0f;
    u_vel_actual       = 0.0f;
    pulso_esc_objetivo = ESC_NEUTRO_US;
    return;
  }

  // Referencia continua de velocidad desde OpenMV
  float v_ref = (omv_linea_ok == 1)
                  ? calcularReferenciaVelocidad(omv_u_dir, omv_curvatura)
                  : V_CURVA;
  V_ref_actual = v_ref;

  // Controlador PI discreto
  float y_k = V_actual_ms;
  float e_k = v_ref - y_k;

  float P_k   = KP_VEL * (B_PESO * v_ref - y_k);
  float u_raw = P_k + I_vel;
  float u_sat = constrain(u_raw, U_VEL_MIN, U_VEL_MAX);
  u_vel_actual = u_sat;

  // Integrador con anti-windup
  float I_next = I_vel + ((KP_VEL * TS_VEL) / TI_VEL) * e_k;
  if ((u_raw > U_VEL_MAX && e_k > 0.0f) || (u_raw < U_VEL_MIN && e_k < 0.0f)) {
    I_next = I_vel;
  }
  I_vel = constrain(I_next, U_VEL_MIN, U_VEL_MAX);

  // Conversión proporcional sin corte brusco a 1500 us:
  // Si u_sat llega a 0 V (desaceleración), baja hasta PCT_COAST_MIN para que el carro
  // reduzca velocidad por inercia sin activar el freno brusco del ESC.
  if (e_k < -0.28f) {
    pulso_esc_objetivo = ESC_NEUTRO_US; // Solo corta a neutro ante exceso fuerte de velocidad
  } else if (u_sat <= 0.02f) {
    pulso_esc_objetivo = porcentajeAMicrosegundos((float)SENTIDO_AVANCE * PCT_COAST_MIN);
  } else {
    float fraccion     = u_sat / U_VEL_MAX;
    float magnitud_pct = PCT_ZONA_MUERTA + fraccion * (PCT_MAX_MOTOR - PCT_ZONA_MUERTA);
    pulso_esc_objetivo = porcentajeAMicrosegundos((float)SENTIDO_AVANCE * magnitud_pct);
  }
}

// =========================================================================
// 7. FFT RADIX-2 AUTOCONTENIDA + ESTIMACIÓN ESTABLE DE VELOCIDAD
// =========================================================================
void inicializarVentanaHann() {
  for (int i = 0; i < FFT_N; i++) {
    ventana_hann[i] = 0.5f * (1.0f - cosf((2.0f * PI * i) / (FFT_N - 1)));
  }
}

void ejecutarFFTRadix2(float *real, float *imag, int n) {
  int j = 0;
  for (int i = 1; i < n; i++) {
    int bit = n >> 1;
    while (j & bit) {
      j ^= bit;
      bit >>= 1;
    }
    j ^= bit;
    if (i < j) {
      float tr = real[i]; real[i] = real[j]; real[j] = tr;
      float ti = imag[i]; imag[i] = imag[j]; imag[j] = ti;
    }
  }

  for (int len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * PI / (float)len;
    float wlen_r = cosf(ang);
    float wlen_i = sinf(ang);
    for (int i = 0; i < n; i += len) {
      float wr = 1.0f;
      float wi = 0.0f;
      int half = len >> 1;
      for (int k = 0; k < half; k++) {
        int u_idx = i + k;
        int v_idx = u_idx + half;
        float vr = real[v_idx] * wr - imag[v_idx] * wi;
        float vi = real[v_idx] * wi + imag[v_idx] * wr;
        real[v_idx] = real[u_idx] - vr;
        imag[v_idx] = imag[u_idx] - vi;
        real[u_idx] += vr;
        imag[u_idx] += vi;
        float next_wr = wr * wlen_r - wi * wlen_i;
        wi = wr * wlen_i + wi * wlen_r;
        wr = next_wr;
      }
    }
  }
}

void actualizarVelocidadPorFFT() {
  uint8_t copia_buf[FFT_N];
  portENTER_CRITICAL(&mux_fft);
  uint16_t idx_inicio = ring_idx;
  for (int i = 0; i < FFT_N; i++) {
    copia_buf[i] = ring_buf[(idx_inicio + i) & (FFT_N - 1)];
  }
  portEXIT_CRITICAL(&mux_fft);

  float suma = 0.0f;
  int transiciones = 0;
  uint8_t prev = copia_buf[0];

  for (int i = 0; i < FFT_N; i++) {
    uint8_t val = copia_buf[i];
    if (val != prev) transiciones++;
    prev = val;
    vReal[i] = (float)val;
    suma += vReal[i];
  }

  if (transiciones < 2) {
    ventanas_sin_pulsos++;
    if (ventanas_sin_pulsos >= 5) {
      freq_fund_hz = 0.0f;
      V_actual_ms  = 0.0f;
    }
    return;
  }
  ventanas_sin_pulsos = 0;

  float media = suma / (float)FFT_N;
  for (int i = 0; i < FFT_N; i++) {
    vReal[i] = (vReal[i] - media) * ventana_hann[i];
    vImag[i] = 0.0f;
  }

  ejecutarFFTRadix2(vReal, vImag, FFT_N);

  int bin_max = BIN_MIN;
  float mag_max = 0.0f;
  for (int k = BIN_MIN; k <= BIN_MAX; k++) {
    float mag2 = (vReal[k] * vReal[k]) + (vImag[k] * vImag[k]);
    if (mag2 > mag_max) {
      mag_max = mag2;
      bin_max = k;
    }
  }

  float bin_fino = (float)bin_max;
  if (bin_max > BIN_MIN && bin_max < BIN_MAX) {
    float m_izq = sqrtf(vReal[bin_max - 1] * vReal[bin_max - 1] + vImag[bin_max - 1] * vImag[bin_max - 1]);
    float m_cen = sqrtf(mag_max);
    float m_der = sqrtf(vReal[bin_max + 1] * vReal[bin_max + 1] + vImag[bin_max + 1] * vImag[bin_max + 1]);
    float denom = (m_izq - 2.0f * m_cen + m_der);
    if (fabsf(denom) > 1e-6f) {
      float delta = 0.5f * (m_izq - m_der) / denom;
      if (delta > -0.5f && delta < 0.5f) {
        bin_fino += delta;
      }
    }
  }

  float f_elec    = bin_fino * (FS_HZ / (float)FFT_N);
  float f_mec     = f_elec / PARES_POLOS;
  float rpm_rueda = (f_mec * 60.0f) / RELACION_TRANSMISION;
  float v_medida  = rpm_rueda * FACTOR_RPM_A_MS;

  freq_fund_hz = f_elec;
  V_actual_ms  = (0.28f * v_medida) + (0.72f * V_actual_ms);
}

// =========================================================================
// 8. RECEPCIÓN UART OPENMV ("detectado,u_dir,y_rad,curvatura\n")
// =========================================================================
void leerTelemetriaOpenMV() {
  while (Serial1.available() > 0) {
    char c = (char)Serial1.read();
    if (c == '\n') {
      int c1 = buffer_uart.indexOf(',');
      int c2 = buffer_uart.indexOf(',', c1 + 1);
      int c3 = buffer_uart.indexOf(',', c2 + 1);
      if (c1 > 0 && c2 > c1) {
        omv_linea_ok = buffer_uart.substring(0, c1).toInt();
        omv_u_dir    = buffer_uart.substring(c1 + 1, c2).toFloat();
        if (c3 > c2) {
          omv_y_rad     = buffer_uart.substring(c2 + 1, c3).toFloat();
          omv_curvatura = buffer_uart.substring(c3 + 1).toFloat();
        } else {
          omv_y_rad     = buffer_uart.substring(c2 + 1).toFloat();
          omv_curvatura = 0.0f;
        }
        ultimo_rx_ms = millis();
      }
      buffer_uart = "";
    } else if (c != '\r') {
      buffer_uart += c;
      if (buffer_uart.length() > 64) buffer_uart = "";
    }
  }
}

// =========================================================================
// 9. SETUP Y LOOP PRINCIPAL
// =========================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

  pinMode(TACHO_PIN, INPUT_PULLUP);
  inicializarVentanaHann();

  escMotor.setPeriodHertz(50);
  escMotor.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);

  Serial.println("Armando ESC WP-10BL50-RTR en 1500 us... Espere 5s.");
  uint32_t t0 = millis();
  while ((millis() - t0) < TIEMPO_ARMADO_MS) {
    escMotor.writeMicroseconds(ESC_NEUTRO_US);
    delay(20);
  }

  const esp_timer_create_args_t timer_args = {
    .callback        = &onTimerControl,
    .arg             = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name            = "pi_fft_2khz"
  };
  esp_timer_create(&timer_args, &timer_control);
  esp_timer_start_periodic(timer_control, 500);

  Serial.printf("=== PI + FFT SUAVIZADO | V_recta: %.2f m/s | V_curva: %.2f m/s ===\n",
                V_RECTA, V_CURVA);
}

void loop() {
  leerTelemetriaOpenMV();

  if (muestras_nuevas >= FFT_HOP) {
    portENTER_CRITICAL(&mux_fft);
    muestras_nuevas = 0;
    portEXIT_CRITICAL(&mux_fft);
    actualizarVelocidadPorFFT();
  }

  // Rampa asimétrica controlada a 50 Hz:
  // Acelera suave (MAX_PASO_ACEL_US) y desacelera progresivo (MAX_PASO_DECEL_US) sin frenazos
  static uint32_t t_esc = 0;
  if (millis() - t_esc >= 20) {
    t_esc = millis();
    int objetivo = pulso_esc_objetivo;
    int mag_obj  = abs(objetivo - ESC_NEUTRO_US);
    int mag_act  = abs(pulso_esc_suave - ESC_NEUTRO_US);

    if (objetivo == ESC_NEUTRO_US && !motor_habilitado) {
      pulso_esc_suave = ESC_NEUTRO_US; // Parada inmediata solo por comando 's' o watchdog
    } else if (mag_obj < mag_act) {
      // Desaceleración progresiva
      int diff = objetivo - pulso_esc_suave;
      if (diff > MAX_PASO_DECEL_US)       pulso_esc_suave += MAX_PASO_DECEL_US;
      else if (diff < -MAX_PASO_DECEL_US) pulso_esc_suave -= MAX_PASO_DECEL_US;
      else                                pulso_esc_suave  = objetivo;
    } else {
      // Aceleración suave
      int diff = objetivo - pulso_esc_suave;
      if (diff > MAX_PASO_ACEL_US)       pulso_esc_suave += MAX_PASO_ACEL_US;
      else if (diff < -MAX_PASO_ACEL_US) pulso_esc_suave -= MAX_PASO_ACEL_US;
      else                               pulso_esc_suave  = objetivo;
    }
    escMotor.writeMicroseconds(pulso_esc_suave);
  }

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.equalsIgnoreCase("g")) {
      modo_manual      = false;
      motor_habilitado = true;
      Serial.println(">> MODO CASCADA OPENMV ACTIVO");
    } else if (cmd.equalsIgnoreCase("s") || cmd == "0") {
      modo_manual      = false;
      motor_habilitado = false;
      pct_manual       = 0.0f;
      Serial.println(">> MOTOR DETENIDO");
    } else if (cmd.startsWith("p ") || cmd.startsWith("P ")) {
      pct_manual  = constrain(cmd.substring(2).toFloat(), -100.0f, 100.0f);
      modo_manual = true;
      Serial.printf(">> MODO MANUAL: %.1f%% (%d us)\n", pct_manual, porcentajeAMicrosegundos(pct_manual));
    }
  }

  static uint32_t t_tel = 0;
  if (millis() - t_tel >= 100) {
    t_tel = millis();
    Serial.printf("Vref:%.2f | Vreal:%.2f m/s (FFT:%.1fHz) | u:%.2fV | ESC:%dus || OMV[Det:%d u_dir:%.2f curv:%.2f]\n",
                  V_ref_actual, V_actual_ms, freq_fund_hz, u_vel_actual, pulso_esc_suave,
                  omv_linea_ok, omv_u_dir, omv_curvatura);
  }
}