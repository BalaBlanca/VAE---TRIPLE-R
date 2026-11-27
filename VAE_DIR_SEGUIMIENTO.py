import csi
import image
import time
import math
from pyb import UART, Pin, Timer, LED

# =========================================================================
# 0. SELECTOR RÁPIDO DE CONTROLADOR (ELIGE: "P", "PI", "PD" o "PID")
# =========================================================================
CONTROLADOR = "P"

# =========================================================================
# 1. CONFIGURACIÓN DE CÁMARA (OpenMV v5.0.1 - Pista Blanca / Cinta Negra)
# =========================================================================
led_verde = LED(2)
led_verde.on()

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.GRAYSCALE)
cam.framesize(csi.QQVGA)        # 160x120
cam.window((0, 60, 160, 60))    # Mitad inferior (160x60)
cam.snapshot(time=1500)

cam.auto_gain(True)
cam.auto_exposure(True)

# 2. UART3 HACIA ESP32-S3 (P4 = TX -> GPIO 18 en ESP32-S3)
uart = UART(3, 115200)

# 3. PWM SERVO DE DIRECCIÓN EN PIN P7 (50 Hz -> 20000 us)
SERVO_CENTRO_US = 1500
SERVO_MIN_US    = 1000
SERVO_MAX_US    = 2000
INVERTIR_GIRO   = 1

tim_servo = Timer(4, freq=50)
ch_servo  = tim_servo.channel(1, Timer.PWM, pin=Pin("P7"))
PERIODO_TICKS = tim_servo.period()

def mover_servo_us(us):
    us_lim = max(SERVO_MIN_US, min(SERVO_MAX_US, int(us)))
    ticks = int((us_lim * PERIODO_TICKS) // 20000)
    ch_servo.pulse_width(ticks)
    return us_lim

mover_servo_us(SERVO_CENTRO_US)

# =========================================================================
# 4. PARÁMETROS ACKERMANN Y GANANCIAS DEL CONTROLADOR (SIN STANLEY)
# =========================================================================
L_ROVER          = 0.23965  # Distancia entre ejes Ackermann [m]
METROS_POR_PIXEL = 0.0022   # Ancho real visto [m] / 160 px
U_DIR_MAX        = 1.65     # Saturación [-1.65, +1.65]

if CONTROLADOR == "P":
    Kp       = 2.0
    Ti       = 0.0
    Td       = 0.0
    N_FILTRO = 4.0
    B_PESO   = 1.0
    USAR_I   = False
    USAR_D   = False
elif CONTROLADOR == "PI":
    Kp       = 1.5
    Ti       = 4.0
    Td       = 0.0
    N_FILTRO = 4.0
    B_PESO   = 1.0
    USAR_I   = True
    USAR_D   = False
elif CONTROLADOR == "PD":
    Kp       = 2.95
    Ti       = 0.0
    Td       = 0.20
    N_FILTRO = 4.0
    B_PESO   = 1.0
    USAR_I   = False
    USAR_D   = True
elif CONTROLADOR == "PID":
    Kp       = 2.5
    Ti       = 10.0
    Td       = 0.30
    N_FILTRO = 4.0
    B_PESO   = 0.9
    USAR_I   = True
    USAR_D   = True
else:
    raise ValueError("CONTROLADOR invalido: usa 'P', 'PI', 'PD' o 'PID'")

print("CONTROLADOR ACTIVO:", CONTROLADOR, "| Kp:", Kp, "| Ti:", Ti, "| Td:", Td)

# =========================================================================
# 5. PARÁMETROS DE VISIÓN ROBUSTA CONTRA PISO GRIS Y SOMBRAS
# =========================================================================
UMBRAL_MAX_NEGRO    = 48    # Más estricto: ignora piso gris (~55-120) y uniones
UMBRAL_NUCLEO_NEGRO = 42    # Intensidad máxima permitida en el centro de la cinta
MIN_BLANCO_FONDO    = 95    # Brillo mínimo exigido al tablero blanco junto a la cinta
CONTRASTE_MINIMO    = 60    # Contraste global mínimo en la franja
CONTRASTE_LOCAL     = 55    # Salto mínimo entre el centro de la cinta y sus lados blancos
ANCHO_MIN_CINTA     = 3
ANCHO_MAX_CINTA     = 105
ANCHO_MAX_EN_BORDE  = 45    # Bloquea sombras anchas pegadas a los extremos de la imagen
MAX_PIXELS_CINTA    = 950
MAX_SALTO_X         = 65
ALPHA_SENSOR        = 0.60

FRANJAS = [
    (0, 40, 160, 20),       # 0: Baja (cercana al eje delantero)
    (0, 20, 160, 20),       # 1: Media
    (0,  0, 160, 20)        # 2: Alta
]
PESOS_FRANJA = [0.60, 0.30, 0.10]
CENTRO_X     = 80.0

# Estados del sensor y controlador discreto
e_lat_filt      = 0.0
curv_filt       = 0.0
y_anterior      = 0.0
I_anterior      = 0.0
D_anterior      = 0.0
u_dir_actual    = 0.0
ultimo_cx       = CENTRO_X
ultima_dir_giro = 1.0
frames_perdidos = 0
contador_led    = 0
t_anterior      = time.ticks_ms()

reloj = time.clock()

def es_cinta_valida(img, b, cx_ref, perdida):
    cx = max(0, min(159, int(b.cx)))
    cy = max(0, min(59, int(b.cy)))
    mitad_w = max(1, b.w // 2)
    borde_izq = cx - mitad_w
    borde_der = cx + mitad_w

    toca_izq = (borde_izq <= 3)
    toca_der = (borde_der >= 156)

    # Descartar franjas oscuras que cubren de lado a lado
    if toca_izq and toca_der:
        return False

    # 1. Verificar que el núcleo sea realmente negro (cinta, no sombra gris)
    p_c = img.get_pixel((cx, cy))
    p_l = img.get_pixel((max(0, cx - 2), cy))
    p_r = img.get_pixel((min(159, cx + 2), cy))
    val_centro = min(p_c, p_l, p_r)
    if val_centro > UMBRAL_NUCLEO_NEGRO:
        return False

    # 2. Muestrear flancos cercanos a la cinta para confirmar tablero blanco
    offset_muestra = min(mitad_w + 6, 24)
    val_izq = img.get_pixel((max(0, cx - offset_muestra), cy))
    val_der = img.get_pixel((min(159, cx + offset_muestra), cy))

    # Caso A: Blob dentro del tablero -> exige blanco nítido a AMBOS lados
    if (not toca_izq) and (not toca_der):
        if (val_izq < MIN_BLANCO_FONDO) or (val_der < MIN_BLANCO_FONDO):
            return False
        if (val_izq - val_centro) < CONTRASTE_LOCAL or (val_der - val_centro) < CONTRASTE_LOCAL:
            return False
        # Evitar salto brusco al lado opuesto justo al re-enganchar
        if perdida:
            if (cx_ref < 55) and (cx > 125):
                return False
            if (cx_ref > 105) and (cx < 35):
                return False
        return True

    # Caso B: Blob tocando borde izquierdo o derecho -> filtro estricto anti-piso/sombra
    if (b.w > ANCHO_MAX_EN_BORDE) or (val_centro > 35):
        return False

    if toca_izq:
        # Solo aceptarlo si la cinta ya venía por la izquierda y tiene blanco a su derecha
        if cx_ref > 75:
            return False
        return (val_der >= MIN_BLANCO_FONDO) and ((val_der - val_centro) >= CONTRASTE_LOCAL)
    else:  # toca_der
        # Solo aceptarlo si la cinta ya venía por la derecha y tiene blanco a su izquierda
        if cx_ref < 85:
            return False
        return (val_izq >= MIN_BLANCO_FONDO) and ((val_izq - val_centro) >= CONTRASTE_LOCAL)

def buscar_cinta_franja(img, roi, cx_ref, restringir_salto, perdida):
    st = img.get_statistics(roi=roi)
    mediana = st.median

    if mediana < 105:
        return None
    if (mediana - st.min) < CONTRASTE_MINIMO:
        return None

    lim = min(UMBRAL_MAX_NEGRO, mediana - 65)
    if lim < 15:
        lim = 15

    blobs = img.find_blobs([(0, lim)], roi=roi, pixels_threshold=14, area_threshold=14, merge=True)

    mejor = None
    mejor_dist = 9999.0
    for b in blobs:
        densidad = b.pixels / max(1, b.w * b.h)
        if not ((ANCHO_MIN_CINTA <= b.w <= ANCHO_MAX_CINTA) and (b.pixels <= MAX_PIXELS_CINTA) and (densidad >= 0.18)):
            continue
        if not es_cinta_valida(img, b, cx_ref, perdida):
            continue
        dist = abs(b.cx - cx_ref)
        if restringir_salto and (dist > MAX_SALTO_X):
            continue
        if dist < mejor_dist:
            mejor_dist = dist
            mejor = b
    return mejor

# =========================================================================
# 6. BUCLE DE CONTROL EN TIEMPO REAL
# =========================================================================
while True:
    reloj.tick()
    img = cam.snapshot()
    img.median(1)

    t_actual = time.ticks_ms()
    T = time.ticks_diff(t_actual, t_anterior) / 1000.0
    t_anterior = t_actual
    if T <= 0.002 or T > 0.15:
        T = 0.020

    contador_led += 1
    if contador_led >= 15:
        led_verde.toggle()
        contador_led = 0

    img.draw_line((80, 0, 80, 60), color=128, thickness=1)
    img.draw_string((2, 2), CONTROLADOR, color=255)

    # 1. Buscar cinta negra en las 3 franjas
    puntos = []
    indices = []
    perdida_actual = (frames_perdidos > 0)
    restringir_primera = (frames_perdidos == 0)
    cx_guia = ultimo_cx

    for i in range(3):
        restringir = restringir_primera if (len(puntos) == 0) else True
        b = buscar_cinta_franja(img, FRANJAS[i], cx_guia, restringir, perdida_actual and (len(puntos) == 0))
        if b is not None:
            px, py = float(b.cx), float(b.cy)
            puntos.append((px, py))
            indices.append(i)
            cx_guia = px

    if len(puntos) >= 1:
        frames_perdidos = 0
        ultimo_cx = puntos[0][0]

        for p in puntos:
            img.draw_cross((int(p[0]), int(p[1])), color=255, size=3)
        for j in range(len(puntos) - 1):
            img.draw_line((int(puntos[j][0]), int(puntos[j][1]),
                           int(puntos[j+1][0]), int(puntos[j+1][1])), color=255, thickness=1)

        # 2. Error geométrico Ackermann puro (sin Stanley)
        suma_cx = 0.0
        suma_w  = 0.0
        for j in range(len(puntos)):
            w_p = PESOS_FRANJA[indices[j]]
            suma_cx += puntos[j][0] * w_p
            suma_w  += w_p
        cx_medido = suma_cx / suma_w
        e_lat_raw = (cx_medido - CENTRO_X) * METROS_POR_PIXEL

        if len(puntos) >= 2:
            dx_px = puntos[-1][0] - puntos[0][0]
            curv_raw = min(1.0, abs(dx_px) / 45.0)
        else:
            curv_raw = min(1.0, abs(puntos[0][0] - CENTRO_X) / 55.0)

        e_lat_filt = (ALPHA_SENSOR * e_lat_raw) + ((1.0 - ALPHA_SENSOR) * e_lat_filt)
        curv_filt  = (0.4 * curv_raw) + (0.6 * curv_filt)

        y_k = math.atan2(e_lat_filt, L_ROVER)

        # 3. LEY DE CONTROL DISCRETA PURA (P / PI / PD / PID)
        u_c = 0.0
        e_k = u_c - y_k

        P_k = Kp * ((B_PESO * u_c) - y_k)

        if USAR_D and Td > 0.0:
            denom = (N_FILTRO * T) + Td
            D_k = ((Td / denom) * D_anterior) - (((Kp * Td * N_FILTRO) / denom) * (y_k - y_anterior))
        else:
            D_k = 0.0

        I_actual = I_anterior if USAR_I else 0.0

        u_raw = P_k + I_actual + D_k
        u_dir_actual = max(-U_DIR_MAX, min(U_DIR_MAX, u_raw))

        if USAR_I and Ti > 0.0:
            I_siguiente = I_anterior + ((Kp * T) / Ti) * e_k
            I_anterior = max(-U_DIR_MAX, min(U_DIR_MAX, I_siguiente))
        else:
            I_anterior = 0.0

        y_anterior = y_k
        D_anterior = D_k

        if abs(u_dir_actual) > 0.15:
            ultima_dir_giro = 1.0 if (u_dir_actual > 0) else -1.0

        delta_vis = 0.3173 * (-u_dir_actual)
        x_ack = int(80 + 35 * math.sin(delta_vis))
        y_ack = int(59 - 35 * math.cos(delta_vis))
        img.draw_line((80, 59, x_ack, y_ack), color=0, thickness=2)

        desplazamiento_us = (u_dir_actual / U_DIR_MAX) * 500.0
        mover_servo_us(SERVO_CENTRO_US + (INVERTIR_GIRO * desplazamiento_us))

        uart.write("1,%.3f,%.3f,%.3f\n" % (u_dir_actual, y_k, curv_filt))

    else:
        # LÍNEA PERDIDA: mantiene el giro hacia el lado donde salió la cinta real
        frames_perdidos += 1
        D_anterior = 0.0
        I_anterior = 0.0

        u_dir_actual = U_DIR_MAX * ultima_dir_giro

        delta_vis = 0.3173 * (-u_dir_actual)
        x_ack = int(80 + 35 * math.sin(delta_vis))
        y_ack = int(59 - 35 * math.cos(delta_vis))
        img.draw_line((80, 59, x_ack, y_ack), color=0, thickness=2)

        desplazamiento_us = (u_dir_actual / U_DIR_MAX) * 500.0
        mover_servo_us(SERVO_CENTRO_US + (INVERTIR_GIRO * desplazamiento_us))

        uart.write("0,%.3f,0.000,1.000\n" % u_dir_actual)
