package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Shooter (2 Yellow Jacket 6000 RPM sin hood): la distancia se controla con las RPM de cada motor.
 * Se ajusta en vivo desde Panels; los valores buenos se copian aquí.
 *
 * La TABLA de tiro (distancia -> RPM de cada motor) se llena con "Test 6: Tiro": desde cada
 * distancia se buscan las RPM que meten la pelota y se anotan. Los valores de abajo son un punto
 * de partida razonable, no medidos.
 */
@Configurable
public class ShooterConstants {

    // ---------- Control de velocidad (por motor) ----------
    /** Potencia por RPM a 12 V (feedforward). 1/6000 = el motor libre a potencia 1. */
    public static double KV = 1.0 / 5900.0;
    /** Potencia mínima para vencer la fricción. */
    public static double KS = 0.03;
    /** Corrección proporcional por cada RPM de error. */
    public static double KP = 0.0006;
    /** Voltaje con el que se calcula KV. Con la batería más baja, la potencia sube para compensar. */
    public static double NOMINAL_VOLTAGE = 12.0;
    /** RPM máximas que se le piden al shooter. */
    public static double MAX_RPM = 5800;

    // ---------- Listo para disparar ----------
    /** El shooter está listo si los dos motores están a menos de esto de su objetivo... */
    public static double READY_TOLERANCE_RPM = 100;
    /** ...durante al menos este tiempo seguido. */
    public static double READY_TIME_S = 0.10;

    // ---------- Tabla de tiro ----------
    /** Distancias del eje de la torreta al objetivo, en pulgadas, de menor a mayor. */
    public static double[] TABLE_DISTANCE_IN = {36, 60, 84, 108, 132, 156};
    public static double[] TABLE_RPM_LEFT    = {2400, 2800, 3200, 3600, 4000, 4400};
    public static double[] TABLE_RPM_RIGHT   = {2400, 2800, 3200, 3600, 4000, 4400};

    /** Tiempo de vuelo de la pelota según la distancia (para disparar en movimiento). */
    public static double[] TOF_DISTANCE_IN = {36, 156};
    public static double[] TOF_SECONDS     = {0.40, 0.80};

    /** Ajuste en vivo de RPM del operador (bumpers): tamaño de cada paso. */
    public static double RPM_TRIM_STEP = 50;
    /** Límite del ajuste en vivo. */
    public static double RPM_TRIM_MAX = 600;

    // ---------- Modo manual (sin visión) ----------
    /** Distancias de los 3 presets del operador: cerca, medio, lejos. */
    public static double[] MANUAL_PRESET_DISTANCE_IN = {48, 84, 120};

    /** RPM mientras el operador mantiene el shooter "preparado" sin objetivo. */
    public static double PREPARE_RPM = 2800;
}
