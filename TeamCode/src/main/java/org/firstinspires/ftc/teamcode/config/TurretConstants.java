package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Control de la torreta (PID + feedforward). Lo mecánico y lo medido con "Test 5" está en
 * TurretConfig; aquí solo lo que se afina en vivo desde Panels.
 *
 * Potencia positiva = antihorario. Ángulos en grados de torreta.
 */
@Configurable
public class TurretConstants {
    /** Potencia por grado de error. */
    public static double KP = 0.012;
    /** Potencia por grado/segundo de error (frena el sobretiro). */
    public static double KD = 0.0005;
    /** Potencia mínima para que la torreta se mueva (fricción). */
    public static double KS = 0.12;   // en el robot, con 0.25 ya le cuesta avanzar con la cadena
    /** Potencia por grado/segundo que se necesita (feedforward de velocidad). */
    public static double KV = 1.0 / 300.0;
    /** Con menos error que esto no se aplica KS (evita vibrar en el objetivo). */
    public static double DEADBAND_DEG = 0.4;
    /** Potencia máxima en automático. */
    public static double MAX_POWER = 0.9;

    /** En el objetivo: error menor a esto... */
    public static double ON_TARGET_DEG = 2.0;
    /** ...y moviéndose más lento que esto. */
    public static double ON_TARGET_RATE_DEG_S = 30;

    /** Cerca de un límite (a menos de esto), la potencia hacia el límite se reduce. */
    public static double SLOW_ZONE_DEG = 15;
    /** Potencia máxima con el stick (control manual). */
    public static double MANUAL_MAX_POWER = 0.7;

    /** Cuánto se adelanta el objetivo para compensar el retraso de la torreta. */
    public static double LEAD_TIME_S = 0.04;

    // ---------- Detección de fallas (solo avisan; nunca cortan la potencia) ----------
    /** Con al menos esta potencia y sin moverse STALL_DEG en STALL_TIME_S: aviso de atorada. */
    public static double STALL_POWER = 0.35;
    public static double STALL_TIME_S = 0.5;
    public static double STALL_DEG = 1.5;
    /**
     * Aviso de dirección: con al menos esta potencia, la torreta va al revés a más de esta
     * velocidad durante este tiempo seguido (un frenado por inercia dura menos y no cuenta). El
     * aviso se borra solo después de WRONG_DIRECTION_RETRY_S sin repetirse.
     */
    public static double WRONG_DIRECTION_POWER = 0.3;
    public static double WRONG_DIRECTION_RATE_DEG_S = 40;
    public static double WRONG_DIRECTION_TIME_S = 0.4;
    public static double WRONG_DIRECTION_RETRY_S = 2.0;
    /** Los dos encoders deben diferir más que ENCODER_MISMATCH_DEG este tiempo para marcar falla. */
    public static double MISMATCH_PERSIST_S = 0.3;
}
