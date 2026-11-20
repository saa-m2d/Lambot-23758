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

    // ---------- Detección de fallas ----------
    /** Con al menos esta potencia y sin moverse STALL_DEG en STALL_TIME_S: atorada. */
    public static double STALL_POWER = 0.35;
    public static double STALL_TIME_S = 0.5;
    public static double STALL_DEG = 1.5;
    /** Tras atorarse, la potencia se corta este tiempo. */
    public static double STALL_COOLDOWN_S = 1.0;
    /** Si se mueve más que esto al revés de la potencia: dirección mal configurada (se bloquea). */
    public static double WRONG_DIRECTION_DEG = 8;
    /** Los dos encoders deben diferir más que ENCODER_MISMATCH_DEG este tiempo para marcar falla. */
    public static double MISMATCH_PERSIST_S = 0.3;
}
