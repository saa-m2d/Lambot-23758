package org.firstinspires.ftc.teamcode.config;

/**
 * Constantes de la torreta: 2 Axon MINI MK2 en modo continuo (CR) sobre el mismo eje. El 4.º cable
 * de cada servo va a un puerto analógico, así que hay dos encoders que deben coincidir.
 *
 * Convención de ángulos: 0° = home (torreta apuntando a la izquierda del robot).
 * Positivo = antihorario visto desde arriba.
 *
 * Los valores marcados "MEDIR" se obtienen con el OpMode "Test 5: Torreta" y se copian aquí.
 */
public final class TurretConfig {
    private TurretConfig() {}

    // ---------- Direcciones de los servos (MEDIR con Test 5) ----------
    /** true si turretLeft gira la torreta en horario con potencia positiva. */
    public static final boolean LEFT_REVERSED = false;
    /** true si turretRight gira la torreta en horario con potencia positiva. */
    public static final boolean RIGHT_REVERSED = false;

    // ---------- Encoder de turretLeft (MEDIR con Test 5) ----------
    /** true si al girar la torreta en antihorario el ángulo de este encoder BAJA. */
    public static final boolean LEFT_ENCODER_REVERSED = false;
    /** Voltaje mínimo y máximo en una vuelta completa del servo. */
    public static final double LEFT_ENCODER_V_MIN = 0.0;
    public static final double LEFT_ENCODER_V_MAX = 3.3;
    /** Voltaje con la torreta en home. Negativo = sin medir: se usa la posición al presionar INIT. */
    public static final double LEFT_HOME_VOLTAGE = -1;

    // ---------- Encoder de turretRight (MEDIR con Test 5) ----------
    public static final boolean RIGHT_ENCODER_REVERSED = false;
    public static final double RIGHT_ENCODER_V_MIN = 0.0;
    public static final double RIGHT_ENCODER_V_MAX = 3.3;
    public static final double RIGHT_HOME_VOLTAGE = -1;

    /*
     * Al encender, cada encoder solo distingue una vuelta del servo = 180° de torreta. Por eso se
     * asume que la torreta arranca a menos de 90° de home: dentro de eso, el ángulo sale exacto;
     * fuera, sale 180° equivocado.
     */

    /** Si los dos encoders difieren más que esto (grados de torreta), algo anda mal. */
    public static final double ENCODER_MISMATCH_DEG = 5.0;

    // ---------- Mecánica ----------
    /** Vueltas del servo por cada vuelta de la torreta. */
    public static final double GEAR_RATIO = 2.0;

    // ---------- Límites de giro (con la cadena portacables) ----------
    /** Límite horario: 40° desde home. */
    public static final double MIN_DEG = -40.0;
    /** Límite antihorario: 360° desde home. */
    public static final double MAX_DEG = 360.0;
    /** Margen antes de cada límite en el que se corta la potencia. */
    public static final double LIMIT_MARGIN_DEG = 5.0;

    // ---------- Señal PWM de los Axon en modo continuo ----------
    /** 1050 µs = máxima velocidad hacia un lado, 1500 = detenido, 1950 = máxima hacia el otro. */
    public static final double CR_PWM_MIN_US = 1050;
    public static final double CR_PWM_MAX_US = 1950;
}
