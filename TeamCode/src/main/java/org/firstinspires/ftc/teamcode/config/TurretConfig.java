package org.firstinspires.ftc.teamcode.config;

/**
 * Constantes de la torreta: 2 Axon MINI MK2 en modo continuo (CR) sobre el mismo eje, más el
 * 4.º cable de turretLeft en un puerto analógico.
 *
 * Convención de ángulos: 0° = home (torreta apuntando a la izquierda del robot).
 * Positivo = antihorario visto desde arriba.
 *
 * Los valores marcados "MEDIR" se obtienen con el OpMode "Test 5: Torreta" y se copian aquí.
 */
public final class TurretConfig {
    private TurretConfig() {}

    // ---------- Direcciones (MEDIR con Test 5, pasos 1 a 3) ----------
    /** true si turretLeft gira la torreta en horario con potencia positiva. */
    public static final boolean LEFT_REVERSED = false;
    /** true si turretRight gira la torreta en horario con potencia positiva. */
    public static final boolean RIGHT_REVERSED = false;
    /** true si al girar la torreta en antihorario el ángulo del encoder BAJA. */
    public static final boolean ENCODER_REVERSED = false;

    // ---------- Encoder analógico (MEDIR con Test 5, paso 5) ----------
    /** Voltaje mínimo y máximo que da el 4.º cable en una vuelta completa del servo. */
    public static final double ENCODER_V_MIN = 0.0;
    public static final double ENCODER_V_MAX = 3.3;
    /**
     * Voltaje del encoder con la torreta exactamente en home. Negativo = todavía no medido: en ese
     * caso se toma como home la posición en la que esté la torreta al presionar INIT.
     *
     * Al encender, el encoder solo distingue una vuelta del servo = 180° de torreta. Por eso se
     * asume que la torreta arranca a menos de 90° de home. Con HOME_VOLTAGE bien medido, cualquier
     * arranque dentro de ±90° da el ángulo exacto; fuera de eso, el ángulo sale 180° equivocado.
     */
    public static final double HOME_VOLTAGE = -1;

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
