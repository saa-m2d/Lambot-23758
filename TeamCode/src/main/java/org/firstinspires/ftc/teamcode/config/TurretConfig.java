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

    // ---------- Medido con la calibración automática del Test 5 (2026-11-19) ----------
    /** true si turretLeft gira la torreta en horario con potencia positiva. */
    public static final boolean LEFT_REVERSED = false;
    /** true si turretRight gira la torreta en horario con potencia positiva. */
    public static final boolean RIGHT_REVERSED = false;

    // ---------- Encoder de turretLeft ----------
    /** true si al girar la torreta en antihorario el ángulo de este encoder BAJA. */
    public static final boolean LEFT_ENCODER_REVERSED = false;
    /** Voltaje mínimo y máximo en una vuelta completa del servo. */
    public static final double LEFT_ENCODER_V_MIN = 0.021;
    public static final double LEFT_ENCODER_V_MAX = 3.213;
    /**
     * Voltaje con la torreta en home. Negativo = sin medir: se usa la posición al presionar INIT.
     * Ojo: 0.128 V está cerca del salto 3.2 V → 0 V; el código lo maneja, pero si algún día se
     * remonta la torreta conviene que home quede lejos de ese salto.
     */
    public static final double LEFT_HOME_VOLTAGE = 0.128;

    // ---------- Encoder de turretRight ----------
    public static final boolean RIGHT_ENCODER_REVERSED = false;
    public static final double RIGHT_ENCODER_V_MIN = 0.017;
    public static final double RIGHT_ENCODER_V_MAX = 3.211;
    public static final double RIGHT_HOME_VOLTAGE = 0.565;

    /*
     * Al encender, cada encoder solo distingue una vuelta del servo = 360° / GEAR_RATIO de torreta
     * (unos 128°). Por eso la torreta debe arrancar a menos de ~64° de home: dentro de eso, el
     * ángulo sale exacto; fuera, sale una vuelta de servo equivocado.
     */

    /** Si los dos encoders difieren más que esto (grados de torreta), algo anda mal. */
    public static final double ENCODER_MISMATCH_DEG = 5.0;

    // ---------- Mecánica ----------
    /**
     * Vueltas del servo por cada vuelta de la torreta. Medido en el robot (2026-11-19):
     *   - con 2.0, la pantalla marcaba 127° con 90° reales → ~2.82;
     *   - con 2.82, en el recorrido completo la pantalla marcaba ~350° con 360° reales → 2.742;
     *   - con 2.742, el ajuste de vuelta completa del Test 5 (D-pad ↑) marcó 352° en una vuelta
     *     exacta de home a home → 2.742 × 352 / 360 = 2.681. Es la medición más precisa.
     * Si se cuentan los dientes (dientes de la torreta ÷ dientes del servo), poner ese valor exacto.
     */
    public static double GEAR_RATIO = 2.681;   // no es final: el Test 5 la ajusta en vivo (D-pad ↑)

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
