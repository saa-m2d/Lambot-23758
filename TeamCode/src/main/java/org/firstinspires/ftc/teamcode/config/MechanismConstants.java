package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/** Intakes, indexer (tope) y flower. Las posiciones de los servos están en RobotMap. */
@Configurable
public class MechanismConstants {
    // ---------- Intakes ----------
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_REVERSE_POWER = 0.7;
    /** true si intakeLeft es el intake de ENFRENTE. CONFIRMAR en el robot. */
    public static boolean INTAKE_LEFT_IS_FRONT = true;
    /**
     * Antiatasco: si la corriente pasa de esto (A) durante INTAKE_JAM_TIME_S, hace un tironcito en
     * reversa muy corto y sigue metiendo a potencia completa. Nunca baja la potencia por su cuenta.
     */
    public static double INTAKE_JAM_AMPS = 5.0;
    public static double INTAKE_JAM_TIME_S = 0.15;
    /** Tironcito en reversa: corto para que las pelotas no se salgan. */
    public static double INTAKE_UNJAM_POWER = 0.5;
    public static double INTAKE_UNJAM_S = 0.10;
    /** Después de arrancar o de un tironcito, no se mide este tiempo (pico de arranque del motor). */
    public static double INTAKE_START_IGNORE_S = 0.3;
    /** Cada cuánto se lee la corriente del intake (s). */
    public static double INTAKE_CURRENT_PERIOD_S = 0.05;

    // ---------- Indexer (tope) ----------
    /** Una vez abierto, el tope se queda abierto al menos esto (deja pasar la pelota completa). */
    public static double TOPE_MIN_OPEN_S = 0.12;
    /** Una vez cerrado, se queda cerrado al menos esto (deja que el shooter recupere RPM). */
    public static double TOPE_MIN_CLOSED_S = 0.08;

    // ---------- Flower ----------
    /** Tiempo para ir de arriba a abajo (o al revés). Más lento = menos golpe. */
    public static double FLOWER_MOVE_TIME_S = 0.6;
    /** El flower solo se puede usar en el último minuto, salvo que el operador lo desbloquee. */
    public static boolean FLOWER_LOCK_UNTIL_ENDGAME = true;
    /** Segundos de TeleOp después de los que empieza el último minuto (TeleOp dura 120 s). */
    public static double ENDGAME_START_S = 60;
}
