package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Limelight 3A montada en la torreta y apuntado.
 *
 * Cómo se apunta: los tags de BIOBUZZ están en las CELLS, que se voltean, así que no tienen una
 * posición fija en el campo (el SDK 12.0 tampoco les da una). Por eso el robot no se ubica con
 * los tags para apuntar: cuando la cámara ve los tags de la CELL de arriba, calcula dónde está esa
 * CELL respecto al robot y la guarda en las coordenadas del Pinpoint. Si después deja de verla,
 * sigue apuntando con el Pinpoint.
 *
 * MegaTag 2 (corregir la posición del robot con los tags) está disponible pero apagado: solo es
 * confiable si el mapa de la Limelight coincide con la posición real de las CELLS.
 */
@Configurable
public class VisionConstants {
    public static int PIPELINE = 0;
    public static int POLL_RATE_HZ = 100;
    /** Un resultado más viejo que esto se ignora. */
    public static double STALE_MS = 100;

    // ---------- Montaje de la cámara (respecto al eje de la torreta) ----------
    /** Inclinación de la cámara hacia arriba, en grados. */
    public static double CAMERA_PITCH_DEG = 54;
    /** Distancia de la lente detrás del eje de la torreta, en pulgadas (9.5 cm). */
    public static double CAMERA_BEHIND_AXIS_IN = 3.74;

    // ---------- Montaje de la torreta (respecto al centro del robot) ----------
    /** Eje de la torreta delante del centro del robot, en pulgadas (negativo = atrás). MEDIR. */
    public static double TURRET_AXIS_FORWARD_IN = 0;
    /** Eje de la torreta a la izquierda del centro del robot, en pulgadas (negativo = derecha). MEDIR. */
    public static double TURRET_AXIS_LEFT_IN = 0;

    // ---------- Seguimiento del objetivo ----------
    /** Cada vista nueva de la CELL se mezcla con lo guardado: 1 = solo la nueva, 0 = ignorarla. */
    public static double TARGET_BLEND = 0.35;
    /** Si una vista nueva difiere más que esto de lo guardado, se toma la nueva completa. */
    public static double TARGET_RESET_IN = 18;
    /** No se toman vistas si el chasis gira más rápido que esto (la imagen sale movida). */
    public static double MAX_TURN_RATE_FOR_TARGET_DEG_S = 200;
    /** No se toman vistas si la torreta gira más rápido que esto. */
    public static double MAX_TURRET_RATE_FOR_TARGET_DEG_S = 150;
    /** Si no se ve la CELL en este tiempo, se avisa al piloto que se apunta solo con odometría. */
    public static double TARGET_OLD_S = 4;
    /** Tiempo que debe verse otra CELL antes de cambiar de objetivo (evita saltar entre las dos). */
    public static double CELL_SWITCH_S = 0.3;

    /** Ajuste del punto de apuntado a la izquierda de los tags (pulgadas, visto desde el robot). */
    public static double AIM_LATERAL_OFFSET_IN = 0;
    /** Se suma a la distancia medida antes de buscar las RPM en la tabla (pulgadas). */
    public static double AIM_DISTANCE_OFFSET_IN = 0;

    // ---------- MegaTag 2 (apagado por defecto) ----------
    public static boolean USE_MEGATAG = false;
    /** Fracción de la corrección de MegaTag 2 que se aplica en cada lectura buena. */
    public static double MEGATAG_BLEND = 0.2;
    /** Correcciones más grandes que esto se ignoran (probablemente una CELL volteada). */
    public static double MEGATAG_MAX_JUMP_IN = 12;
    /** Mitad del campo en pulgadas: posiciones fuera de esto se ignoran. */
    public static double FIELD_HALF_IN = 72;
}
