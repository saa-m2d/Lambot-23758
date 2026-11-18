package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Autónomo. Las posiciones van en pulgadas y grados, en las coordenadas del campo de FTC
 * (origen al centro del campo). Formato de cada pose: {x, y, heading en grados}.
 *
 * Mientras PATHS_CONFIGURED sea false (o Pedro no esté tuneado), el autónomo hace la versión
 * sencilla: dispara las pelotas precargadas desde donde arranca y sale de la zona de inicio.
 * Para la versión completa: mide las poses en el campo, escríbelas aquí y pon
 * PATHS_CONFIGURED = true.
 */
@Configurable
public class AutoConstants {
    public static boolean PATHS_CONFIGURED = false;

    // ---------- Alianza roja ----------
    public static double[] RED_START = {0, 0, 0};
    public static double[] RED_SHOOT = {0, 0, 0};
    public static double[] RED_PICKUP_START = {0, 0, 0};
    public static double[] RED_PICKUP_END = {0, 0, 0};
    public static double[] RED_LEAVE = {0, 0, 0};
    /** Heading (grados) que para los pilotos rojos es "adelante" en el TeleOp. */
    public static double RED_DRIVER_FORWARD_DEG = 0;

    // ---------- Alianza azul ----------
    public static double[] BLUE_START = {0, 0, 0};
    public static double[] BLUE_SHOOT = {0, 0, 0};
    public static double[] BLUE_PICKUP_START = {0, 0, 0};
    public static double[] BLUE_PICKUP_END = {0, 0, 0};
    public static double[] BLUE_LEAVE = {0, 0, 0};
    public static double BLUE_DRIVER_FORWARD_DEG = 0;

    // ---------- Posición aproximada de las CELLS (para apuntar antes de verlas) ----------
    /** true para usar las posiciones de abajo hasta que la cámara vea la CELL. {x, y} */
    public static boolean USE_CELL_PRIORS = false;
    public static double[] RED_FAR_CELL = {0, 0};
    public static double[] RED_AUDIENCE_CELL = {0, 0};
    public static double[] BLUE_FAR_CELL = {0, 0};
    public static double[] BLUE_AUDIENCE_CELL = {0, 0};

    // ---------- Tiempos ----------
    /** Disparar las precargadas al empezar. */
    public static boolean SHOOT_PRELOAD = true;
    /** Una vez que empieza a disparar, cuánto tiempo sigue (con el tope abriendo y cerrando). */
    public static double SHOOT_TIME_S = 2.5;
    /** Si en este tiempo no logra apuntar y tener RPM, deja de esperar y sigue. */
    public static double SHOOT_TIMEOUT_S = 5.0;
    /** Tiempo máximo por trayectoria. */
    public static double PATH_TIMEOUT_S = 6.0;
    /** A este segundo el autónomo termina y guarda la posición para el TeleOp. */
    public static double AUTO_END_S = 29.0;

    // ---------- Salida sencilla (sin Pedro) ----------
    public static double SIMPLE_LEAVE_FORWARD = 0.4;
    public static double SIMPLE_LEAVE_LEFT = 0;
    public static double SIMPLE_LEAVE_TIME_S = 1.2;
}
