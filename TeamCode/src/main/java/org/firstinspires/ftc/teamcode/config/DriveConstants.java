package org.firstinspires.ftc.teamcode.config;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Manejo del chasis. Todo se puede cambiar en vivo desde Panels (http://192.168.43.1:8001).
 * Lo que sirva, cópialo aquí: lo que se cambia en Panels se pierde al reiniciar el robot.
 */
@Configurable
public class DriveConstants {
    /** Debajo de esto, el stick cuenta como suelto. */
    public static double STICK_DEADBAND = 0.05;
    /** Curva del stick: 1 = lineal; más alto = más control a baja velocidad. */
    public static double STICK_EXPONENT = 1.5;
    /** Velocidad máxima de giro (fracción de la potencia). */
    public static double TURN_SCALE = 0.85;
    /** Velocidad con el modo lento (gatillo izquierdo del piloto). */
    public static double SLOW_MODE_SCALE = 0.35;
    /** Compensa que el mecanum se mueve menos de lado que hacia adelante. */
    public static double STRAFE_CORRECTION = 1.1;
    /** true = el chasis frena al soltar los sticks; false = rueda libre. */
    public static boolean BRAKE_WHEN_STOPPED = true;
}
