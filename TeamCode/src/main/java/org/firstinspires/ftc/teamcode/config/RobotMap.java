package org.firstinspires.ftc.teamcode.config;

/**
 * Nombres EXACTOS de cada dispositivo en la configuración del robot (Driver Hub).
 * Todo el código usa estas constantes: si cambias un nombre, cámbialo solo aquí.
 *
 * Puerto por puerto (ver también CHECKLIST.md):
 *
 *   Control Hub - motores        Expansion Hub - motores
 *     0: shooterLeft               0: intakeRight
 *     1: shooterRight              1: intakeLeft
 *     2: frontLeft                 2: backRight
 *     3: backLeft                  3: frontRight
 *
 *   Control Hub - servos         Expansion Hub - servos
 *     4: turretLeft                0: tope
 *     5: flowerLeft                1: turretRight
 *                                  2: flowerRight
 *
 *   Control Hub - analógico 0: turretEnc (4.º cable de turretLeft)
 *   Expansion Hub - I2C bus 0: pinpoint
 *   Control Hub - USB 3.0 (azul): limelight
 *
 * Tipos: turretLeft y turretRight son "Continuous Rotation Servo" (firmware CR del Axon);
 * tope, flowerLeft y flowerRight son "Servo".
 */
public final class RobotMap {
    private RobotMap() {}

    // Chasis mecanum (goBILDA 1620 RPM, sin encoder conectado)
    public static final String FRONT_LEFT  = "frontLeft";
    public static final String BACK_LEFT   = "backLeft";
    public static final String FRONT_RIGHT = "frontRight";
    public static final String BACK_RIGHT  = "backRight";

    // Shooter (goBILDA Yellow Jacket 6000 RPM, con encoder)
    public static final String SHOOTER_LEFT  = "shooterLeft";
    public static final String SHOOTER_RIGHT = "shooterRight";

    // Intakes (goBILDA 1620 RPM)
    public static final String INTAKE_LEFT  = "intakeLeft";
    public static final String INTAKE_RIGHT = "intakeRight";

    // Servos
    public static final String TURRET_LEFT  = "turretLeft";   // Axon MINI MK2, modo continuo
    public static final String TURRET_RIGHT = "turretRight";  // Axon MINI MK2, modo continuo
    public static final String TOPE         = "tope";         // Axon MINI MK2 (indexer)
    public static final String FLOWER_LEFT  = "flowerLeft";   // Swyft speed servo
    public static final String FLOWER_RIGHT = "flowerRight";  // Swyft speed servo

    // Sensores
    public static final String PINPOINT  = "pinpoint";
    public static final String LIMELIGHT = "limelight";

    /** 4.º cable de turretLeft (posición del servo dentro de una vuelta), tipo "Analog Input". */
    public static final String TURRET_ENCODER = "turretEnc";

    // Pinpoint: posición de los pods respecto al centro del robot, en milímetros.
    // X pod (forward): negativo = a la derecha del centro. Y pod (strafe): negativo = atrás del centro.
    public static final double PINPOINT_X_POD_OFFSET_MM = -82.5;
    public static final double PINPOINT_Y_POD_OFFSET_MM = -70.74;

    // Ticks por vuelta del eje de un Yellow Jacket 1:1 (6000 RPM)
    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Pulso que aceptan los Axon MK2 en modo posición (el SDK usa 600-2400 por defecto).
    // Para los servos de la torreta en modo continuo, ver TurretConfig.
    public static final double SERVO_PWM_MIN_US = 500;
    public static final double SERVO_PWM_MAX_US = 2500;
}
