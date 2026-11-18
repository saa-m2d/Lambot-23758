package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

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
 *   Control Hub - analógico 0 (conector 0-1, pin azul):   turretEncLeft  (4.º cable de turretLeft)
 *   Expansion Hub - analógico 2 (conector 2-3, pin azul): turretEncRight (4.º cable de turretRight)
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

    // Direcciones medidas con "Test 1: Motores" (2026-11-17). Con estas, potencia positiva =
    // chasis hacia adelante, shooter lanza, intake mete.
    public static final Direction FRONT_LEFT_DIRECTION    = Direction.REVERSE;
    public static final Direction BACK_LEFT_DIRECTION     = Direction.REVERSE;
    public static final Direction FRONT_RIGHT_DIRECTION   = Direction.FORWARD;
    public static final Direction BACK_RIGHT_DIRECTION    = Direction.FORWARD;
    public static final Direction SHOOTER_LEFT_DIRECTION  = Direction.REVERSE;
    public static final Direction SHOOTER_RIGHT_DIRECTION = Direction.REVERSE;
    public static final Direction INTAKE_LEFT_DIRECTION   = Direction.FORWARD;
    public static final Direction INTAKE_RIGHT_DIRECTION  = Direction.FORWARD;

    // Servos
    public static final String TURRET_LEFT  = "turretLeft";   // Axon MINI MK2, modo continuo
    public static final String TURRET_RIGHT = "turretRight";  // Axon MINI MK2, modo continuo
    public static final String TOPE         = "tope";         // Axon MINI MK2 (indexer)
    public static final String FLOWER_LEFT  = "flowerLeft";   // Swyft speed servo
    public static final String FLOWER_RIGHT = "flowerRight";  // Swyft speed servo

    // Posiciones del tope medidas con "Test 2: Servos" (2026-11-17), con PWM 500-2500 µs.
    public static final double TOPE_OPEN   = 0.700;   // 1900 µs
    public static final double TOPE_CLOSED = 0.8975;  // 2295 µs

    // Posiciones del flower, cada servo medido solo (2026-11-17). Están en espejo: giran al revés.
    // flowerRight abajo se corrigió de 0.750 a 0.490 porque zumbaba con los dos juntos.
    // Confirmar con el modo "juntos" de "Test 2: Servos" que no zumben en ningún punto.
    public static final double FLOWER_LEFT_UP    = 0.830;   // 2160 µs
    public static final double FLOWER_LEFT_DOWN  = 0.4875;  // 1475 µs
    public static final double FLOWER_RIGHT_UP   = 0.200;   //  900 µs
    public static final double FLOWER_RIGHT_DOWN = 0.490;   // 1480 µs

    // Sensores
    public static final String PINPOINT  = "pinpoint";
    public static final String LIMELIGHT = "limelight";

    /** 4.º cable de cada servo de la torreta (posición dentro de una vuelta), tipo "Analog Input". */
    public static final String TURRET_ENCODER_LEFT  = "turretEncLeft";
    public static final String TURRET_ENCODER_RIGHT = "turretEncRight";

    // Pinpoint: posición de los pods respecto al centro del robot, en milímetros.
    // X pod (forward): negativo = a la derecha del centro. Y pod (strafe): negativo = atrás del centro.
    public static final double PINPOINT_X_POD_OFFSET_MM = -82.5;
    public static final double PINPOINT_Y_POD_OFFSET_MM = -70.74;
    // Dirección de cada pod: se confirma con "Test 3: Pinpoint" (empujar adelante: X sube;
    // empujar a la izquierda: Y sube). Lo usan la prueba, el TeleOp y PedroPathing.
    public static final EncoderDirection PINPOINT_X_POD_DIRECTION = EncoderDirection.FORWARD;
    public static final EncoderDirection PINPOINT_Y_POD_DIRECTION = EncoderDirection.FORWARD;

    // Ticks por vuelta del eje de un Yellow Jacket 1:1 (6000 RPM)
    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Pulso que aceptan los Axon MK2 en modo posición (el SDK usa 600-2400 por defecto).
    // Para los servos de la torreta en modo continuo, ver TurretConfig.
    public static final double SERVO_PWM_MIN_US = 500;
    public static final double SERVO_PWM_MAX_US = 2500;
}
