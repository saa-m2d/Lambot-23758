package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Chasis mecanum de 4 motores con manejo field-centric.
 *
 * Configuración:
 *   Control Hub motor 0: frontLeft    motor 1: backLeft
 *               motor 2: frontRight   motor 3: backRight
 *               I2C bus 1: pinpoint  (tipo "goBILDA Pinpoint Odometry Computer")
 *
 * Por ahora solo se usa el giroscopio interno del Pinpoint para el heading; los pods de
 * odometría no se configuran todavía.
 */
public class Drivetrain {

    // TODO: verificar en el robot. Si un motor gira al revés, cambia su dirección aquí.
    static final DcMotorSimple.Direction LEFT_DIRECTION  = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    static final double STRAFE_CORRECTION = 1.1;   // compensa que el mecanum avanza menos de lado

    private final DcMotor frontLeft, backLeft, frontRight, backRight;
    private final GoBildaPinpointDriver pinpoint;
    private double headingOffset = 0;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft  = initMotor(hardwareMap, "frontLeft", LEFT_DIRECTION);
        backLeft   = initMotor(hardwareMap, "backLeft", LEFT_DIRECTION);
        frontRight = initMotor(hardwareMap, "frontRight", RIGHT_DIRECTION);
        backRight  = initMotor(hardwareMap, "backRight", RIGHT_DIRECTION);

        // Calibra el giroscopio, así que el robot debe estar quieto al inicializar.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
    }

    private static DcMotor initMotor(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    /** Lee el Pinpoint. Llamar una vez por ciclo antes de manejar. */
    public void update() {
        pinpoint.update();
    }

    /** Heading en radianes, relativo a la última vez que se llamó resetHeading(). */
    public double getHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS) - headingOffset;
    }

    /** Lo que el robot tiene enfrente en este momento pasa a ser "adelante". */
    public void resetHeading() {
        headingOffset = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    /**
     * Maneja relativo al campo.
     * @param forward hacia adelante del campo (-1 a 1)
     * @param strafe  hacia la derecha del campo (-1 a 1)
     * @param turn    giro, positivo a la derecha (-1 a 1)
     * @param scale   multiplicador de velocidad (1 = normal)
     */
    public void driveFieldCentric(double forward, double strafe, double turn, double scale) {
        double heading = getHeading();
        // Rotar el vector del campo al marco del robot
        double rotX = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotY = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        driveRobotCentric(rotY, rotX, turn, scale);
    }

    /** Maneja relativo al robot. Mismos parámetros que driveFieldCentric. */
    public void driveRobotCentric(double forward, double strafe, double turn, double scale) {
        strafe *= STRAFE_CORRECTION;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        frontLeft.setPower((forward + strafe + turn) / denominator * scale);
        backLeft.setPower((forward - strafe + turn) / denominator * scale);
        frontRight.setPower((forward - strafe - turn) / denominator * scale);
        backRight.setPower((forward + strafe - turn) / denominator * scale);
    }

    public void stop() {
        driveRobotCentric(0, 0, 0, 0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(getHeading()));
    }
}
