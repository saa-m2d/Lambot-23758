package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.DriveConstants;
import org.firstinspires.ftc.teamcode.config.RobotMap;

/**
 * Chasis mecanum de 4 motores (goBILDA 1620 RPM, sin encoders).
 *
 * Convención: adelante y a la izquierda positivos; giro positivo = antihorario.
 */
public class Drivetrain implements Subsystem {
    private final DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private final boolean available;

    private double forward, left, turn;   // lo pedido, marco del robot
    private boolean brake = DriveConstants.BRAKE_WHEN_STOPPED;

    public Drivetrain(HardwareCheck hardware) {
        frontLeft = hardware.get(DcMotorEx.class, RobotMap.FRONT_LEFT, "motor Control Hub 2");
        backLeft = hardware.get(DcMotorEx.class, RobotMap.BACK_LEFT, "motor Control Hub 3");
        frontRight = hardware.get(DcMotorEx.class, RobotMap.FRONT_RIGHT, "motor Expansion Hub 3");
        backRight = hardware.get(DcMotorEx.class, RobotMap.BACK_RIGHT, "motor Expansion Hub 2");
        available = frontLeft != null && backLeft != null && frontRight != null && backRight != null;

        setup(frontLeft, RobotMap.FRONT_LEFT_DIRECTION);
        setup(backLeft, RobotMap.BACK_LEFT_DIRECTION);
        setup(frontRight, RobotMap.FRONT_RIGHT_DIRECTION);
        setup(backRight, RobotMap.BACK_RIGHT_DIRECTION);
    }

    private void setup(DcMotorEx motor, DcMotorSimple.Direction direction) {
        if (motor == null) return;
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(brakeBehavior());
    }

    private DcMotor.ZeroPowerBehavior brakeBehavior() {
        return brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
    }

    @Override
    public boolean isAvailable() {
        return available;
    }

    /** Movimiento en el marco del robot. Valores de -1 a 1. */
    public void driveRobotCentric(double forward, double left, double turn) {
        this.forward = forward;
        this.left = left;
        this.turn = turn;
    }

    /**
     * Movimiento en el marco del piloto: "adelante" es hacia donde el piloto definió adelante.
     *
     * @param headingFromDriverForward heading del robot menos el heading "adelante" del piloto (rad)
     */
    public void driveFieldCentric(double forward, double left, double turn, double headingFromDriverForward) {
        double cos = Math.cos(headingFromDriverForward);
        double sin = Math.sin(headingFromDriverForward);
        driveRobotCentric(forward * cos + left * sin, -forward * sin + left * cos, turn);
    }

    @Override
    public void periodic() {
        if (!available) return;
        if (brake != DriveConstants.BRAKE_WHEN_STOPPED) {
            brake = DriveConstants.BRAKE_WHEN_STOPPED;
            for (DcMotorEx motor : new DcMotorEx[]{frontLeft, backLeft, frontRight, backRight}) {
                motor.setZeroPowerBehavior(brakeBehavior());
            }
        }
        double strafe = left * DriveConstants.STRAFE_CORRECTION;
        double fl = forward - strafe - turn;
        double bl = forward + strafe - turn;
        double fr = forward + strafe + turn;
        double br = forward - strafe + turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br))));
        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    @Override
    public void stop() {
        driveRobotCentric(0, 0, 0);
        periodic();
    }

    /** Corriente de los 4 motores (A). Lectura lenta: llamar pocas veces por segundo. */
    public double[] readCurrents() {
        if (!available) return new double[0];
        return new double[]{
                frontLeft.getCurrent(CurrentUnit.AMPS),
                backLeft.getCurrent(CurrentUnit.AMPS),
                frontRight.getCurrent(CurrentUnit.AMPS),
                backRight.getCurrent(CurrentUnit.AMPS)};
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Chasis", available
                ? String.format("adelante %.2f  izq %.2f  giro %.2f", forward, left, turn)
                : "FALTA HARDWARE");
    }
}
