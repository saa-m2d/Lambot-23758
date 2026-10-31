package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Doble intake: un motor por intake. También sube las pelotas al shooter.
 *
 * Configuración:
 *   Expansion Hub motor 0: intakeFront    motor 1: intakeBack
 */
public class Intake {

    // TODO: verificar en el robot. Si un intake gira al revés, cambia su dirección aquí.
    static final DcMotorSimple.Direction FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;

    static final double INTAKE_POWER = 1.0;
    static final double FEED_POWER = 1.0;
    static final double REVERSE_POWER = -1.0;

    private final DcMotor front, back;
    private String state = "apagado";

    public Intake(HardwareMap hardwareMap) {
        front = hardwareMap.get(DcMotor.class, "intakeFront");
        back  = hardwareMap.get(DcMotor.class, "intakeBack");
        front.setDirection(FRONT_DIRECTION);
        back.setDirection(BACK_DIRECTION);
    }

    /** Recoger pelotas del piso. */
    public void collect() {
        setPower(INTAKE_POWER, "recogiendo");
    }

    /** Empujar pelotas hacia el shooter. */
    public void feed() {
        setPower(FEED_POWER, "alimentando shooter");
    }

    /** Sacar pelotas para desatorar. */
    public void reverse() {
        setPower(REVERSE_POWER, "reversa");
    }

    public void stop() {
        setPower(0, "apagado");
    }

    /** Detener mientras el shooter acelera, para que las pelotas no salgan lentas. */
    public void holdForShooter() {
        setPower(0, "esperando shooter");
    }

    private void setPower(double power, String newState) {
        front.setPower(power);
        back.setPower(power);
        state = newState;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intake", state);
    }
}
