package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.MechanismConstants;
import org.firstinspires.ftc.teamcode.config.RobotMap;

/** Los dos intakes (enfrente y atrás). Potencia positiva = meter pelotas. */
public class Intake implements Subsystem {
    public enum Mode { OFF, IN, OUT }

    private final DcMotorEx left, right;
    private Mode front = Mode.OFF, back = Mode.OFF;

    // Detección de atasco: corriente alta sostenida (se lee pocas veces por segundo)
    private final ElapsedTime jamTimer = new ElapsedTime();
    private boolean jammed;
    private double lastAmpsLeft, lastAmpsRight;

    public Intake(HardwareCheck hardware) {
        left = hardware.get(DcMotorEx.class, RobotMap.INTAKE_LEFT, "motor Expansion Hub 1");
        right = hardware.get(DcMotorEx.class, RobotMap.INTAKE_RIGHT, "motor Expansion Hub 0");
        setup(left, RobotMap.INTAKE_LEFT_DIRECTION);
        setup(right, RobotMap.INTAKE_RIGHT_DIRECTION);
    }

    private static void setup(DcMotorEx motor, DcMotorSimple.Direction direction) {
        if (motor == null) return;
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public boolean isAvailable() {
        return left != null && right != null;
    }

    public void set(Mode both) {
        front = both;
        back = both;
    }

    public void setFront(Mode mode) { front = mode; }
    public void setBack(Mode mode) { back = mode; }

    private static double power(Mode mode) {
        switch (mode) {
            case IN: return MechanismConstants.INTAKE_POWER;
            case OUT: return -MechanismConstants.INTAKE_REVERSE_POWER;
            default: return 0;
        }
    }

    @Override
    public void periodic() {
        double frontPower = power(front);
        double backPower = power(back);
        boolean leftIsFront = MechanismConstants.INTAKE_LEFT_IS_FRONT;
        if (left != null) left.setPower(leftIsFront ? frontPower : backPower);
        if (right != null) right.setPower(leftIsFront ? backPower : frontPower);
    }

    /** Revisa la corriente. Lectura lenta: llamar pocas veces por segundo. */
    public void checkCurrent() {
        lastAmpsLeft = left != null ? left.getCurrent(CurrentUnit.AMPS) : 0;
        lastAmpsRight = right != null ? right.getCurrent(CurrentUnit.AMPS) : 0;
        boolean high = Math.max(lastAmpsLeft, lastAmpsRight) > MechanismConstants.INTAKE_JAM_AMPS;
        if (!high) jamTimer.reset();
        jammed = high && jamTimer.seconds() > MechanismConstants.INTAKE_JAM_TIME_S;
    }

    public boolean isJammed() {
        return jammed;
    }

    @Override
    public void stop() {
        set(Mode.OFF);
        periodic();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intakes", isAvailable()
                ? String.format("enfrente %s  atrás %s  (%.1f A / %.1f A)", front, back, lastAmpsLeft, lastAmpsRight)
                : "FALTA HARDWARE");
    }
}
