package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotMap;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;

/**
 * Shooter de 2 Yellow Jacket 6000 RPM, cada uno con su propia velocidad objetivo (sin hood, la
 * distancia se controla con la diferencia y el total de RPM).
 *
 * Control por motor: feedforward (KS + KV, compensado por voltaje) + proporcional (KP).
 */
public class Shooter implements Subsystem {
    private final DcMotorEx left, right;
    private final BatteryMonitor battery;

    private boolean enabled;
    private double targetLeft, targetRight;   // RPM
    private double rpmLeft, rpmRight;
    private double powerLeft, powerRight;
    private boolean ready;
    private final ElapsedTime readyTimer = new ElapsedTime();

    public Shooter(HardwareCheck hardware, BatteryMonitor battery) {
        this.battery = battery;
        left = hardware.get(DcMotorEx.class, RobotMap.SHOOTER_LEFT, "motor Control Hub 0, con encoder");
        right = hardware.get(DcMotorEx.class, RobotMap.SHOOTER_RIGHT, "motor Control Hub 1, con encoder");
        setup(left, RobotMap.SHOOTER_LEFT_DIRECTION);
        setup(right, RobotMap.SHOOTER_RIGHT_DIRECTION);
    }

    private static void setup(DcMotorEx motor, DcMotorSimple.Direction direction) {
        if (motor == null) return;
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // el control de velocidad es propio
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public boolean isAvailable() {
        return left != null && right != null;
    }

    /** Lee la velocidad de los dos motores (datos del bulk read). */
    public void readSensors() {
        rpmLeft = left != null ? toRpm(left.getVelocity()) : 0;
        rpmRight = right != null ? toRpm(right.getVelocity()) : 0;
    }

    private static double toRpm(double ticksPerSecond) {
        return ticksPerSecond / RobotMap.SHOOTER_TICKS_PER_REV * 60.0;
    }

    /** Pide RPM a cada motor. 0 en los dos = apagado. */
    public void setTargets(double rpmLeftTarget, double rpmRightTarget) {
        double newLeft = Range.clip(rpmLeftTarget, 0, ShooterConstants.MAX_RPM);
        double newRight = Range.clip(rpmRightTarget, 0, ShooterConstants.MAX_RPM);
        enabled = newLeft > 0 || newRight > 0;
        targetLeft = newLeft;
        targetRight = newRight;
    }

    public void off() {
        setTargets(0, 0);
    }

    @Override
    public void periodic() {
        if (!isAvailable()) return;
        if (!enabled) {
            powerLeft = 0;
            powerRight = 0;
            ready = false;
        } else {
            double compensation = ShooterConstants.NOMINAL_VOLTAGE / battery.voltage();
            powerLeft = control(targetLeft, rpmLeft, compensation);
            powerRight = control(targetRight, rpmRight, compensation);

            boolean within = Math.abs(targetLeft - rpmLeft) < ShooterConstants.READY_TOLERANCE_RPM
                    && Math.abs(targetRight - rpmRight) < ShooterConstants.READY_TOLERANCE_RPM;
            if (!within) readyTimer.reset();
            ready = within && readyTimer.seconds() >= ShooterConstants.READY_TIME_S;
        }
        left.setPower(powerLeft);
        right.setPower(powerRight);
    }

    private static double control(double target, double measured, double voltageCompensation) {
        if (target <= 0) return 0;
        double feedforward = (ShooterConstants.KS + ShooterConstants.KV * target) * voltageCompensation;
        double feedback = ShooterConstants.KP * (target - measured);
        return Range.clip(feedforward + feedback, 0, 1);   // sin frenar en reversa
    }

    /** true si los dos motores están en su objetivo (y el shooter está encendido). */
    public boolean isReady() {
        return ready;
    }

    public boolean isEnabled() { return enabled; }
    public double rpmLeft() { return rpmLeft; }
    public double rpmRight() { return rpmRight; }
    public double targetLeft() { return targetLeft; }
    public double targetRight() { return targetRight; }
    public double powerLeft() { return powerLeft; }
    public double powerRight() { return powerRight; }

    /** Corriente de los dos motores (A). Lectura lenta: llamar pocas veces por segundo. */
    public double[] readCurrents() {
        if (!isAvailable()) return new double[0];
        return new double[]{left.getCurrent(CurrentUnit.AMPS), right.getCurrent(CurrentUnit.AMPS)};
    }

    @Override
    public void stop() {
        off();
        periodic();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (!isAvailable()) {
            telemetry.addData("Shooter", "FALTA HARDWARE");
            return;
        }
        telemetry.addData("Shooter", "%s  izq %.0f/%.0f  der %.0f/%.0f RPM",
                !enabled ? "apagado" : ready ? "LISTO" : "subiendo",
                rpmLeft, targetLeft, rpmRight, targetRight);
    }
}
