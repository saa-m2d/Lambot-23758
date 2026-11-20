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
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double integralLeft, integralRight;
    private double rawPower = Double.NaN;

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

    /** Solo para pruebas: potencia fija a los dos motores, sin control. NaN = control normal. */
    public void setRawPower(double power) {
        rawPower = power;
    }

    @Override
    public void periodic() {
        if (!isAvailable()) return;
        if (!Double.isNaN(rawPower)) {
            powerLeft = rawPower;
            powerRight = rawPower;
            ready = false;
        } else if (!enabled) {
            powerLeft = 0;
            powerRight = 0;
            integralLeft = 0;
            integralRight = 0;
            loopTimer.reset();
            ready = false;
        } else {
            double compensation = ShooterConstants.NOMINAL_VOLTAGE / battery.voltage();
            double dt = Math.min(loopTimer.seconds(), 0.1);
            loopTimer.reset();
            integralLeft = updateIntegral(integralLeft, targetLeft, rpmLeft, dt);
            integralRight = updateIntegral(integralRight, targetRight, rpmRight, dt);
            powerLeft = control(targetLeft, rpmLeft, compensation, integralLeft,
                    ShooterConstants.KS_LEFT, ShooterConstants.KV_LEFT);
            powerRight = control(targetRight, rpmRight, compensation, integralRight,
                    ShooterConstants.KS_RIGHT, ShooterConstants.KV_RIGHT);

            boolean within = Math.abs(targetLeft - rpmLeft) < ShooterConstants.READY_TOLERANCE_RPM
                    && Math.abs(targetRight - rpmRight) < ShooterConstants.READY_TOLERANCE_RPM;
            if (!within) readyTimer.reset();
            ready = within && readyTimer.seconds() >= ShooterConstants.READY_TIME_S;
        }
        left.setPower(powerLeft);
        right.setPower(powerRight);
    }

    /**
     * Integral del error, ya en unidades de potencia. Solo acumula cerca del objetivo (así no se
     * pasa al arrancar) y tiene un tope. Corrige que KV no sea exacto o que la batería esté baja:
     * sin ella, el motor se queda unas RPM abajo y nunca queda "listo".
     */
    private static double updateIntegral(double integral, double target, double measured, double dt) {
        if (target <= 0) return 0;
        double error = target - measured;
        if (Math.abs(error) > ShooterConstants.INTEGRAL_ZONE_RPM) return integral;
        double max = ShooterConstants.INTEGRAL_MAX_POWER;
        return Range.clip(integral + ShooterConstants.KI * error * dt, -max, max);
    }

    private static double control(double target, double measured, double voltageCompensation, double integral,
                                  double ks, double kv) {
        if (target <= 0) return 0;
        double feedforward = (ks + kv * target) * voltageCompensation;
        double feedback = ShooterConstants.KP * (target - measured) + integral;
        return Range.clip(feedforward + feedback, 0, 1);   // sin frenar en reversa
    }

    /** true si el motor ya está a potencia máxima y aun así no llega (pide menos RPM o carga la batería). */
    public boolean isSaturated() {
        return enabled && ((powerLeft >= 0.99 && rpmLeft < targetLeft - ShooterConstants.READY_TOLERANCE_RPM)
                || (powerRight >= 0.99 && rpmRight < targetRight - ShooterConstants.READY_TOLERANCE_RPM));
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
