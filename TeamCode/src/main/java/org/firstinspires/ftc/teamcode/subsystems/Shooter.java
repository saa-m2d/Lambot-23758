package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter de 2 flywheels independientes con control de velocidad por encoder.
 *
 * Configuración:
 *   Expansion Hub motor 2: shooterBack (rodillo + rueda trasera)
 *                 motor 3: shooterFront (rueda delantera)
 */
public class Shooter {

    // TODO: verificar en el robot. Si un flywheel gira al revés, cambia su dirección aquí.
    static final DcMotorSimple.Direction BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Ticks por vuelta del eje de salida del flywheel. Yellow Jacket 1:1 (6000 RPM) = 28.
    // Si el motor tiene caja de engranes, multiplica 28 por la relación (ej. 3.7:1 -> 103.6).
    static final double TICKS_PER_REV = 28.0;
    public static final double DEFAULT_RPM = 3000;
    static final double RPM_TOLERANCE = 100;   // qué tan cerca del objetivo cuenta como "listo"

    private final DcMotorEx back, front;
    private double targetRpm = DEFAULT_RPM;
    private boolean spinning = false;

    public Shooter(HardwareMap hardwareMap) {
        back  = initFlywheel(hardwareMap, "shooterBack", BACK_DIRECTION);
        front = initFlywheel(hardwareMap, "shooterFront", FRONT_DIRECTION);
    }

    private static DcMotorEx initFlywheel(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // que se detenga sola, sin frenar
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    /** Acelerar ambos flywheels al RPM objetivo. */
    public void spinUp() {
        spinning = true;
        setVelocity(rpmToTicksPerSec(targetRpm));
    }

    public void stop() {
        spinning = false;
        setVelocity(0);
    }

    private void setVelocity(double ticksPerSec) {
        back.setVelocity(ticksPerSec);
        front.setVelocity(ticksPerSec);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
        if (spinning) spinUp();
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getBackRpm() {
        return ticksPerSecToRpm(back.getVelocity());
    }

    public double getFrontRpm() {
        return ticksPerSecToRpm(front.getVelocity());
    }

    /** true cuando ambos flywheels están girando a la velocidad objetivo. */
    public boolean isReady() {
        return spinning
                && Math.abs(getBackRpm() - targetRpm) < RPM_TOLERANCE
                && Math.abs(getFrontRpm() - targetRpm) < RPM_TOLERANCE;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Shooter objetivo (RPM)", "%.0f", targetRpm);
        telemetry.addData("Shooter trasero / delantero (RPM)", "%.0f / %.0f", getBackRpm(), getFrontRpm());
        telemetry.addData("Shooter", isReady() ? "LISTO" : (spinning ? "acelerando..." : "apagado"));
    }

    static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }
}
