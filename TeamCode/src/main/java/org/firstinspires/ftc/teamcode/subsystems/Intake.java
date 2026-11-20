package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.MechanismConstants;
import org.firstinspires.ftc.teamcode.config.RobotMap;

/**
 * Los dos intakes (enfrente y atrás). Potencia positiva = meter pelotas.
 *
 * Antiatasco (cada intake por separado): si al meter la corriente se queda alta un momento, hace
 * un tironcito en reversa muy corto y sigue metiendo a potencia completa. Nunca baja la potencia
 * por su cuenta. Justo después de arrancar (o de cada tironcito) no mide, porque el motor jala un
 * pico de corriente al acelerar que no es un atasco.
 */
public class Intake implements Subsystem {
    public enum Mode { OFF, IN, OUT }

    private static class Side {
        final DcMotorEx motor;
        Mode mode = Mode.OFF;
        boolean unjamming;
        double amps;
        boolean high;
        int unjams;
        final ElapsedTime highTimer = new ElapsedTime();
        final ElapsedTime unjamTimer = new ElapsedTime();
        /** Desde que arrancó a meter o terminó el último tironcito. */
        final ElapsedTime sinceStart = new ElapsedTime();

        Side(DcMotorEx motor) {
            this.motor = motor;
        }
    }

    private final Side left, right;
    private final ElapsedTime currentTimer = new ElapsedTime();

    public Intake(HardwareCheck hardware) {
        DcMotorEx leftMotor = hardware.get(DcMotorEx.class, RobotMap.INTAKE_LEFT, "motor Expansion Hub 1");
        DcMotorEx rightMotor = hardware.get(DcMotorEx.class, RobotMap.INTAKE_RIGHT, "motor Expansion Hub 0");
        setup(leftMotor, RobotMap.INTAKE_LEFT_DIRECTION);
        setup(rightMotor, RobotMap.INTAKE_RIGHT_DIRECTION);
        left = new Side(leftMotor);
        right = new Side(rightMotor);
    }

    private static void setup(DcMotorEx motor, DcMotorSimple.Direction direction) {
        if (motor == null) return;
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public boolean isAvailable() {
        return left.motor != null && right.motor != null;
    }

    private Side front() { return MechanismConstants.INTAKE_LEFT_IS_FRONT ? left : right; }
    private Side back() { return MechanismConstants.INTAKE_LEFT_IS_FRONT ? right : left; }

    public void set(Mode both) {
        setSide(left, both);
        setSide(right, both);
    }

    public void setFront(Mode mode) { setSide(front(), mode); }
    public void setBack(Mode mode) { setSide(back(), mode); }

    private static void setSide(Side side, Mode mode) {
        if (mode != side.mode) {
            side.unjamming = false;
            side.sinceStart.reset();   // el arranque no cuenta como atasco
        }
        side.mode = mode;
    }

    @Override
    public void periodic() {
        apply(left);
        apply(right);
    }

    private static void apply(Side side) {
        if (side.motor == null) return;
        double power;
        if (side.mode == Mode.OUT) {
            power = -MechanismConstants.INTAKE_REVERSE_POWER;
        } else if (side.mode == Mode.OFF) {
            power = 0;
        } else {
            if (side.unjamming && side.unjamTimer.seconds() > MechanismConstants.INTAKE_UNJAM_S) {
                side.unjamming = false;
                side.sinceStart.reset();
            }
            power = side.unjamming ? -MechanismConstants.INTAKE_UNJAM_POWER : MechanismConstants.INTAKE_POWER;
        }
        side.motor.setPower(power);
    }

    /**
     * Llamar en cada ciclo: lee la corriente cada INTAKE_CURRENT_PERIOD_S, solo mientras algún
     * intake está metiendo (cada lectura tarda unos milisegundos).
     */
    public void updateCurrent() {
        if (currentTimer.seconds() < MechanismConstants.INTAKE_CURRENT_PERIOD_S) return;
        currentTimer.reset();
        if (left.mode != Mode.IN && right.mode != Mode.IN) return;
        checkCurrent();
    }

    /** Revisa la corriente y, si está atorado, hace un tironcito. */
    public void checkCurrent() {
        check(left);
        check(right);
    }

    private static void check(Side side) {
        if (side.motor == null) return;
        side.amps = side.motor.getCurrent(CurrentUnit.AMPS);
        boolean high = side.amps > MechanismConstants.INTAKE_JAM_AMPS;
        if (high && !side.high) side.highTimer.reset();
        side.high = high;
        if (side.mode != Mode.IN || side.unjamming) return;
        if (side.sinceStart.seconds() < MechanismConstants.INTAKE_START_IGNORE_S) return;
        if (!high || side.highTimer.seconds() < MechanismConstants.INTAKE_JAM_TIME_S) return;
        side.unjamming = true;
        side.unjams++;
        side.unjamTimer.reset();
        side.highTimer.reset();
    }

    /** Cuántos tironcitos lleva cada intake (para ajustar en pruebas). */
    public int unjamCount() {
        return left.unjams + right.unjams;
    }

    @Override
    public void stop() {
        set(Mode.OFF);
        periodic();
    }

    private static String describe(Side side) {
        if (side.motor == null) return "falta";
        String state = side.mode != Mode.IN ? side.mode.toString() : side.unjamming ? "tironcito" : "IN";
        return String.format("%s %.1f A (%d tironcitos)", state, side.amps, side.unjams);
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Intakes", isAvailable()
                ? String.format("enfrente %s  |  atrás %s", describe(front()), describe(back()))
                : "FALTA HARDWARE");
    }
}
