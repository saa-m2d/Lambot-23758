package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotMap;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.config.TurretConstants;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Torreta: 2 Axon MINI MK2 en modo continuo (CR) sobre el mismo eje, 2 vueltas de servo = 1 de
 * torreta. El ángulo sale del encoder izquierdo (4.º cable); el derecho se compara con él.
 *
 * Ángulos en grados de torreta desde home (home = apuntando a la izquierda del robot).
 * Positivo = antihorario. Rango con la cadena portacables: -40° a +360°.
 */
public class Turret implements Subsystem {
    public enum Mode { OFF, MANUAL, POSITION }

    private final CRServoImplEx left, right;
    private final TurretEncoder encoderLeft, encoderRight, primary;
    private final boolean available;

    private Mode mode = Mode.OFF;
    private boolean pwmEnabled;
    private double manualPower;
    private double targetDeg, targetRateDegS;

    private double angleDeg, rateDegS, lastAngleDeg;
    private final ElapsedTime rateTimer = new ElapsedTime();
    private boolean hasLastAngle;
    private double appliedPower;

    // Fallas
    private boolean wrongDirection;
    private boolean stalled;
    private boolean mismatch;
    private int segmentSign;
    private double segmentStartDeg;
    private double stallReferenceDeg;
    private final ElapsedTime stallTimer = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();
    private final ElapsedTime mismatchTimer = new ElapsedTime();

    /** @param hintDeg ángulo esperado al iniciar: 0 si la torreta está en home. */
    public Turret(HardwareCheck hardware, double hintDeg) {
        left = hardware.get(CRServoImplEx.class, RobotMap.TURRET_LEFT, "Continuous Rotation Servo, Control Hub servo 4");
        right = hardware.get(CRServoImplEx.class, RobotMap.TURRET_RIGHT, "Continuous Rotation Servo, Expansion Hub servo 1");
        AnalogInput inputLeft = hardware.get(AnalogInput.class, RobotMap.TURRET_ENCODER_LEFT, "Analog Input, Control Hub analógico 0");
        AnalogInput inputRight = hardware.get(AnalogInput.class, RobotMap.TURRET_ENCODER_RIGHT, "Analog Input, Expansion Hub analógico 2");

        encoderLeft = inputLeft == null ? null : new TurretEncoder(inputLeft, TurretConfig.LEFT_ENCODER_REVERSED,
                TurretConfig.LEFT_ENCODER_V_MIN, TurretConfig.LEFT_ENCODER_V_MAX, TurretConfig.LEFT_HOME_VOLTAGE, hintDeg);
        encoderRight = inputRight == null ? null : new TurretEncoder(inputRight, TurretConfig.RIGHT_ENCODER_REVERSED,
                TurretConfig.RIGHT_ENCODER_V_MIN, TurretConfig.RIGHT_ENCODER_V_MAX, TurretConfig.RIGHT_HOME_VOLTAGE, hintDeg);
        primary = encoderLeft != null ? encoderLeft : encoderRight;

        // Se puede mover con un solo servo, pero sin ningún encoder no hay control seguro
        available = (left != null || right != null) && primary != null;

        PwmControl.PwmRange crRange = new PwmControl.PwmRange(TurretConfig.CR_PWM_MIN_US, TurretConfig.CR_PWM_MAX_US);
        for (CRServoImplEx servo : new CRServoImplEx[]{left, right}) {
            if (servo == null) continue;
            servo.setPwmRange(crRange);
            servo.setPwmDisable();
        }
        if (primary != null) {
            angleDeg = primary.turretDeg();
            lastAngleDeg = angleDeg;
        }
    }

    @Override
    public boolean isAvailable() {
        return available;
    }

    /** Límite inferior usable (con margen). */
    public static double lowLimit() {
        return TurretConfig.MIN_DEG + TurretConfig.LIMIT_MARGIN_DEG;
    }

    /** Límite superior usable (con margen). */
    public static double highLimit() {
        return TurretConfig.MAX_DEG - TurretConfig.LIMIT_MARGIN_DEG;
    }

    /** Lee los encoders. Llamar una vez por ciclo antes de decidir qué hacer. */
    public void readSensors() {
        if (encoderLeft != null) encoderLeft.read();
        if (encoderRight != null) encoderRight.read();
        if (primary == null) return;
        angleDeg = primary.turretDeg();
        double dt = rateTimer.seconds();
        rateTimer.reset();
        if (hasLastAngle && dt > 1e-4) {
            rateDegS = MathUtil.lowPass(rateDegS, (angleDeg - lastAngleDeg) / dt, 0.5);
        }
        lastAngleDeg = angleDeg;
        hasLastAngle = true;

        if (encoderLeft != null && encoderRight != null
                && Math.abs(encoderLeft.turretDeg() - encoderRight.turretDeg()) > TurretConfig.ENCODER_MISMATCH_DEG) {
            mismatch = mismatchTimer.seconds() > TurretConstants.MISMATCH_PERSIST_S;
        } else {
            mismatchTimer.reset();
            mismatch = false;
        }
    }

    /**
     * De todos los ángulos equivalentes (cada 360°) al pedido, el que cabe en el rango y queda más
     * cerca de donde está la torreta.
     */
    public double chooseAngle(double desiredDeg) {
        double wrapped = MathUtil.wrap180(desiredDeg);
        double best = Double.NaN;
        for (int turns = -2; turns <= 2; turns++) {
            double candidate = wrapped + 360.0 * turns;
            if (candidate < lowLimit() || candidate > highLimit()) continue;
            if (Double.isNaN(best) || Math.abs(candidate - angleDeg) < Math.abs(best - angleDeg)) {
                best = candidate;
            }
        }
        return Double.isNaN(best) ? Range.clip(wrapped, lowLimit(), highLimit()) : best;
    }

    /** Ir a un ángulo (ya dentro del rango), con la velocidad a la que se mueve el objetivo. */
    public void setTarget(double deg, double rateDegS) {
        mode = Mode.POSITION;
        targetDeg = Range.clip(deg, lowLimit(), highLimit());
        targetRateDegS = rateDegS;
    }

    /** Potencia directa (-1 a 1, positivo = antihorario), respetando límites y fallas. */
    public void setManualPower(double power) {
        mode = Mode.MANUAL;
        manualPower = Range.clip(power, -1, 1) * TurretConstants.MANUAL_MAX_POWER;
    }

    /** Deja la torreta suelta (sin señal). */
    public void disable() {
        mode = Mode.OFF;
    }

    /** Borra la falla de dirección (después de revisar el robot). */
    public void clearFaults() {
        wrongDirection = false;
        stalled = false;
        segmentSign = 0;
    }

    @Override
    public void periodic() {
        if (!available) return;

        double power;
        switch (mode) {
            case POSITION:
                power = positionControl();
                break;
            case MANUAL:
                power = manualPower;
                break;
            default:
                power = 0;
        }

        power = applyLimits(power);
        power = applyFaultChecks(power);
        appliedPower = power;

        boolean wantEnabled = mode != Mode.OFF;
        if (wantEnabled != pwmEnabled) {
            for (CRServoImplEx servo : new CRServoImplEx[]{left, right}) {
                if (servo == null) continue;
                if (wantEnabled) servo.setPwmEnable(); else servo.setPwmDisable();
            }
            pwmEnabled = wantEnabled;
        }
        if (left != null) left.setPower(TurretConfig.LEFT_REVERSED ? -power : power);
        if (right != null) right.setPower(TurretConfig.RIGHT_REVERSED ? -power : power);
    }

    private double positionControl() {
        double error = targetDeg - angleDeg;
        double power = TurretConstants.KV * targetRateDegS
                + TurretConstants.KP * error
                + TurretConstants.KD * (targetRateDegS - rateDegS);
        if (Math.abs(error) > TurretConstants.DEADBAND_DEG) {
            power += Math.copySign(TurretConstants.KS, error);
        }
        return Range.clip(power, -TurretConstants.MAX_POWER, TurretConstants.MAX_POWER);
    }

    /** Nunca empuja más allá de un límite; cerca de él, empuja más suave. */
    private double applyLimits(double power) {
        if (power > 0) {
            double room = highLimit() - angleDeg;
            if (room <= 0) return 0;
            if (room < TurretConstants.SLOW_ZONE_DEG) power *= Math.max(0.2, room / TurretConstants.SLOW_ZONE_DEG);
        } else if (power < 0) {
            double room = angleDeg - lowLimit();
            if (room <= 0) return 0;
            if (room < TurretConstants.SLOW_ZONE_DEG) power *= Math.max(0.2, room / TurretConstants.SLOW_ZONE_DEG);
        }
        return power;
    }

    private double applyFaultChecks(double power) {
        if (wrongDirection) return 0;

        // Dirección: con potencia en un sentido, el ángulo no debe ir claramente al otro
        int sign = Math.abs(power) < 0.05 ? 0 : (int) Math.signum(power);
        if (sign != segmentSign) {
            segmentSign = sign;
            segmentStartDeg = angleDeg;
        }
        if (sign != 0 && sign * (angleDeg - segmentStartDeg) < -TurretConstants.WRONG_DIRECTION_DEG) {
            wrongDirection = true;
            return 0;
        }

        // Atorada: mucha potencia y no se mueve
        if (stalled) {
            if (cooldownTimer.seconds() < TurretConstants.STALL_COOLDOWN_S) return 0;
            stalled = false;
        }
        if (Math.abs(power) >= TurretConstants.STALL_POWER) {
            if (Math.abs(angleDeg - stallReferenceDeg) > TurretConstants.STALL_DEG) {
                stallReferenceDeg = angleDeg;
                stallTimer.reset();
            } else if (stallTimer.seconds() > TurretConstants.STALL_TIME_S) {
                stalled = true;
                cooldownTimer.reset();
                return 0;
            }
        } else {
            stallReferenceDeg = angleDeg;
            stallTimer.reset();
        }
        return power;
    }

    public double angleDeg() { return angleDeg; }
    public double rateDegS() { return rateDegS; }
    public double targetDeg() { return targetDeg; }
    public Mode mode() { return mode; }
    public double appliedPower() { return appliedPower; }

    /** En el objetivo: cerca del ángulo pedido y casi a la misma velocidad. */
    public boolean isOnTarget() {
        return available && mode == Mode.POSITION && !hasFault()
                && Math.abs(targetDeg - angleDeg) < TurretConstants.ON_TARGET_DEG
                && Math.abs(rateDegS - targetRateDegS) < TurretConstants.ON_TARGET_RATE_DEG_S;
    }

    public boolean hasFault() { return wrongDirection || stalled; }
    public boolean isWrongDirection() { return wrongDirection; }
    public boolean isStalled() { return stalled; }
    public boolean isEncoderMismatch() { return mismatch; }

    /** Diferencia entre encoders (grados), o NaN si falta uno. */
    public double encoderDifference() {
        if (encoderLeft == null || encoderRight == null) return Double.NaN;
        return encoderRight.turretDeg() - encoderLeft.turretDeg();
    }

    public boolean hasBothEncoders() {
        return encoderLeft != null && encoderRight != null;
    }

    public boolean hasBothServos() {
        return left != null && right != null;
    }

    /** Voltaje del encoder principal (para detectar cable suelto). */
    public double primaryVoltage() {
        return primary != null ? primary.voltage() : 0;
    }

    @Override
    public void stop() {
        disable();
        periodic();
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (!available) {
            telemetry.addData("Torreta", "FALTA HARDWARE (servo o encoder)");
            return;
        }
        String state = wrongDirection ? "FALLA: dirección" : stalled ? "ATORADA" : mode.toString();
        telemetry.addData("Torreta", "%s  %.1f° → %.1f°  %.0f°/s  pot %.2f",
                state, angleDeg, targetDeg, rateDegS, appliedPower);
    }
}
