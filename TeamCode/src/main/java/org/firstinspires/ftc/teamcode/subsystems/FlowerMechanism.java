package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.MechanismConstants;
import org.firstinspires.ftc.teamcode.config.RobotMap;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Mecanismo del flower: 2 servos Swyft en espejo que mueven la misma pieza. Cada uno tiene sus
 * propios valores de arriba y abajo (RobotMap.FLOWER_*). Se mueven juntos con un recorrido de
 * 0 (arriba) a 1 (abajo), a velocidad limitada, así nunca pelean.
 */
public class FlowerMechanism implements Subsystem {
    public enum Position { UP, DOWN }

    private final ServoImplEx left, right;
    private Position target = Position.UP;
    private double travel = 0;           // 0 = arriba, 1 = abajo
    private boolean energized;
    private final ElapsedTime dtTimer = new ElapsedTime();

    public FlowerMechanism(HardwareCheck hardware) {
        left = hardware.get(ServoImplEx.class, RobotMap.FLOWER_LEFT, "servo Control Hub 5");
        right = hardware.get(ServoImplEx.class, RobotMap.FLOWER_RIGHT, "servo Expansion Hub 2");
        for (ServoImplEx servo : new ServoImplEx[]{left, right}) {
            if (servo == null) continue;
            servo.setPwmRange(new PwmControl.PwmRange(RobotMap.SERVO_PWM_MIN_US, RobotMap.SERVO_PWM_MAX_US));
            servo.setPwmDisable();   // sueltos hasta el primer comando
        }
    }

    @Override
    public boolean isAvailable() {
        return left != null && right != null;
    }

    /** Ir arriba o abajo. El primer comando energiza los servos en la posición de arriba. */
    public void setTarget(Position position) {
        target = position;
        if (!energized && isAvailable()) {
            travel = 0;
            apply();
            left.setPwmEnable();
            right.setPwmEnable();
            energized = true;
            dtTimer.reset();
        }
    }

    public void toggle() {
        setTarget(target == Position.UP ? Position.DOWN : Position.UP);
    }

    public Position target() {
        return target;
    }

    /** true si ya llegó a donde se le pidió. */
    public boolean isSettled() {
        return travel == (target == Position.UP ? 0 : 1);
    }

    @Override
    public void periodic() {
        double dt = dtTimer.seconds();
        dtTimer.reset();
        if (!energized || !isAvailable()) return;
        double goal = target == Position.UP ? 0 : 1;
        double maxStep = dt / Math.max(0.05, MechanismConstants.FLOWER_MOVE_TIME_S);
        travel += MathUtil.clamp(goal - travel, -maxStep, maxStep);
        apply();
    }

    private void apply() {
        left.setPosition(MathUtil.lerp(RobotMap.FLOWER_LEFT_UP, RobotMap.FLOWER_LEFT_DOWN, travel));
        right.setPosition(MathUtil.lerp(RobotMap.FLOWER_RIGHT_UP, RobotMap.FLOWER_RIGHT_DOWN, travel));
    }

    /** Suelta los dos servos. */
    @Override
    public void stop() {
        if (!isAvailable()) return;
        left.setPwmDisable();
        right.setPwmDisable();
        energized = false;
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Flower", !isAvailable() ? "FALTA HARDWARE"
                : !energized ? "suelto" : String.format("%s (recorrido %.2f)", target, travel));
    }
}
