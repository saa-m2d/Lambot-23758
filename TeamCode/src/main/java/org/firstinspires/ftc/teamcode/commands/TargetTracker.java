package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.config.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseHistory;

import java.util.EnumMap;
import java.util.Map;

/**
 * Recuerda dónde está cada CELL de la alianza, en las coordenadas del Pinpoint.
 *
 * Cada vez que la cámara ve una CELL, calcula su posición con la del robot y la torreta en el
 * momento de la foto, y la mezcla con lo que ya tenía. La CELL activa (a la que se apunta) es la
 * que se ve arriba; el operador puede fijarla a mano.
 */
public class TargetTracker {
    private static class CellState {
        boolean known;
        double x, y;
        long lastSeenNanos;
    }

    private Alliance alliance;
    private final Map<Alliance.Cell, CellState> cells = new EnumMap<>(Alliance.Cell.class);
    private Alliance.Cell active;
    private Alliance.Cell override;
    private Alliance.Cell pending;
    private long pendingSinceNanos;
    private Vision.CellObservation lastObservation;
    private boolean seenThisLoop;

    public TargetTracker(Alliance alliance) {
        this.alliance = alliance;
        for (Alliance.Cell cell : Alliance.Cell.values()) cells.put(cell, new CellState());
    }

    public void setAlliance(Alliance newAlliance) {
        if (newAlliance != alliance) {
            alliance = newAlliance;
            clear();
        }
    }

    public Alliance alliance() {
        return alliance;
    }

    /** Olvida todo (por ejemplo, si la posición del robot se reinició). */
    public void clear() {
        for (CellState state : cells.values()) state.known = false;
        active = null;
        pending = null;
        lastObservation = null;
    }

    /** Posición aproximada de una CELL antes de verla (el autónomo la conoce del plano). */
    public void setPrior(Alliance.Cell cell, double x, double y) {
        CellState state = cells.get(cell);
        state.known = true;
        state.x = x;
        state.y = y;
        state.lastSeenNanos = 0;
        if (active == null) active = cell;
    }

    /** null = automático (la que se ve arriba). */
    public void setOverride(Alliance.Cell cell) {
        override = cell;
    }

    public Alliance.Cell override() {
        return override;
    }

    /**
     * Procesa lo que vio la cámara en este ciclo.
     *
     * @param turnRateRadS  velocidad de giro del chasis
     * @param turretRateDegS velocidad de la torreta
     */
    public void update(Vision.CellObservation observation, Localization localization,
                       double turnRateRadS, double turretRateDegS) {
        seenThisLoop = false;
        lastObservation = observation;
        if (observation == null) return;
        long now = System.nanoTime();

        // ¿Cambió la CELL de arriba? Solo si se ve la otra un rato seguido
        if (active == null) {
            active = observation.cell;
        } else if (observation.cell != active) {
            if (pending != observation.cell) {
                pending = observation.cell;
                pendingSinceNanos = now;
            } else if ((now - pendingSinceNanos) / 1e9 > VisionConstants.CELL_SWITCH_S) {
                active = observation.cell;
                pending = null;
            }
        } else {
            pending = null;
        }

        if (!observation.hasRange || !localization.isAvailable()) return;
        if (Math.abs(Math.toDegrees(turnRateRadS)) > VisionConstants.MAX_TURN_RATE_FOR_TARGET_DEG_S) return;
        if (Math.abs(turretRateDegS) > VisionConstants.MAX_TURRET_RATE_FOR_TARGET_DEG_S) return;

        PoseHistory.Sample sample = localization.at(observation.captureNanos);
        if (sample == null) return;
        Vector2D point = observationToField(sample.pose, sample.turretDeg, observation);

        CellState state = cells.get(observation.cell);
        boolean reset = !state.known
                || Math.hypot(point.x() - state.x, point.y() - state.y) > VisionConstants.TARGET_RESET_IN;
        double blend = reset ? 1.0 : VisionConstants.TARGET_BLEND;
        state.x += blend * (point.x() - state.x);
        state.y += blend * (point.y() - state.y);
        state.known = true;
        state.lastSeenNanos = now;
        seenThisLoop = observation.cell == targetCell();
    }

    /** Dónde queda en el campo lo que vio la cámara, dado dónde estaban el robot y la torreta. */
    public static Vector2D observationToField(Pose robot, double turretDeg, Vision.CellObservation observation) {
        double heading = robot.heading();
        double axisX = robot.x() + VisionConstants.TURRET_AXIS_FORWARD_IN * Math.cos(heading)
                - VisionConstants.TURRET_AXIS_LEFT_IN * Math.sin(heading);
        double axisY = robot.y() + VisionConstants.TURRET_AXIS_FORWARD_IN * Math.sin(heading)
                + VisionConstants.TURRET_AXIS_LEFT_IN * Math.cos(heading);
        double turretHeading = heading + Math.toRadians(90 + turretDeg);
        double x = axisX + observation.forwardIn * Math.cos(turretHeading) - observation.leftIn * Math.sin(turretHeading);
        double y = axisY + observation.forwardIn * Math.sin(turretHeading) + observation.leftIn * Math.cos(turretHeading);
        return Vector2D.cartesian(x, y);
    }

    /** La CELL a la que se apunta: la del operador si la fijó, si no la que se ve arriba. */
    public Alliance.Cell targetCell() {
        return override != null ? override : active;
    }

    /** Posición de la CELL objetivo, o null si nunca se ha visto. */
    public Vector2D target() {
        Alliance.Cell cell = targetCell();
        if (cell == null) return null;
        CellState state = cells.get(cell);
        return state.known ? Vector2D.cartesian(state.x, state.y) : null;
    }

    /** true si la CELL objetivo se vio hace poco (si no, se apunta solo con odometría). */
    public boolean isFresh() {
        Alliance.Cell cell = targetCell();
        if (cell == null) return false;
        CellState state = cells.get(cell);
        return state.known && state.lastSeenNanos != 0
                && (System.nanoTime() - state.lastSeenNanos) / 1e9 < VisionConstants.TARGET_OLD_S;
    }

    public boolean seenThisLoop() {
        return seenThisLoop;
    }

    /** Lo que se vio en este ciclo (puede no tener distancia), o null. */
    public Vision.CellObservation lastObservation() {
        return lastObservation;
    }
}
