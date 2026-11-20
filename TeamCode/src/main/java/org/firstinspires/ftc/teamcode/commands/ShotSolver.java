package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.config.TurretConstants;
import org.firstinspires.ftc.teamcode.config.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Calcula el tiro desde donde esté el robot: hacia dónde apuntar la torreta y qué RPM poner en
 * cada motor (interpolando la tabla de tiro).
 *
 * Disparo en movimiento: la pelota sale con la velocidad del robot. Para compensar se apunta a un
 * "objetivo virtual" = objetivo real - velocidad del robot × tiempo de vuelo. Como el tiempo de
 * vuelo depende de la distancia y la distancia del objetivo virtual, se repite 3 veces.
 */
public class ShotSolver {

    public static class Solution {
        public final boolean valid;
        /** Ángulo deseado de la torreta (-180 a 180, sin elegir vuelta todavía). */
        public final double turretDeg;
        /** Qué tan rápido cambia ese ángulo (grados/s), para el feedforward. */
        public final double turretRateDegS;
        public final double distanceIn;
        public final double rpmLeft, rpmRight;
        public final double timeOfFlightS;
        /** true si la distancia está dentro de la tabla (afuera, las RPM son el extremo). */
        public final boolean inRange;

        Solution(boolean valid, double turretDeg, double turretRateDegS, double distanceIn,
                 double rpmLeft, double rpmRight, double timeOfFlightS, boolean inRange) {
            this.valid = valid;
            this.turretDeg = turretDeg;
            this.turretRateDegS = turretRateDegS;
            this.distanceIn = distanceIn;
            this.rpmLeft = rpmLeft;
            this.rpmRight = rpmRight;
            this.timeOfFlightS = timeOfFlightS;
            this.inRange = inRange;
        }

        public static Solution invalid() {
            return new Solution(false, 0, 0, 0, 0, 0, 0, false);
        }
    }

    /** Fuera de la tabla por más de esto, el tiro se marca fuera de rango. */
    private static final double RANGE_MARGIN_IN = 6;

    /** RPM de la tabla para una distancia: {izquierdo, derecho}, o NaN si la tabla está mal. */
    public static double[] rpmFor(double distanceIn) {
        return new double[]{
                MathUtil.interpolate(ShooterConstants.TABLE_DISTANCE_IN, ShooterConstants.TABLE_RPM_LEFT, distanceIn),
                MathUtil.interpolate(ShooterConstants.TABLE_DISTANCE_IN, ShooterConstants.TABLE_RPM_RIGHT, distanceIn)};
    }

    public static boolean tablesValid() {
        return MathUtil.isValidTable(ShooterConstants.TABLE_DISTANCE_IN, ShooterConstants.TABLE_RPM_LEFT)
                && MathUtil.isValidTable(ShooterConstants.TABLE_DISTANCE_IN, ShooterConstants.TABLE_RPM_RIGHT)
                && MathUtil.isValidTable(ShooterConstants.TOF_DISTANCE_IN, ShooterConstants.TOF_SECONDS);
    }

    /**
     * Tiro solo con lo que ve la cámara en esta foto (sin Pinpoint). Sin disparo en movimiento.
     *
     * @param observation        la CELL vista, con distancia
     * @param turretAtCaptureDeg ángulo de la torreta cuando se tomó la foto
     */
    public Solution solveFromObservation(Vision.CellObservation observation, double turretAtCaptureDeg, double rpmTrim) {
        if (observation == null || !observation.hasRange || !tablesValid()) return Solution.invalid();
        double forward = observation.forwardIn, left = observation.leftIn;
        double distance = Math.hypot(forward, left);
        if (distance < 1) return Solution.invalid();

        // Ajuste lateral del punto de apuntado (a la izquierda vista desde el robot)
        double lateral = VisionConstants.AIM_LATERAL_OFFSET_IN;
        double aimForward = forward - left / distance * lateral;
        double aimLeft = left + forward / distance * lateral;

        double turretDeg = MathUtil.wrap180(turretAtCaptureDeg + Math.toDegrees(Math.atan2(aimLeft, aimForward)));
        double rangeIn = Math.hypot(aimForward, aimLeft) + VisionConstants.AIM_DISTANCE_OFFSET_IN;
        double[] rpm = rpmFor(rangeIn);
        double[] table = ShooterConstants.TABLE_DISTANCE_IN;
        boolean inRange = rangeIn >= table[0] - RANGE_MARGIN_IN && rangeIn <= table[table.length - 1] + RANGE_MARGIN_IN;
        return new Solution(true, turretDeg, 0, rangeIn, rpm[0] + rpmTrim, rpm[1] + rpmTrim, 0, inRange);
    }

    /**
     * @param robot       pose del robot (pulgadas, radianes)
     * @param vx,vy       velocidad del robot en el campo (pulg/s)
     * @param omega       velocidad de giro (rad/s)
     * @param target      posición de la CELL, o null
     * @param shootOnMove compensar la velocidad del robot
     * @param rpmTrim     ajuste en vivo del operador (se suma a los dos motores)
     */
    public Solution solve(Pose robot, double vx, double vy, double omega, Vector2D target,
                          boolean shootOnMove, double rpmTrim) {
        if (target == null || robot == null || !tablesValid()) return Solution.invalid();

        double heading = robot.heading();
        double offsetX = VisionConstants.TURRET_AXIS_FORWARD_IN * Math.cos(heading)
                - VisionConstants.TURRET_AXIS_LEFT_IN * Math.sin(heading);
        double offsetY = VisionConstants.TURRET_AXIS_FORWARD_IN * Math.sin(heading)
                + VisionConstants.TURRET_AXIS_LEFT_IN * Math.cos(heading);

        // Velocidad del eje de la torreta (incluye el giro del chasis si la torreta no está al centro)
        double axisVx = vx - omega * offsetY;
        double axisVy = vy + omega * offsetX;

        // Se adelanta todo un poco para compensar lo que tarda la torreta en reaccionar
        double lead = TurretConstants.LEAD_TIME_S;
        double axisX = robot.x() + offsetX + axisVx * lead;
        double axisY = robot.y() + offsetY + axisVy * lead;
        double headingLead = heading + omega * lead;

        double aimX = target.x(), aimY = target.y();
        double distance = Math.hypot(aimX - axisX, aimY - axisY);
        double timeOfFlight = 0;
        if (shootOnMove) {
            for (int i = 0; i < 3; i++) {
                timeOfFlight = MathUtil.interpolate(ShooterConstants.TOF_DISTANCE_IN, ShooterConstants.TOF_SECONDS,
                        distance + VisionConstants.AIM_DISTANCE_OFFSET_IN);
                aimX = target.x() - axisVx * timeOfFlight;
                aimY = target.y() - axisVy * timeOfFlight;
                distance = Math.hypot(aimX - axisX, aimY - axisY);
            }
        }
        if (distance < 1) return Solution.invalid();

        // Ajuste lateral del punto de apuntado (a la izquierda vista desde el robot)
        double dx = aimX - axisX, dy = aimY - axisY;
        double lateral = VisionConstants.AIM_LATERAL_OFFSET_IN;
        double normalX = -dy / distance, normalY = dx / distance;   // perpendicular a la izquierda
        dx += normalX * lateral;
        dy += normalY * lateral;
        double bearing = Math.atan2(dy, dx);
        double rangeIn = Math.hypot(dx, dy) + VisionConstants.AIM_DISTANCE_OFFSET_IN;

        // La torreta en home apunta a la izquierda (+90° respecto al frente del robot)
        double turretDeg = MathUtil.wrap180(Math.toDegrees(bearing - headingLead) - 90);

        // Qué tan rápido gira la línea al objetivo, menos lo que gira el chasis
        double distanceSquared = dx * dx + dy * dy;
        double bearingRate = (dx * -axisVy - dy * -axisVx) / distanceSquared;
        double turretRate = Math.toDegrees(bearingRate - omega);

        double[] rpm = rpmFor(rangeIn);
        double[] table = ShooterConstants.TABLE_DISTANCE_IN;
        boolean inRange = rangeIn >= table[0] - RANGE_MARGIN_IN && rangeIn <= table[table.length - 1] + RANGE_MARGIN_IN;
        return new Solution(true, turretDeg, turretRate, rangeIn,
                rpm[0] + rpmTrim, rpm[1] + rpmTrim, timeOfFlight, inRange);
    }
}
