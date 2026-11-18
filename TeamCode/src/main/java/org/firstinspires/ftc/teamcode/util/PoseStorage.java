package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Lo que el autónomo le deja al TeleOp: dónde quedó el robot, el ángulo de la torreta, la alianza
 * y hacia dónde mira el piloto. Vive en memoria mientras la app del robot no se reinicie.
 */
public final class PoseStorage {
    private PoseStorage() {}

    /** Después de esto, lo guardado ya no se usa (no viene del autónomo de este match). */
    private static final double MAX_AGE_S = 60;

    private static Pose pose;
    private static double turretDeg;
    private static Alliance alliance;
    /** Heading del robot (radianes) que el piloto ve como "hacia adelante". */
    private static double driverForwardHeading;
    private static long savedAtNanos;

    public static void save(Pose robotPose, double turretAngleDeg, Alliance savedAlliance, double driverForward) {
        pose = robotPose;
        turretDeg = turretAngleDeg;
        alliance = savedAlliance;
        driverForwardHeading = driverForward;
        savedAtNanos = System.nanoTime();
    }

    /** true si hay datos de un autónomo reciente. */
    public static boolean isFresh() {
        return pose != null && (System.nanoTime() - savedAtNanos) / 1e9 < MAX_AGE_S;
    }

    public static Pose pose() { return pose; }
    public static double turretDeg() { return turretDeg; }
    public static Alliance alliance() { return alliance; }
    public static double driverForwardHeading() { return driverForwardHeading; }

    public static void clear() {
        pose = null;
    }
}
