package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.config.RobotMap;
import org.firstinspires.ftc.teamcode.config.VisionConstants;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

/**
 * Limelight 3A sobre la torreta. Encuentra la CELL de la alianza que está arriba y dice dónde
 * queda respecto al eje de la torreta.
 *
 * Marco de la torreta: "adelante" = hacia donde apunta la torreta, "izquierda" y "arriba".
 */
public class Vision implements Subsystem {

    /** Lo que la cámara ve de una CELL en una foto. */
    public static class CellObservation {
        public final Alliance.Cell cell;
        /** Posición del centro de los tags respecto al eje de la torreta (pulgadas). */
        public final double forwardIn, leftIn, upIn;
        /** false si la Limelight no dio distancia (solo ángulo). */
        public final boolean hasRange;
        /** Ángulo horizontal al objetivo visto desde la cámara (grados, positivo = izquierda). */
        public final double bearingDeg;
        public final int tagCount;
        /** Hora (System.nanoTime) en que se tomó la foto. */
        public final long captureNanos;

        CellObservation(Alliance.Cell cell, double forwardIn, double leftIn, double upIn, boolean hasRange,
                        double bearingDeg, int tagCount, long captureNanos) {
            this.cell = cell;
            this.forwardIn = forwardIn;
            this.leftIn = leftIn;
            this.upIn = upIn;
            this.hasRange = hasRange;
            this.bearingDeg = bearingDeg;
            this.tagCount = tagCount;
            this.captureNanos = captureNanos;
        }

        public double horizontalDistanceIn() {
            return Math.hypot(forwardIn, leftIn);
        }
    }

    private final Limelight3A limelight;
    private boolean started;
    private LLResult result;
    private boolean fresh;
    private long resultCaptureNanos;

    public Vision(HardwareCheck hardware) {
        limelight = hardware.get(Limelight3A.class, RobotMap.LIMELIGHT, "Limelight 3A, USB 3.0 del Control Hub");
    }

    @Override
    public boolean isAvailable() {
        return limelight != null;
    }

    public void start() {
        if (limelight == null || started) return;
        limelight.setPollRateHz(VisionConstants.POLL_RATE_HZ);
        limelight.pipelineSwitch(VisionConstants.PIPELINE);
        limelight.start();
        started = true;
    }

    /**
     * Manda a la Limelight hacia dónde mira la cámara (para MegaTag 2) y toma el último resultado.
     *
     * @param cameraYawDeg heading de la cámara en el marco del campo (heading del robot + 90° +
     *                     ángulo de la torreta)
     */
    public void update(double cameraYawDeg) {
        if (limelight == null) return;
        start();
        limelight.updateRobotOrientation(AngleUnit.normalizeDegrees(cameraYawDeg));
        result = limelight.getLatestResult();
        fresh = result != null && result.isValid() && result.getStaleness() < VisionConstants.STALE_MS;
        if (fresh) {
            double ageMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
            resultCaptureNanos = System.nanoTime() - (long) (ageMs * 1e6);
        }
    }

    /** true si hay un resultado reciente con al menos un objetivo. */
    public boolean hasFreshResult() {
        return fresh;
    }

    public boolean isConnected() {
        return limelight != null && limelight.isConnected();
    }

    /** Milisegundos desde el último dato de la Limelight. */
    public long timeSinceLastUpdateMs() {
        return limelight != null ? limelight.getTimeSinceLastUpdate() : Long.MAX_VALUE;
    }

    /**
     * La CELL de la alianza que se ve, o null. Si se ven las dos, se elige la más alta (la que
     * mira hacia arriba).
     */
    public CellObservation observe(Alliance alliance) {
        if (!fresh) return null;
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags == null || tags.isEmpty()) return null;

        CellObservation best = null;
        for (Alliance.Cell cell : Alliance.Cell.values()) {
            double sumF = 0, sumL = 0, sumU = 0, sumBearing = 0;
            int count = 0, withRange = 0;
            for (LLResultTypes.FiducialResult tag : tags) {
                if (alliance.cellOf(tag.getFiducialId()) != cell) continue;
                count++;
                sumBearing += -tag.getTargetXDegrees();   // tx positivo = a la derecha
                double[] turretFrame = cameraToTurret(tag.getTargetPoseCameraSpace());
                if (turretFrame != null) {
                    sumF += turretFrame[0];
                    sumL += turretFrame[1];
                    sumU += turretFrame[2];
                    withRange++;
                }
            }
            if (count == 0) continue;
            boolean hasRange = withRange > 0;
            CellObservation observation = new CellObservation(cell,
                    hasRange ? sumF / withRange : 0,
                    hasRange ? sumL / withRange : 0,
                    hasRange ? sumU / withRange : 0,
                    hasRange, sumBearing / count, count, resultCaptureNanos);
            if (best == null || isHigher(observation, best)) best = observation;
        }
        return best;
    }

    private static boolean isHigher(CellObservation a, CellObservation b) {
        if (a.hasRange && b.hasRange) return a.upIn > b.upIn;
        return a.tagCount > b.tagCount;
    }

    /**
     * Convierte una posición en el marco de la cámara de la Limelight (x derecha, y abajo,
     * z hacia afuera de la lente; metros) al marco de la torreta (adelante, izquierda, arriba;
     * pulgadas, desde el eje). Devuelve null si la Limelight no dio la posición.
     */
    public static double[] cameraToTurret(Pose3D cameraSpace) {
        if (cameraSpace == null) return null;
        Position p = cameraSpace.getPosition().toUnit(DistanceUnit.INCH);
        if (p.x == 0 && p.y == 0 && p.z == 0) return null;
        double pitch = Math.toRadians(VisionConstants.CAMERA_PITCH_DEG);
        double sin = Math.sin(pitch), cos = Math.cos(pitch);
        double forward = p.y * sin + p.z * cos - VisionConstants.CAMERA_BEHIND_AXIS_IN;
        double left = -p.x;
        double up = -p.y * cos + p.z * sin;
        return new double[]{forward, left, up};
    }

    /** Pose de MegaTag 1 (del eje de la torreta, marco del campo), o null. */
    public Pose3D megaTag1() {
        return fresh ? result.getBotpose() : null;
    }

    /** Pose de MegaTag 2 (del eje de la torreta, marco del campo), o null. */
    public Pose3D megaTag2() {
        return fresh ? result.getBotpose_MT2() : null;
    }

    public int megaTagTagCount() {
        return fresh ? result.getBotposeTagCount() : 0;
    }

    public long captureNanos() {
        return resultCaptureNanos;
    }

    @Override
    public void periodic() {
        // La Limelight no tiene salidas: todo pasa en update()
    }

    @Override
    public void stop() {
        if (limelight != null && started) limelight.stop();
        started = false;
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        if (limelight == null) {
            telemetry.addData("Limelight", "FALTA HARDWARE");
            return;
        }
        if (!isConnected()) {
            telemetry.addData("Limelight", "SIN CONEXIÓN");
        } else if (!fresh) {
            telemetry.addData("Limelight", "sin tags a la vista");
        } else {
            telemetry.addData("Limelight", "%d tags  latencia %.0f ms", result.getFiducialResults().size(),
                    result.getCaptureLatency() + result.getTargetingLatency());
        }
    }
}
