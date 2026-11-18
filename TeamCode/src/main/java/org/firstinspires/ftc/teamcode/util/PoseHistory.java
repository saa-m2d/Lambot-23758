package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

/**
 * Las últimas posiciones del robot y ángulos de la torreta, con su hora. Sirve para saber dónde
 * estaba el robot cuando la cámara tomó una foto (la foto llega unos 20-40 ms tarde).
 */
public class PoseHistory {
    private static final int SIZE = 64;

    private final long[] times = new long[SIZE];
    private final double[] xs = new double[SIZE];
    private final double[] ys = new double[SIZE];
    private final double[] headings = new double[SIZE];
    private final double[] turretDegs = new double[SIZE];
    private int next = 0;
    private int count = 0;

    /** Registro de un instante. */
    public static class Sample {
        public final Pose pose;
        public final double turretDeg;

        Sample(Pose pose, double turretDeg) {
            this.pose = pose;
            this.turretDeg = turretDeg;
        }
    }

    public void add(long nanos, Pose pose, double turretDeg) {
        times[next] = nanos;
        xs[next] = pose.x();
        ys[next] = pose.y();
        headings[next] = pose.heading();
        turretDegs[next] = turretDeg;
        next = (next + 1) % SIZE;
        count = Math.min(count + 1, SIZE);
    }

    public void clear() {
        count = 0;
        next = 0;
    }

    /**
     * El estado en el instante pedido, interpolado entre los dos registros más cercanos.
     * Si es más viejo que todo el historial devuelve el más viejo; null si está vacío.
     */
    public Sample at(long nanos) {
        if (count == 0) return null;
        int newest = (next - 1 + SIZE) % SIZE;
        if (nanos >= times[newest]) return sample(newest);
        for (int i = 1; i < count; i++) {
            int older = (newest - i + SIZE) % SIZE;
            int newer = (older + 1) % SIZE;
            if (nanos >= times[older]) {
                double span = times[newer] - times[older];
                double t = span <= 0 ? 0 : (nanos - times[older]) / span;
                double heading = headings[older] + t * MathUtil.wrapPi(headings[newer] - headings[older]);
                Pose pose = new Pose(
                        MathUtil.lerp(xs[older], xs[newer], t),
                        MathUtil.lerp(ys[older], ys[newer], t),
                        heading);
                return new Sample(pose, MathUtil.lerp(turretDegs[older], turretDegs[newer], t));
            }
        }
        return sample((next - count + SIZE) % SIZE);
    }

    private Sample sample(int index) {
        return new Sample(new Pose(xs[index], ys[index], headings[index]), turretDegs[index]);
    }
}
