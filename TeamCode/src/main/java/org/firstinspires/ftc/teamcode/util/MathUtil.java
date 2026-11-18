package org.firstinspires.ftc.teamcode.util;

/** Funciones de matemáticas que se usan en varios lados. */
public final class MathUtil {
    private MathUtil() {}

    /** Lleva un ángulo en grados al rango [-180, 180). */
    public static double wrap180(double degrees) {
        double d = (degrees + 180.0) % 360.0;
        if (d < 0) d += 360.0;
        return d - 180.0;
    }

    /** Lleva un ángulo en radianes al rango [-pi, pi). */
    public static double wrapPi(double radians) {
        return Math.toRadians(wrap180(Math.toDegrees(radians)));
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public static double lerp(double from, double to, double t) {
        return from + (to - from) * t;
    }

    /** Stick con zona muerta y curva: suelto = 0; conserva el signo. */
    public static double shapeStick(double value, double deadband, double exponent) {
        double magnitude = Math.abs(value);
        if (magnitude < deadband) return 0;
        double scaled = (magnitude - deadband) / (1.0 - deadband);
        return Math.copySign(Math.pow(scaled, exponent), value);
    }

    /**
     * Interpolación lineal en una tabla. Fuera de la tabla devuelve el extremo más cercano.
     * Devuelve NaN si la tabla está mal (largos distintos, vacía o x no crecientes).
     */
    public static double interpolate(double[] xs, double[] ys, double x) {
        if (!isValidTable(xs, ys)) return Double.NaN;
        if (x <= xs[0]) return ys[0];
        int last = xs.length - 1;
        if (x >= xs[last]) return ys[last];
        for (int i = 1; i <= last; i++) {
            if (x <= xs[i]) {
                double t = (x - xs[i - 1]) / (xs[i] - xs[i - 1]);
                return lerp(ys[i - 1], ys[i], t);
            }
        }
        return ys[last];
    }

    public static boolean isValidTable(double[] xs, double[] ys) {
        if (xs == null || ys == null || xs.length == 0 || xs.length != ys.length) return false;
        for (int i = 1; i < xs.length; i++) {
            if (xs[i] <= xs[i - 1]) return false;
        }
        return true;
    }

    /** Filtro pasa-bajos: alpha 1 = sin filtro. */
    public static double lowPass(double previous, double current, double alpha) {
        return previous + alpha * (current - previous);
    }
}
