package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PoseHistory;

/**
 * Dónde está el robot, con el Pinpoint. Usa el mismo localizador que PedroPathing, así el
 * autónomo y el TeleOp hablan de las mismas coordenadas: pulgadas, heading en radianes,
 * antihorario positivo.
 *
 * En el autónomo, el Follower de Pedro actualiza el localizador; en el TeleOp lo hace esta clase.
 */
public class Localization {
    /** Qué tanto se suaviza la velocidad calculada (1 = sin suavizar). */
    private static final double VELOCITY_FILTER = 0.4;

    private final Localizer localizer;          // null si falta el Pinpoint
    private final GoBildaPinpointDriver pinpoint;
    private final boolean ownsUpdates;
    private final PoseHistory history = new PoseHistory();

    private Pose pose = Pose.zero();
    private double vx, vy, omega;               // marco del campo: pulg/s y rad/s
    private long lastNanos;
    private boolean hasLast;

    /**
     * @param localizer   el localizador ya creado (el del Follower en el autónomo), o null para
     *                    crear uno propio
     * @param ownsUpdates true si esta clase debe llamar a update() del localizador
     */
    public Localization(HardwareCheck hardware, Localizer localizer, boolean ownsUpdates) {
        pinpoint = hardware.get(GoBildaPinpointDriver.class, Constants.PINPOINT_NAME,
                "Pinpoint, Expansion Hub I2C bus 0");
        if (localizer == null && pinpoint != null) {
            localizer = Constants.createLocalizer(hardware.hardwareMap());
        }
        this.localizer = pinpoint != null ? localizer : null;
        this.ownsUpdates = ownsUpdates;
    }

    public boolean isAvailable() {
        return localizer != null;
    }

    /** Lee el Pinpoint (si le toca a esta clase) y calcula la velocidad. */
    public void update() {
        if (localizer == null) return;
        if (ownsUpdates) localizer.update();
        Pose current = localizer.pose();
        long now = System.nanoTime();
        if (hasLast) {
            double dt = (now - lastNanos) / 1e9;
            if (dt > 1e-4) {
                vx = MathUtil.lowPass(vx, (current.x() - pose.x()) / dt, VELOCITY_FILTER);
                vy = MathUtil.lowPass(vy, (current.y() - pose.y()) / dt, VELOCITY_FILTER);
                omega = MathUtil.lowPass(omega, MathUtil.wrapPi(current.heading() - pose.heading()) / dt, VELOCITY_FILTER);
            }
        }
        pose = current;
        lastNanos = now;
        hasLast = true;
    }

    /** Guarda el estado de este ciclo (con el ángulo de la torreta) para cruzarlo con la cámara. */
    public void record(double turretDeg) {
        history.add(lastNanos, pose, turretDeg);
    }

    public PoseHistory.Sample at(long nanos) {
        return history.at(nanos);
    }

    /** Cambia la posición del robot (al empezar, o al corregir con visión). */
    public void setPose(Pose newPose) {
        pose = newPose;
        if (localizer != null) localizer.setPose(newPose);
        history.clear();
        hasLast = false;
    }

    /** Corrección pequeña de la posición (visión) sin borrar el historial. */
    public void nudge(double dx, double dy) {
        pose = new Pose(pose.x() + dx, pose.y() + dy, pose.heading());
        if (localizer != null) localizer.setPose(pose);
    }

    public Pose pose() { return pose; }
    public double vx() { return vx; }
    public double vy() { return vy; }
    /** Velocidad de giro, rad/s, antihorario positivo. */
    public double omega() { return omega; }

    /** Estado del Pinpoint, o null si falta. */
    public GoBildaPinpointDriver.DeviceStatus pinpointStatus() {
        return pinpoint != null ? pinpoint.getDeviceStatus() : null;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (localizer == null) {
            telemetry.addData("Posición", "SIN PINPOINT: field-centric y apuntado con odometría apagados");
            return;
        }
        telemetry.addData("Posición", "x %.1f  y %.1f in  heading %.1f°  |  %.0f in/s",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()), Math.hypot(vx, vy));
    }
}
