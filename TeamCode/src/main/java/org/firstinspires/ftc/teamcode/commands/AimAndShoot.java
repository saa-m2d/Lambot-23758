package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * Coordina torreta, shooter y tope. Lo usan el TeleOp y el autónomo.
 *
 * Reglas:
 *   - El tope solo abre si se pide disparar, el shooter está en sus RPM y (en automático) la
 *     torreta está en el objetivo y la distancia está dentro de la tabla.
 *   - Si no hay objetivo calculado, se cae a modo manual: la torreta se mueve con el stick y las
 *     RPM salen del preset de distancia elegido por el operador.
 */
public class AimAndShoot {
    public enum Status {
        IDLE("shooter apagado"),
        NO_TARGET("sin objetivo: torreta manual y RPM del preset"),
        TRACKING_ANGLE("viendo la CELL sin distancia: torreta sigue, RPM del preset"),
        AIMING("apuntando"),
        SPINNING_UP("subiendo RPM"),
        OUT_OF_RANGE("fuera del rango de la tabla"),
        READY("LISTO"),
        FIRING("DISPARANDO");

        public final String label;
        Status(String label) { this.label = label; }
    }

    private final Turret turret;
    private final Shooter shooter;
    private final Indexer indexer;

    private boolean autoAim = true;
    private boolean shootOnMove = true;
    private double rpmTrim;
    private int presetIndex = 1;
    private boolean holding;
    private double holdDeg;
    private Status status = Status.IDLE;
    private boolean usingSolution;

    public AimAndShoot(Turret turret, Shooter shooter, Indexer indexer) {
        this.turret = turret;
        this.shooter = shooter;
        this.indexer = indexer;
    }

    public void setAutoAim(boolean enabled) { autoAim = enabled; }
    public boolean autoAim() { return autoAim; }
    public void setShootOnMove(boolean enabled) { shootOnMove = enabled; }
    public boolean shootOnMove() { return shootOnMove; }
    public double rpmTrim() { return rpmTrim; }

    public void adjustTrim(int steps) {
        rpmTrim = MathUtil.clamp(rpmTrim + steps * ShooterConstants.RPM_TRIM_STEP,
                -ShooterConstants.RPM_TRIM_MAX, ShooterConstants.RPM_TRIM_MAX);
    }

    public void cyclePreset(int direction) {
        int count = ShooterConstants.MANUAL_PRESET_DISTANCE_IN.length;
        if (count == 0) return;
        presetIndex = ((presetIndex + direction) % count + count) % count;
    }

    public double presetDistance() {
        double[] presets = ShooterConstants.MANUAL_PRESET_DISTANCE_IN;
        if (presets.length == 0) return 0;
        return presets[Math.min(presetIndex, presets.length - 1)];
    }

    public int presetIndex() { return presetIndex; }

    /**
     * Una vez por ciclo.
     *
     * @param solution     tiro calculado (puede ser inválido)
     * @param observation  lo que la cámara ve en este ciclo (para seguir solo con el ángulo), o null
     * @param manualTurret potencia del stick de la torreta (0 = suelto)
     * @param spinUp       mantener el shooter en RPM
     * @param fire         disparar (abre el tope cuando todo está listo)
     */
    public void update(ShotSolver.Solution solution, Vision.CellObservation observation,
                       double manualTurret, boolean spinUp, boolean fire) {
        usingSolution = autoAim && solution.valid;
        boolean angleOnly = autoAim && !usingSolution && observation != null && !observation.hasRange;

        // ---------- Torreta ----------
        if (manualTurret != 0) {
            turret.setManualPower(manualTurret);
            holding = false;
        } else if (usingSolution) {
            turret.setTarget(turret.chooseAngle(solution.turretDeg), solution.turretRateDegS);
            holding = false;
        } else if (angleOnly) {
            turret.setTarget(turret.chooseAngle(turret.angleDeg() + observation.bearingDeg), 0);
            holding = false;
        } else {
            if (!holding) {
                holdDeg = turret.angleDeg();
                holding = true;
            }
            turret.setTarget(holdDeg, 0);
        }

        // ---------- Shooter ----------
        boolean shooterOn = spinUp || fire;
        if (!shooterOn) {
            shooter.off();
        } else if (usingSolution) {
            shooter.setTargets(solution.rpmLeft, solution.rpmRight);
        } else {
            double[] rpm = ShotSolver.rpmFor(presetDistance());
            shooter.setTargets(rpm[0] + rpmTrim, rpm[1] + rpmTrim);
        }

        // ---------- Tope ----------
        boolean aimed = !usingSolution || (turret.isOnTarget() && solution.inRange) || !turret.isAvailable();
        boolean feed = fire && shooter.isReady() && aimed;
        indexer.setOpen(feed);

        // ---------- Estado para el piloto ----------
        if (!shooterOn) {
            status = Status.IDLE;
        } else if (feed) {
            status = Status.FIRING;
        } else if (!usingSolution) {
            status = angleOnly ? Status.TRACKING_ANGLE : Status.NO_TARGET;
        } else if (!solution.inRange) {
            status = Status.OUT_OF_RANGE;
        } else if (!turret.isOnTarget()) {
            status = Status.AIMING;
        } else if (!shooter.isReady()) {
            status = Status.SPINNING_UP;
        } else {
            status = Status.READY;
        }
    }

    public Status status() {
        return status;
    }

    /** true si se está apuntando con el tiro calculado (no con el preset). */
    public boolean isUsingSolution() {
        return usingSolution;
    }

    public void addTelemetry(Telemetry telemetry, ShotSolver.Solution solution) {
        telemetry.addData("Tiro", "%s  |  auto %s  movimiento %s  ajuste %+.0f RPM",
                status.label, autoAim ? "ON" : "OFF", shootOnMove ? "ON" : "OFF", rpmTrim);
        if (usingSolution) {
            telemetry.addData("Solución", "%.0f in  torreta %.1f°  RPM %.0f / %.0f  vuelo %.2f s",
                    solution.distanceIn, solution.turretDeg, solution.rpmLeft, solution.rpmRight, solution.timeOfFlightS);
        } else {
            telemetry.addData("Preset", "%d: %.0f in", presetIndex + 1, presetDistance());
        }
    }
}
