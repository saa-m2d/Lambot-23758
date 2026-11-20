package org.firstinspires.ftc.teamcode.diagnostics;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.ShotSolver;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.util.Debounce;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Revisa el robot en cada ciclo y arma una lista de alertas para el piloto. Las fallas CRÍTICAS
 * nuevas se marcan para que el OpMode haga vibrar los controles.
 *
 * Cada alerta dice qué pasa y qué hacer. Varias desactivan algo solas (por ejemplo, sin Limelight
 * el apuntado sigue con odometría o en manual).
 */
public class ErrorDetectionSystem {
    public enum Severity { INFO, WARNING, CRITICAL }

    public static final class Alert {
        public final Severity severity;
        public final String message;

        Alert(Severity severity, String message) {
            this.severity = severity;
            this.message = message;
        }
    }

    // Umbrales
    private static final double BATTERY_LOW_V = 11.5;
    private static final double BATTERY_CRITICAL_V = 10.5;
    private static final double LOOP_SLOW_MS = 40;
    private static final double MOTOR_OVERCURRENT_A = 9.0;
    private static final double SHOOTER_DEAD_POWER = 0.3;
    private static final double SHOOTER_DEAD_RPM = 100;
    private static final double SLOW_CHECK_PERIOD_S = 0.2;
    private static final double ENCODER_DEAD_V = 0.01;

    private final List<Alert> alerts = new ArrayList<>();
    private final Set<String> previousCritical = new HashSet<>();
    private boolean newCritical;

    private final Debounce batteryLow = new Debounce();
    private final Debounce batteryCritical = new Debounce();
    private final Debounce slowLoop = new Debounce();
    private final Debounce shooterLeftDead = new Debounce();
    private final Debounce shooterRightDead = new Debounce();
    private final Debounce limelightStale = new Debounce();
    private final Debounce turretEncoderDead = new Debounce();
    private final Debounce pinpointStarting = new Debounce();
    private final Debounce[] driveOvercurrent = {new Debounce(), new Debounce(), new Debounce(), new Debounce()};
    private final Debounce[] shooterOvercurrent = {new Debounce(), new Debounce()};

    private final ElapsedTime slowCheckTimer = new ElapsedTime();
    private double[] driveAmps = new double[0];
    private double[] shooterAmps = new double[0];
    private double loopMsFiltered;

    public void update(Robot robot, double loopMs) {
        alerts.clear();
        loopMsFiltered += 0.1 * (loopMs - loopMsFiltered);

        // ---------- Configuración ----------
        for (String missing : robot.hardware.missing()) {
            add(Severity.CRITICAL, "Falta en la configuración: " + missing);
        }
        if (!ShotSolver.tablesValid()) {
            add(Severity.CRITICAL, "Tabla de tiro mal escrita (largos distintos o distancias no crecientes). Apuntado automático apagado.");
        }

        // ---------- Batería y ciclo ----------
        double volts = robot.battery.voltage();
        if (batteryCritical.update(volts < BATTERY_CRITICAL_V, 2)) {
            add(Severity.CRITICAL, String.format("Batería en %.1f V: cámbiala.", volts));
        } else if (batteryLow.update(volts < BATTERY_LOW_V, 2)) {
            add(Severity.WARNING, String.format("Batería baja (%.1f V).", volts));
        }
        if (slowLoop.update(loopMsFiltered > LOOP_SLOW_MS, 1)) {
            add(Severity.WARNING, String.format("Ciclo lento (%.0f ms): el control responde tarde.", loopMsFiltered));
        }

        // ---------- Pinpoint ----------
        GoBildaPinpointDriver.DeviceStatus status = robot.localization.pinpointStatus();
        if (status != null && status != GoBildaPinpointDriver.DeviceStatus.READY) {
            boolean starting = status == GoBildaPinpointDriver.DeviceStatus.CALIBRATING
                    || status == GoBildaPinpointDriver.DeviceStatus.NOT_READY;
            add(starting && pinpointStarting.update(true, 3) ? Severity.CRITICAL : starting ? Severity.WARNING : Severity.CRITICAL,
                    "Pinpoint: " + pinpointMessage(status));
        } else {
            pinpointStarting.update(false, 3);
        }

        // ---------- Limelight ----------
        if (robot.vision.isAvailable()) {
            if (!robot.vision.isConnected()) {
                add(Severity.WARNING, "Limelight sin conexión: se apunta con odometría (si ya vio la CELL) o en manual.");
            } else if (limelightStale.update(robot.vision.timeSinceLastUpdateMs() > 500, 1)) {
                add(Severity.WARNING, "Limelight sin datos nuevos. Revisa el cable USB 3.0.");
            }
        }
        if (robot.tracker.target() != null && !robot.tracker.isFresh()) {
            add(Severity.INFO, "No se ve la CELL: apuntando solo con odometría.");
        }

        // ---------- Torreta ----------
        if (robot.turret.isWrongDirection()) {
            add(Severity.INFO, String.format("Torreta: se movió al revés de lo pedido (%d veces). Sigue funcionando;"
                    + " si pasa seguido, revisa direcciones con Test 5.", robot.turret.wrongDirectionCount()));
        }
        if (robot.turret.isStalled()) {
            add(Severity.INFO, "Torreta: le cuesta avanzar (¿cadena portacables?). Sigue funcionando.");
        }
        if (robot.turret.isEncoderMismatch()) {
            add(Severity.WARNING, String.format("Encoders de la torreta no coinciden (%.1f°): engrane que patina o cable flojo.",
                    robot.turret.encoderDifference()));
        }
        if (robot.turret.isAvailable() && turretEncoderDead.update(robot.turret.primaryVoltage() < ENCODER_DEAD_V, 1)) {
            add(Severity.CRITICAL, "Encoder de la torreta en 0 V: 4.º cable suelto. La torreta no sabe dónde está.");
        }
        if (TurretConfig.LEFT_HOME_VOLTAGE < 0) {
            add(Severity.INFO, "Home de la torreta sin medir: debe arrancar en la marca de home.");
        }

        // ---------- Shooter ----------
        if (robot.shooter.isAvailable()) {
            if (shooterLeftDead.update(robot.shooter.powerLeft() > SHOOTER_DEAD_POWER
                    && robot.shooter.rpmLeft() < SHOOTER_DEAD_RPM, 0.6)) {
                add(Severity.CRITICAL, "shooterLeft: hay potencia pero no hay RPM. Motor, cable de potencia o encoder.");
            }
            if (shooterRightDead.update(robot.shooter.powerRight() > SHOOTER_DEAD_POWER
                    && robot.shooter.rpmRight() < SHOOTER_DEAD_RPM, 0.6)) {
                add(Severity.CRITICAL, "shooterRight: hay potencia pero no hay RPM. Motor, cable de potencia o encoder.");
            }
        }

        // ---------- Corrientes (lectura lenta) ----------
        if (slowCheckTimer.seconds() > SLOW_CHECK_PERIOD_S) {
            slowCheckTimer.reset();
            if (robot.drivetrain != null) driveAmps = robot.drivetrain.readCurrents();
            shooterAmps = robot.shooter.readCurrents();
        }
        String[] driveNames = {"frontLeft", "backLeft", "frontRight", "backRight"};
        for (int i = 0; i < driveAmps.length && i < driveOvercurrent.length; i++) {
            if (driveOvercurrent[i].update(driveAmps[i] > MOTOR_OVERCURRENT_A, 1)) {
                add(Severity.WARNING, String.format("%s con %.1f A sostenidos: algo lo frena.", driveNames[i], driveAmps[i]));
            }
        }
        String[] shooterNames = {"shooterLeft", "shooterRight"};
        for (int i = 0; i < shooterAmps.length && i < shooterOvercurrent.length; i++) {
            if (shooterOvercurrent[i].update(shooterAmps[i] > MOTOR_OVERCURRENT_A, 1)) {
                add(Severity.WARNING, String.format("%s con %.1f A sostenidos: revisa que gire libre.", shooterNames[i], shooterAmps[i]));
            }
        }

        // ---------- ¿Hay fallas críticas nuevas? ----------
        Set<String> currentCritical = new HashSet<>();
        for (Alert alert : alerts) {
            if (alert.severity == Severity.CRITICAL) currentCritical.add(alert.message);
        }
        for (String message : currentCritical) {
            if (!previousCritical.contains(message)) newCritical = true;
        }
        previousCritical.clear();
        previousCritical.addAll(currentCritical);
    }

    /** Qué significa cada estado (LED del Pinpoint según goBILDA) y qué hacer. */
    private static String pinpointMessage(GoBildaPinpointDriver.DeviceStatus status) {
        switch (status) {
            case CALIBRATING:
            case NOT_READY:
                return "arrancando o calibrando (LED rojo). Robot quieto. Si no pasa a verde en unos segundos, "
                        + "se está reiniciando: revisa batería y el cable de poder del Expansion Hub.";
            case FAULT_NO_PODS_DETECTED:
                return "no detecta ningún pod (LED morado). Revisa los dos cables de los pods.";
            case FAULT_X_POD_NOT_DETECTED:
                return "no detecta el pod X (LED azul). Revisa su cable.";
            case FAULT_Y_POD_NOT_DETECTED:
                return "no detecta el pod Y (LED naranja). Revisa su cable.";
            case FAULT_IMU_RUNAWAY:
                return "el giroscopio se desbocó. Recalibra con el robot quieto.";
            case FAULT_BAD_READ:
                return "lectura I2C corrupta. Revisa el cable I2C (Expansion Hub, bus 0).";
            default:
                return status.toString();
        }
    }

    private void add(Severity severity, String message) {
        alerts.add(new Alert(severity, message));
    }

    public List<Alert> alerts() {
        return alerts;
    }

    public boolean hasCritical() {
        for (Alert alert : alerts) {
            if (alert.severity == Severity.CRITICAL) return true;
        }
        return false;
    }

    /** true una vez por cada falla crítica nueva (para vibrar los controles). */
    public boolean consumeNewCritical() {
        boolean value = newCritical;
        newCritical = false;
        return value;
    }

    public void addTelemetry(Telemetry telemetry) {
        if (alerts.isEmpty()) {
            telemetry.addLine("Sin fallas");
            return;
        }
        for (Alert alert : alerts) {
            String prefix = alert.severity == Severity.CRITICAL ? "!! " : alert.severity == Severity.WARNING ? "!  " : "   ";
            telemetry.addLine(prefix + alert.message);
        }
    }
}
