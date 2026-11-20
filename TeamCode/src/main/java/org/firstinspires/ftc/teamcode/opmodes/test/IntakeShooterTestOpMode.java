package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.BatteryMonitor;
import org.firstinspires.ftc.teamcode.subsystems.HardwareCheck;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.MathUtil;

import java.util.List;

/*
 * Intakes, tope y shooter solos: sin torreta, sin Limelight y sin chasis. Para revisar RPM.
 *
 * gamepad1 (control REV: ✕ = A, ○ = B, □ = X, △ = Y)
 *   R1                      intakes meter: encender / apagar
 *   L1 (mantener)           intakes en reversa
 *   △                       shooter: encender / apagar
 *   ✕                       qué motor se ajusta: AMBOS → izquierdo → derecho
 *   D-pad arriba / abajo    RPM ±100
 *   D-pad derecha / izq.    RPM ±25
 *   R2 (mantener)           disparar: el tope abre solo cuando las RPM están listas
 *   ○ (mantener)            abrir el tope SIN esperar RPM (para revisar el tope o disparar igual)
 *   □                       anotar las RPM en la lista de abajo
 *   Options                 MEDIR el shooter: lo pone a dos potencias fijas (~5 s), calcula KV y
 *                           KS, y los aplica al momento. Con eso debe llegar a sus RPM.
 *
 * Después de cada tiro muestra cuánto bajaron las RPM y cuánto tardaron en recuperarse.
 * Los valores de control (KV, KS, KP, tolerancia) están en ShooterConstants.
 */
@TeleOp(name = "Test 7: Intake y tiro", group = "Test")
public class IntakeShooterTestOpMode extends LinearOpMode {

    /** Log del Driver Hub (la telemetría combinada con Panels no tiene log()). */
    org.firstinspires.ftc.robotcore.external.Telemetry.Log dsLog;

    static final double COARSE_STEP = 100;
    static final double FINE_STEP = 25;

    enum Selection { BOTH, LEFT, RIGHT }

    @Override
    public void runOpMode() {
        dsLog = telemetry.log();   // la telemetría combinada con Panels no tiene log()
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        HardwareCheck hardware = new HardwareCheck(hardwareMap);
        BatteryMonitor battery = new BatteryMonitor(hardwareMap);
        Intake intake = new Intake(hardware);
        Indexer indexer = new Indexer(hardware);
        Shooter shooter = new Shooter(hardware, battery);

        while (opModeInInit()) {
            telemetry.addLine("Intakes, tope y shooter. Sin torreta ni Limelight.");
            if (hardware.missing().isEmpty()) {
                telemetry.addLine("Todo el hardware encontrado.");
            } else {
                for (String missing : hardware.missing()) telemetry.addLine("FALTA: " + missing);
            }
            telemetry.addData("Batería", "%.1f V", battery.voltage());
            telemetry.update();
        }
        if (isStopRequested()) return;

        boolean intakeOn = false;
        boolean shooterOn = false;
        Selection selection = Selection.BOTH;
        double rpmLeft = ShooterConstants.PREPARE_RPM;
        double rpmRight = ShooterConstants.PREPARE_RPM;

        // Medición de cada tiro: caída de RPM y tiempo de recuperación
        boolean measuring = false;
        double minLeft = 0, minRight = 0;
        double lastDropLeft = 0, lastDropRight = 0, lastRecoveryS = 0;
        int shots = 0;
        ElapsedTime shotTimer = new ElapsedTime();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            battery.update();
            shooter.readSensors();

            // ---------- Botones ----------
            if (gamepad1.rightBumperWasPressed()) intakeOn = !intakeOn;
            if (gamepad1.yWasPressed()) shooterOn = !shooterOn;
            if (gamepad1.aWasPressed()) selection = Selection.values()[(selection.ordinal() + 1) % Selection.values().length];
            double step = 0;
            if (gamepad1.dpadUpWasPressed()) step += COARSE_STEP;
            if (gamepad1.dpadDownWasPressed()) step -= COARSE_STEP;
            if (gamepad1.dpadRightWasPressed()) step += FINE_STEP;
            if (gamepad1.dpadLeftWasPressed()) step -= FINE_STEP;
            if (selection != Selection.RIGHT) rpmLeft = MathUtil.clamp(rpmLeft + step, 0, ShooterConstants.MAX_RPM);
            if (selection != Selection.LEFT) rpmRight = MathUtil.clamp(rpmRight + step, 0, ShooterConstants.MAX_RPM);
            if (gamepad1.xWasPressed()) {
                dsLog.add("objetivo izq %.0f der %.0f  |  medido izq %.0f der %.0f RPM",
                        rpmLeft, rpmRight, shooter.rpmLeft(), shooter.rpmRight());
            }
            if (gamepad1.startWasPressed()) {
                characterize(hubs, battery, shooter, indexer);
                shooterOn = false;
            }

            // ---------- Intakes ----------
            if (gamepad1.left_bumper) {
                intake.set(Intake.Mode.OUT);
            } else {
                intake.set(intakeOn ? Intake.Mode.IN : Intake.Mode.OFF);
            }
            intake.updateCurrent();

            // ---------- Shooter y tope ----------
            if (shooterOn) shooter.setTargets(rpmLeft, rpmRight); else shooter.off();
            boolean fire = gamepad1.right_trigger > 0.5;
            boolean forceOpen = gamepad1.b;   // abre sin esperar RPM (solo para revisar)
            indexer.setOpen((fire && shooter.isReady()) || forceOpen);

            // Un tiro empieza cuando el tope abre y termina cuando el shooter vuelve a estar listo
            if (indexer.isOpen() && !measuring) {
                measuring = true;
                minLeft = shooter.rpmLeft();
                minRight = shooter.rpmRight();
                shotTimer.reset();
            }
            if (measuring) {
                minLeft = Math.min(minLeft, shooter.rpmLeft());
                minRight = Math.min(minRight, shooter.rpmRight());
                if (!indexer.isOpen() && shooter.isReady()) {
                    measuring = false;
                    shots++;
                    lastDropLeft = shooter.targetLeft() - minLeft;
                    lastDropRight = shooter.targetRight() - minRight;
                    lastRecoveryS = shotTimer.seconds();
                }
            }

            intake.periodic();
            shooter.periodic();
            indexer.periodic();

            // ---------- Pantalla ----------
            telemetry.addData("Shooter", !shooterOn ? "APAGADO (△ para encender)"
                    : shooter.isReady() ? "LISTO" : "subiendo");
            telemetry.addData("Ajustando", selection == Selection.BOTH ? "AMBOS" : selection == Selection.LEFT ? "izquierdo" : "derecho");
            telemetry.addData("Izquierdo", "objetivo %.0f  medido %.0f RPM  potencia %.2f",
                    rpmLeft, shooter.rpmLeft(), shooter.powerLeft());
            telemetry.addData("Derecho", "objetivo %.0f  medido %.0f RPM  potencia %.2f",
                    rpmRight, shooter.rpmRight(), shooter.powerRight());
            if (shooterOn) {
                telemetry.addData("Diagnóstico izq", diagnose(shooter.powerLeft(), shooter.rpmLeft(), rpmLeft));
                telemetry.addData("Diagnóstico der", diagnose(shooter.powerRight(), shooter.rpmRight(), rpmRight));
            }
            telemetry.addData("Tope", indexer.isOpen() ? "ABIERTO" : "cerrado");
            telemetry.addData("Último tiro", shots == 0 ? "ninguno todavía"
                    : String.format("caída izq %.0f  der %.0f RPM  |  recuperó en %.2f s  (%d tiros)",
                    lastDropLeft, lastDropRight, lastRecoveryS, shots));
            intake.addTelemetry(telemetry);
            telemetry.addData("Batería", "%.1f V", battery.voltage());
            telemetry.addData("Control izq", "KV %.6f  KS %.3f  KP %.4f  KI %.4f  (Options: medir)",
                    ShooterConstants.KV_LEFT, ShooterConstants.KS_LEFT, ShooterConstants.KP, ShooterConstants.KI);
            telemetry.addData("Control der", "KV %.6f  KS %.3f", ShooterConstants.KV_RIGHT, ShooterConstants.KS_RIGHT);
            if (!characterization.isEmpty()) telemetry.addData("Medición", characterization);
            for (String missing : hardware.missing()) telemetry.addLine("FALTA: " + missing);
            telemetry.update();
        }
        intake.stop();
        shooter.stop();
        indexer.stop();
    }

    // ===================== Medición automática de KV y KS (Options) =====================

    static final double[] CHAR_POWERS = {0.35, 0.70};
    static final double CHAR_SPINUP_S = 2.0;
    static final double CHAR_MEASURE_S = 0.6;

    /** Último resultado de la medición, para mostrarlo en pantalla. */
    String characterization = "";

    /**
     * Pone los dos motores a dos potencias fijas, mide las RPM estables y calcula cuánta potencia
     * necesita cada RPM (KV) y para arrancar (KS), ya compensado a 12 V. Aplica el promedio en vivo
     * a ShooterConstants para probar de inmediato; los valores se copian al código después.
     */
    void characterize(List<LynxModule> hubs, BatteryMonitor battery, Shooter shooter, Indexer indexer) {
        indexer.setOpen(false);
        double[][] rpm = new double[CHAR_POWERS.length][2];
        double voltSum = 0;
        int voltCount = 0;
        for (int i = 0; i < CHAR_POWERS.length && opModeIsActive(); i++) {
            shooter.setRawPower(CHAR_POWERS[i]);
            ElapsedTime timer = new ElapsedTime();
            double sumLeft = 0, sumRight = 0;
            int samples = 0;
            while (opModeIsActive() && timer.seconds() < CHAR_SPINUP_S + CHAR_MEASURE_S) {
                for (LynxModule hub : hubs) hub.clearBulkCache();
                battery.update();
                shooter.readSensors();
                shooter.periodic();
                indexer.periodic();
                if (timer.seconds() > CHAR_SPINUP_S) {
                    sumLeft += shooter.rpmLeft();
                    sumRight += shooter.rpmRight();
                    samples++;
                    voltSum += battery.voltage();
                    voltCount++;
                }
                telemetry.addLine(String.format("MIDIENDO SHOOTER: potencia %.2f (%d de %d). Manos lejos.",
                        CHAR_POWERS[i], i + 1, CHAR_POWERS.length));
                telemetry.addData("RPM", "izq %.0f   der %.0f", shooter.rpmLeft(), shooter.rpmRight());
                telemetry.update();
            }
            rpm[i][0] = samples > 0 ? sumLeft / samples : 0;
            rpm[i][1] = samples > 0 ? sumRight / samples : 0;
        }
        shooter.setRawPower(Double.NaN);
        shooter.off();
        shooter.periodic();
        if (!opModeIsActive()) return;

        double volts = voltCount > 0 ? voltSum / voltCount : 12.0;
        double toNominal = volts / ShooterConstants.NOMINAL_VOLTAGE;
        double p1 = CHAR_POWERS[0], p2 = CHAR_POWERS[CHAR_POWERS.length - 1];
        StringBuilder text = new StringBuilder();
        String[] names = {"izq", "der"};
        for (int m = 0; m < 2; m++) {
            double r1 = rpm[0][m], r2 = rpm[CHAR_POWERS.length - 1][m];
            if (r2 < 100) {
                text.append(String.format("%s: NO GIRA o el encoder no cuenta (%.0f RPM con potencia %.2f).  ", names[m], r2, p2));
                continue;
            }
            if (r2 - r1 < 50) {
                text.append(String.format("%s: las RPM casi no cambian (%.0f → %.0f): ¿algo lo frena?  ", names[m], r1, r2));
                continue;
            }
            double kvRaw = (p2 - p1) / (r2 - r1);
            double ksRaw = p1 - kvRaw * r1;
            double maxRpm = (1.0 - ksRaw) / kvRaw;
            double kv = kvRaw * toNominal;
            double ks = Math.max(0, ksRaw * toNominal);
            text.append(String.format("%s: %.0f / %.0f RPM, KV %.6f, KS %.3f, máx ~%.0f RPM (APLICADO).  ",
                    names[m], r1, r2, kv, ks, maxRpm));
            if (m == 0) {
                ShooterConstants.KV_LEFT = kv;
                ShooterConstants.KS_LEFT = ks;
            } else {
                ShooterConstants.KV_RIGHT = kv;
                ShooterConstants.KS_RIGHT = ks;
            }
        }
        text.append(String.format("Batería %.1f V.", volts));
        characterization = text.toString();
        dsLog.add("Medición: %s", characterization);
    }

    /** Qué le pasa a un motor del shooter, según potencia y RPM. */
    static String diagnose(double power, double rpm, double target) {
        if (rpm < -100) return "las RPM salen NEGATIVAS: encoder al revés o motor girando al revés";
        if (power > 0.3 && rpm < 100) return "hay potencia pero no hay RPM: cable de encoder, cable de poder o motor";
        if (power >= 0.99 && rpm < target - ShooterConstants.READY_TOLERANCE_RPM) {
            return String.format("a potencia máxima solo llega a %.0f: pide menos RPM o carga la batería", rpm);
        }
        double error = target - rpm;
        if (Math.abs(error) < ShooterConstants.READY_TOLERANCE_RPM) return "en su objetivo";
        return String.format("le faltan %.0f RPM (corrigiendo)", error);
    }
}
