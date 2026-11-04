package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShootSequence;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/*
 * TeleOp de Froggy (LamBot 23758) para dos pilotos. La configuración de hardware está
 * documentada en cada subsistema.
 *
 * Piloto 1 (gamepad1) - chasis
 *   Stick izquierdo: moverse (relativo al campo)    Stick derecho X: girar
 *   Gatillo derecho: modo lento
 *   Back + Y (mantener 1.5 s): el frente actual pasa a ser "adelante"
 *
 * Piloto 2 (gamepad2) - mecanismos
 *   Gatillo derecho (mantener): recoger             Gatillo izquierdo (mantener): sacar pelotas
 *   A: entrar/salir del modo disparo                B: salir del modo disparo
 *   Bumper derecho (mantener): disparar
 *   D-pad arriba/abajo: subir/bajar RPM de shooterBack
 *   D-pad derecha/izquierda: subir/bajar RPM de shooterFront
 *
 * Los flywheels giran hacia atrás todo el tiempo. Al entrar al modo disparo, primero todo gira hacia
 * atrás para alejar las pelotas; después los flywheels aceleran, y con el bumper los intakes solo
 * empujan cuando ambos llegaron a su RPM objetivo. Ver ShootSequence.
 */
@TeleOp(name = "Froggy TeleOp", group = "Froggy")
public class FroggyTeleOp extends LinearOpMode {

    static final double SLOW_MODE_SCALE = 0.35;      // velocidad del chasis en modo lento
    static final double STICK_DEADBAND = 0.05;       // ignora lo poco que marcan los sticks sueltos
    static final double TRIGGER_THRESHOLD = 0.1;     // desde cuánto cuenta un gatillo como presionado
    static final double SHOOTER_RPM_STEP = 100;      // cuánto cambia el RPM con cada toque del D-pad
    static final double HEADING_RESET_HOLD_SECONDS = 1.5;  // evita reiniciar el heading por accidente
    static final double TELEMETRY_PERIOD_SECONDS = 0.15;   // no mandar telemetría en cada ciclo

    // Subsistemas
    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;
    ShootSequence shootSequence;

    final ElapsedTime telemetryTimer = new ElapsedTime();
    final ElapsedTime headingResetTimer = new ElapsedTime();
    boolean headingResetDone = false;

    // Estado anterior de los botones, para detectar cuándo se acaban de presionar
    boolean lastA, lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight, lastHeadingResetHeld;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shootSequence = new ShootSequence(shooter, intake);

        telemetry.addLine("Listo. Deja el robot quieto mientras calibra el giroscopio.");
        telemetry.update();
        waitForStart();

        telemetryTimer.reset();
        while (opModeIsActive()) {
            // Leer sensores primero, para que los controles usen datos de este ciclo
            drivetrain.update();
            driverControls();
            mechanismControls();
            shooter.update();

            if (telemetryTimer.seconds() >= TELEMETRY_PERIOD_SECONDS) {
                updateTelemetry();
                telemetryTimer.reset();
            }
        }
    }

    /** Piloto 1: chasis field-centric. */
    void driverControls() {
        handleHeadingReset();

        double scale = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
        drivetrain.driveFieldCentric(deadband(-gamepad1.left_stick_y), deadband(gamepad1.left_stick_x),
                                     deadband(gamepad1.right_stick_x), scale);
    }

    /** Back + Y mantenidos: reinicia el heading una sola vez y avisa con vibración. */
    void handleHeadingReset() {
        boolean held = gamepad1.back && gamepad1.y;
        if (held && !lastHeadingResetHeld) {
            headingResetTimer.reset();
            headingResetDone = false;
        }
        if (held && !headingResetDone && headingResetTimer.seconds() >= HEADING_RESET_HOLD_SECONDS) {
            drivetrain.resetHeading();
            gamepad1.rumble(500);
            headingResetDone = true;
        }
        lastHeadingResetHeld = held;
    }

    /** Piloto 2: intake y shooter. */
    void mechanismControls() {
        // A prende o apaga el modo disparo; B solo lo apaga
        if (gamepad2.a && !lastA) {
            if (shootSequence.isActive()) {
                shootSequence.cancel();
            } else {
                shootSequence.start();
            }
        }
        if (gamepad2.b) shootSequence.cancel();

        // Ajustar el RPM de cada flywheel con un toque por paso
        if (gamepad2.dpad_up && !lastDpadUp) shooter.adjustBackTargetRpm(SHOOTER_RPM_STEP);
        if (gamepad2.dpad_down && !lastDpadDown) shooter.adjustBackTargetRpm(-SHOOTER_RPM_STEP);
        if (gamepad2.dpad_right && !lastDpadRight) shooter.adjustFrontTargetRpm(SHOOTER_RPM_STEP);
        if (gamepad2.dpad_left && !lastDpadLeft) shooter.adjustFrontTargetRpm(-SHOOTER_RPM_STEP);

        lastA = gamepad2.a;
        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;
        lastDpadLeft = gamepad2.dpad_left;
        lastDpadRight = gamepad2.dpad_right;

        shootSequence.update(gamepad2.right_bumper);
        if (shootSequence.isActive()) return;  // en modo disparo, la secuencia controla el intake

        // Fuera del modo disparo, sacar pelotas tiene prioridad sobre recoger
        if (gamepad2.left_trigger > TRIGGER_THRESHOLD) {
            intake.reverse();
        } else if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
            intake.collect();
        } else {
            intake.stop();
        }
    }

    void updateTelemetry() {
        drivetrain.addTelemetry(telemetry);
        shootSequence.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);

        // Cuenta regresiva mientras se mantiene el reinicio del heading
        if (lastHeadingResetHeld) {
            telemetry.addLine(headingResetDone ? "Heading reiniciado"
                    : String.format("Reinicio de heading en %.1f s",
                            HEADING_RESET_HOLD_SECONDS - headingResetTimer.seconds()));
        }

        telemetry.addLine("GP2 A: modo disparo | B: cancelar | RB: disparar");
        telemetry.update();
    }

    static double deadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0 : value;
    }
}
