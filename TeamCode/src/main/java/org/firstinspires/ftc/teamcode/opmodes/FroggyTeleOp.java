package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/*
 * TeleOp de Froggy (LamBot 23758) para dos pilotos. La configuración de hardware está
 * documentada en cada subsistema.
 *
 * Piloto 1 (gamepad1) - chasis
 *   Stick izquierdo: moverse (relativo al campo)    Stick derecho X: girar
 *   Gatillo derecho: modo lento                     Options: el frente actual pasa a ser "adelante"
 *
 * Piloto 2 (gamepad2) - mecanismos
 *   Bumper derecho: prender/apagar intake           Bumper izquierdo (mantener): intake en reversa
 *   X: prender/apagar flywheels (pre-calentar)      Gatillo derecho (mantener): disparar
 *   D-pad arriba/abajo: subir/bajar RPM del shooter
 *
 * Al disparar, los intakes solo empujan pelotas cuando ambos flywheels llegaron a su RPM objetivo,
 * para que todos los tiros salgan con la misma velocidad.
 */
@TeleOp(name = "Froggy TeleOp", group = "Froggy")
public class FroggyTeleOp extends LinearOpMode {

    static final double SLOW_MODE_SCALE = 0.35;
    static final double SHOOTER_RPM_STEP = 100;

    Drivetrain drivetrain;
    Intake intake;
    Shooter shooter;

    boolean intakeOn = false;
    boolean flywheelsOn = false;

    // Estado anterior de los botones, para detectar cuándo se acaban de presionar
    boolean lastRightBumper, lastX, lastDpadUp, lastDpadDown;

    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);

        telemetry.addLine("Listo. Deja el robot quieto mientras calibra el giroscopio.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            drivetrain.update();
            driverControls();
            mechanismControls();

            drivetrain.addTelemetry(telemetry);
            shooter.addTelemetry(telemetry);
            intake.addTelemetry(telemetry);
            telemetry.update();
        }
    }

    /** Piloto 1: chasis field-centric. */
    void driverControls() {
        if (gamepad1.options) drivetrain.resetHeading();

        double scale = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;
        drivetrain.driveFieldCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, scale);
    }

    /** Piloto 2: intake y shooter. */
    void mechanismControls() {
        if (gamepad2.right_bumper && !lastRightBumper) intakeOn = !intakeOn;
        if (gamepad2.x && !lastX) flywheelsOn = !flywheelsOn;
        if (gamepad2.dpad_up && !lastDpadUp) shooter.setTargetRpm(shooter.getTargetRpm() + SHOOTER_RPM_STEP);
        if (gamepad2.dpad_down && !lastDpadDown) shooter.setTargetRpm(shooter.getTargetRpm() - SHOOTER_RPM_STEP);
        lastRightBumper = gamepad2.right_bumper;
        lastX = gamepad2.x;
        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;

        boolean shooting = gamepad2.right_trigger > 0.5;

        if (flywheelsOn || shooting) {
            shooter.spinUp();
        } else {
            shooter.stop();
        }

        if (gamepad2.left_bumper) {
            intake.reverse();
        } else if (shooting) {
            if (shooter.isReady()) {
                intake.feed();
            } else {
                intake.holdForShooter();
            }
        } else if (intakeOn) {
            intake.collect();
        } else {
            intake.stop();
        }
    }
}
