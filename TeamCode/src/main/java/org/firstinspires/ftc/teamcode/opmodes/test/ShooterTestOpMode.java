package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.ShotSolver;
import org.firstinspires.ftc.teamcode.config.DriveConstants;
import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/*
 * Llena la tabla de tiro: desde varias distancias, busca las RPM de cada motor que meten la
 * POLLEN en la CELL y anótalas en ShooterConstants (TABLE_DISTANCE_IN, TABLE_RPM_LEFT/RIGHT).
 *
 * Todo con gamepad1 (control REV: ✕ = A, ○ = B, □ = X, △ = Y):
 *   INIT: ○ = alianza roja, ✕ = alianza azul (para saber qué CELL buscar).
 *   Stick izquierdo / derecho X  mover el robot despacio (sin field-centric)
 *   R1                           intakes meter: encender / apagar
 *   △                            shooter: encender / apagar
 *   R2 (mantener)                disparar (el tope abre solo con las RPM listas)
 *   D-pad arriba / abajo         las dos RPM ±50
 *   D-pad derecha / izquierda    más RPM al derecho / al izquierdo (±25 cada uno, cambia el efecto)
 *   ✕                            copiar las RPM que da la tabla para la distancia actual
 *   □                            anotar distancia y RPM en la lista de abajo
 *   L1 / L2 (mantener)           girar la torreta a mano a la izquierda / derecha
 *   La torreta apunta sola a la CELL mientras no se use L1/L2.
 *
 * Procedimiento:
 *   1. Pon el robot donde vea la CELL. Espera a que "Distancia" muestre un valor.
 *   2. Enciende el shooter (△), dispara (R2) y ajusta RPM hasta que entre seguido.
 *   3. Anota con □. Repite en 6 a 10 distancias, de cerca a lejos.
 *   4. Copia las distancias y RPM a ShooterConstants (de menor a mayor distancia).
 * También sirve para estimar el tiempo de vuelo (TOF_SECONDS) grabando un video del tiro.
 */
@TeleOp(name = "Test 6: Tiro", group = "Test")
public class ShooterTestOpMode extends LinearOpMode {

    /** Log del Driver Hub (la telemetría combinada con Panels no tiene log()). */
    org.firstinspires.ftc.robotcore.external.Telemetry.Log dsLog;

    static final double RPM_STEP = 50;
    static final double SPLIT_STEP = 25;
    static final double DRIVE_SCALE = 0.35;
    static final double MANUAL_TURRET = 0.6;

    @Override
    public void runOpMode() {
        dsLog = telemetry.log();   // la telemetría combinada con Panels no tiene log()
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        Alliance alliance = Alliance.RED;
        Robot robot = new Robot(hardwareMap, alliance, null, true, 0);
        robot.localization.setPose(Pose.zero());

        while (opModeInInit()) {
            if (gamepad1.bWasPressed()) alliance = Alliance.RED;
            if (gamepad1.aWasPressed()) alliance = Alliance.BLUE;
            robot.tracker.setAlliance(alliance);
            robot.readInputs();
            robot.diagnostics.update(robot, 0);
            telemetry.addData("Alianza", "%s  (○ roja, ✕ azul)", alliance.label);
            telemetry.addLine("Torreta en la marca de home al presionar INIT.");
            robot.addTelemetry(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        boolean shooterOn = false;
        boolean intakeOn = false;
        double rpmLeft = ShooterConstants.PREPARE_RPM;
        double rpmRight = ShooterConstants.PREPARE_RPM;

        while (opModeIsActive()) {
            robot.readInputs();
            ShotSolver.Solution solution = robot.solution();

            if (gamepad1.yWasPressed()) shooterOn = !shooterOn;
            if (gamepad1.rightBumperWasPressed()) intakeOn = !intakeOn;
            if (gamepad1.dpadUpWasPressed()) { rpmLeft += RPM_STEP; rpmRight += RPM_STEP; }
            if (gamepad1.dpadDownWasPressed()) { rpmLeft -= RPM_STEP; rpmRight -= RPM_STEP; }
            if (gamepad1.dpadRightWasPressed()) { rpmRight += SPLIT_STEP; rpmLeft -= SPLIT_STEP; }
            if (gamepad1.dpadLeftWasPressed()) { rpmLeft += SPLIT_STEP; rpmRight -= SPLIT_STEP; }
            if (gamepad1.aWasPressed() && solution.valid) {
                double[] table = ShotSolver.rpmFor(solution.distanceIn);
                rpmLeft = table[0];
                rpmRight = table[1];
            }
            rpmLeft = MathUtil.clamp(rpmLeft, 0, ShooterConstants.MAX_RPM);
            rpmRight = MathUtil.clamp(rpmRight, 0, ShooterConstants.MAX_RPM);
            if (gamepad1.xWasPressed()) {
                dsLog.add("distancia %s in  ->  izq %.0f  der %.0f RPM",
                        solution.valid ? String.format("%.0f", solution.distanceIn) : "?", rpmLeft, rpmRight);
            }

            // Chasis despacio, sin field-centric
            double forward = MathUtil.shapeStick(-gamepad1.left_stick_y, DriveConstants.STICK_DEADBAND, 1) * DRIVE_SCALE;
            double left = MathUtil.shapeStick(-gamepad1.left_stick_x, DriveConstants.STICK_DEADBAND, 1) * DRIVE_SCALE;
            double turn = MathUtil.shapeStick(-gamepad1.right_stick_x, DriveConstants.STICK_DEADBAND, 1) * DRIVE_SCALE;
            robot.drivetrain.driveRobotCentric(forward, left, turn);
            robot.intake.set(intakeOn ? Intake.Mode.IN : Intake.Mode.OFF);

            // Torreta: automática salvo con L1/L2. El shooter lo controla esta prueba.
            double manualTurret = gamepad1.left_bumper ? MANUAL_TURRET : gamepad1.left_trigger > 0.5 ? -MANUAL_TURRET : 0;
            robot.updateAim(manualTurret, false, false);
            if (shooterOn) robot.shooter.setTargets(rpmLeft, rpmRight); else robot.shooter.off();
            robot.indexer.setOpen(gamepad1.right_trigger > 0.5 && robot.shooter.isReady());

            robot.writeOutputs();

            telemetry.addData("Distancia", solution.valid ? String.format("%.0f in", solution.distanceIn) : "sin objetivo");
            if (solution.valid) {
                double[] table = ShotSolver.rpmFor(solution.distanceIn);
                telemetry.addData("Tabla dice", "izq %.0f  der %.0f RPM", table[0], table[1]);
            }
            telemetry.addData("Prueba", "izq %.0f  der %.0f RPM  %s", rpmLeft, rpmRight,
                    !shooterOn ? "(apagado)" : robot.shooter.isReady() ? "LISTO" : "subiendo");
            telemetry.addData("Torreta en objetivo", robot.turret.isOnTarget() ? "sí" : "no");
            robot.addTelemetry(telemetry);
            telemetry.update();
        }
        robot.stop();
    }
}
