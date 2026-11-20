package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.AimAndShoot;
import org.firstinspires.ftc.teamcode.config.DriveConstants;
import org.firstinspires.ftc.teamcode.config.MechanismConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

/*
 * TeleOp de competencia de Furby, para UN SOLO PILOTO (gamepad1).
 *
 * Lo automático: la torreta apunta sola a la CELL; el shooter se prepara solo en cuanto hay
 * objetivo; el tope solo deja pasar la pelota con la torreta apuntando y las RPM listas.
 * El piloto maneja, mete pelotas y aprieta R2.
 *
 * ANTES DE PLAY (INIT): ○ = alianza ROJA, ✕ = alianza AZUL. Si viene del autónomo, ya está puesta.
 *   Sin autónomo: robot quieto, torreta en home y el frente del robot mirando lejos del piloto.
 *
 * CONTROL (REV: ✕ = A, ○ = B, □ = X, △ = Y, Options = Start, Share = Back)
 *   Stick izquierdo        mover (field-centric: adelante = lejos del piloto)
 *   Stick derecho X        girar
 *   R2 (mantener)          DISPARAR
 *   R1                     intake: encender / apagar
 *   L1 (mantener)          intake en reversa
 *   L2 (mantener)          modo lento
 *
 *   D-pad ← / → (mantener) torreta a mano (al soltar, vuelve a apuntar sola)
 *   D-pad ↑ / ↓            más / menos RPM (si los tiros quedan cortos o largos)
 *   △                      shooter: automático / apagado
 *   □                      apuntado automático: encendido / apagado
 *   ✕                      distancia para tirar SIN cámara: cerca → medio → lejos
 *   ○                      flower arriba / abajo (solo en el último minuto)
 *   Options                "adelante" = hacia donde mira el robot ahora
 *   Share                  field-centric / robot-centric
 *   Touchpad               ver toda la información (otra vez: pantalla corta)
 *
 * LUZ DEL CONTROL: verde = listo para disparar, amarillo = apuntando o subiendo RPM,
 *   rojo = sin objetivo o fuera de rango, azul = shooter apagado.
 * VIBRA: 3 golpes al empezar el último minuto; uno largo si aparece una falla importante.
 */
@TeleOp(name = "Furby TeleOp", group = "Competencia")
public class CompetitionTeleOp extends LinearOpMode {

    /** Log del Driver Hub (la telemetría combinada con Panels no tiene log()). */
    org.firstinspires.ftc.robotcore.external.Telemetry.Log dsLog;

    /** Potencia de la torreta con el D-pad (fracción de la potencia manual máxima). */
    static final double DPAD_TURRET_POWER = 0.6;

    @Override
    public void runOpMode() {
        dsLog = telemetry.log();
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());

        boolean fromAuto = PoseStorage.isFresh();
        Alliance alliance = fromAuto && PoseStorage.alliance() != null ? PoseStorage.alliance() : Alliance.RED;
        Robot robot = new Robot(hardwareMap, alliance, null, true, fromAuto ? PoseStorage.turretDeg() : 0);
        double driverForward = 0;
        if (fromAuto) {
            robot.localization.setPose(PoseStorage.pose());
            driverForward = PoseStorage.driverForwardHeading();
        } else {
            // El Pinpoint guarda la posición de la última prueba: se empieza de cero
            robot.localization.setPose(Pose.zero());
        }
        PoseStorage.clear();

        // ---------- INIT ----------
        while (opModeInInit()) {
            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) alliance = Alliance.RED;
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) alliance = Alliance.BLUE;
            robot.tracker.setAlliance(alliance);
            robot.readInputs();
            robot.diagnostics.update(robot, 0);

            telemetry.addData("Alianza", "%s   (○ roja, ✕ azul)", alliance.label);
            telemetry.addData("Inicio", fromAuto ? "posición y torreta del autónomo"
                    : "sin autónomo: torreta en home y robot mirando lejos del piloto");
            robot.addCompactTelemetry(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        // ---------- PLAY ----------
        ElapsedTime matchTimer = new ElapsedTime();
        boolean fieldCentric = true;
        boolean intakeOn = false;
        boolean shooterAuto = true;
        boolean endgameWarned = false;
        boolean fullTelemetry = false;
        AimAndShoot.Status lastLedStatus = null;
        Gamepad pad = gamepad1;

        while (opModeIsActive()) {
            robot.readInputs();

            // ===== Botones =====
            if (pad.startWasPressed()) driverForward = robot.localization.pose().heading();
            if (pad.backWasPressed()) fieldCentric = !fieldCentric;
            if (pad.rightBumperWasPressed()) intakeOn = !intakeOn;
            if (pad.yWasPressed()) shooterAuto = !shooterAuto;
            if (pad.xWasPressed()) robot.aim.setAutoAim(!robot.aim.autoAim());
            if (pad.aWasPressed()) robot.aim.cyclePreset(1);
            if (pad.dpadUpWasPressed()) robot.aim.adjustTrim(1);
            if (pad.dpadDownWasPressed()) robot.aim.adjustTrim(-1);
            if (pad.touchpadWasPressed()) fullTelemetry = !fullTelemetry;
            boolean endgame = matchTimer.seconds() >= MechanismConstants.ENDGAME_START_S
                    || !MechanismConstants.FLOWER_LOCK_UNTIL_ENDGAME;
            if (pad.bWasPressed() && endgame) robot.flower.toggle();

            // ===== Manejo =====
            double scale = pad.left_trigger > 0.5 ? DriveConstants.SLOW_MODE_SCALE : 1.0;
            double forward = MathUtil.shapeStick(-pad.left_stick_y, DriveConstants.STICK_DEADBAND, DriveConstants.STICK_EXPONENT) * scale;
            double left = MathUtil.shapeStick(-pad.left_stick_x, DriveConstants.STICK_DEADBAND, DriveConstants.STICK_EXPONENT) * scale;
            double turn = MathUtil.shapeStick(-pad.right_stick_x, DriveConstants.STICK_DEADBAND, DriveConstants.STICK_EXPONENT)
                    * DriveConstants.TURN_SCALE * scale;
            boolean canFieldCentric = fieldCentric && robot.localization.isAvailable();
            if (canFieldCentric) {
                robot.drivetrain.driveFieldCentric(forward, left, turn,
                        MathUtil.wrapPi(robot.localization.pose().heading() - driverForward));
            } else {
                robot.drivetrain.driveRobotCentric(forward, left, turn);
            }

            // ===== Intake =====
            if (pad.left_bumper) {
                robot.intake.set(Intake.Mode.OUT);
            } else {
                robot.intake.set(intakeOn ? Intake.Mode.IN : Intake.Mode.OFF);
            }

            // ===== Torreta, shooter y tope =====
            double manualTurret = pad.dpad_left ? DPAD_TURRET_POWER : pad.dpad_right ? -DPAD_TURRET_POWER : 0;
            boolean fire = pad.right_trigger > 0.5;
            // El shooter se prepara solo en cuanto hay un tiro calculado (así ya está listo al apretar R2)
            boolean spinUp = shooterAuto && robot.solution().valid;
            robot.updateAim(manualTurret, spinUp, fire && shooterAuto);

            robot.writeOutputs();

            // ===== Avisos =====
            if (robot.diagnostics.consumeNewCritical()) pad.rumble(500);
            if (matchTimer.seconds() >= MechanismConstants.ENDGAME_START_S && !endgameWarned) {
                endgameWarned = true;
                pad.rumbleBlips(3);
            }
            AimAndShoot.Status status = shooterAuto ? robot.aim.status() : AimAndShoot.Status.IDLE;
            if (status != lastLedStatus) {
                setLed(pad, status);
                lastLedStatus = status;
            }

            // ===== Pantalla =====
            telemetry.addData("Partido", "%s  |  %s  |  %.0f s", alliance.label,
                    canFieldCentric ? "field-centric" : "robot-centric", matchTimer.seconds());
            telemetry.addData("Modos", "shooter %s  |  apuntado %s  |  intake %s",
                    shooterAuto ? "AUTO" : "APAGADO", robot.aim.autoAim() ? "AUTO" : "MANUAL", intakeOn ? "ON" : "off");
            if (!robot.aim.isUsingSolution()) {
                telemetry.addData("Sin cámara", "distancia %.0f in (✕ cambia)", robot.aim.presetDistance());
            }
            if (robot.aim.rpmTrim() != 0) telemetry.addData("Ajuste RPM", "%+.0f", robot.aim.rpmTrim());
            telemetry.addData("Flower", endgame ? "habilitado (○)" : "bloqueado hasta el último minuto");
            if (fullTelemetry) {
                robot.addTelemetry(telemetry);
            } else {
                robot.addCompactTelemetry(telemetry);
            }
            telemetry.update();
        }
        robot.stop();
    }

    private static void setLed(Gamepad gamepad, AimAndShoot.Status status) {
        switch (status) {
            case READY:
            case FIRING:
                gamepad.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case AIMING:
            case SPINNING_UP:
            case TRACKING_ANGLE:
                gamepad.setLedColor(1, 0.8, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            case NO_TARGET:
            case OUT_OF_RANGE:
                gamepad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                break;
            default:
                gamepad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}
