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
import org.firstinspires.ftc.teamcode.config.VisionConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

/*
 * TeleOp de competencia de Furby. Control REV: ✕ = A, ○ = B, □ = X, △ = Y, Options = Start,
 * Share = Back, L1/R1 = bumpers, L2/R2 = gatillos.
 *
 * ANTES DE PLAY (INIT)
 *   ○ = alianza ROJA, ✕ = alianza AZUL (cualquier control). Si viene del autónomo, ya está puesta.
 *   Sin autónomo: el robot debe estar quieto, con la torreta en la marca de home y el frente
 *   del robot apuntando hacia el lado contrario a los pilotos ("adelante" del field-centric).
 *
 * PILOTO (gamepad1)
 *   Stick izquierdo        mover (field-centric: adelante = lejos de los pilotos)
 *   Stick derecho X        girar
 *   L2                     modo lento
 *   R1                     intakes meter: encender / apagar
 *   L1 (mantener)          intakes en reversa
 *   Options                "adelante" = hacia donde mira el robot ahora
 *   Share                  cambiar field-centric / robot-centric
 *   D-pad arriba           ubicar el robot con MegaTag 1 (solo si USE_MEGATAG; robot quieto)
 *
 * OPERADOR (gamepad2)
 *   R2 (mantener)          disparar: el tope abre solo con RPM listas y torreta en el objetivo
 *   ✕                      shooter preparado: encender / apagar
 *   □                      apuntado automático: encender / apagar
 *   Stick derecho X        mover la torreta a mano (al soltar, regresa a automático)
 *   D-pad arriba / abajo   preset de distancia (cuando no hay objetivo calculado)
 *   D-pad izquierda        fijar la CELL lejana al público (otra vez = automático)
 *   D-pad derecha          fijar la CELL del lado del público (otra vez = automático)
 *   R1 / L1                más / menos RPM (ajuste en vivo)
 *   ○                      disparo en movimiento: encender / apagar
 *   △                      flower: arriba / abajo (último minuto, o desbloqueado)
 *   Options                desbloquear el flower antes del último minuto
 *   Share                  borrar falla de la torreta (después de revisarla)
 *
 * LUZ DEL CONTROL DEL OPERADOR
 *   verde = listo / disparando, amarillo = apuntando o subiendo RPM, rojo = sin objetivo o fuera
 *   de rango, azul = shooter apagado.
 */
@TeleOp(name = "Furby TeleOp", group = "Competencia")
public class CompetitionTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
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
                    : "sin autónomo: torreta en home y robot mirando lejos de los pilotos");
            robot.addTelemetry(telemetry);
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
        boolean spinUp = false;
        boolean flowerUnlocked = !MechanismConstants.FLOWER_LOCK_UNTIL_ENDGAME;
        boolean endgameWarned = false;
        AimAndShoot.Status lastLedStatus = null;

        while (opModeIsActive()) {
            robot.readInputs();

            // ===== Piloto =====
            if (gamepad1.startWasPressed()) driverForward = robot.localization.pose().heading();
            if (gamepad1.backWasPressed()) fieldCentric = !fieldCentric;
            if (gamepad1.rightBumperWasPressed()) intakeOn = !intakeOn;
            if (gamepad1.dpadUpWasPressed() && VisionConstants.USE_MEGATAG) {
                boolean ok = robot.relocalizeWithMegaTag1();
                telemetry.log().add(ok ? "Posición tomada de MegaTag 1" : "MegaTag 1 sin lectura: no se cambió la posición");
            }

            double scale = gamepad1.left_trigger > 0.5 ? DriveConstants.SLOW_MODE_SCALE : 1.0;
            double forward = MathUtil.shapeStick(-gamepad1.left_stick_y, DriveConstants.STICK_DEADBAND, DriveConstants.STICK_EXPONENT) * scale;
            double left = MathUtil.shapeStick(-gamepad1.left_stick_x, DriveConstants.STICK_DEADBAND, DriveConstants.STICK_EXPONENT) * scale;
            double turn = MathUtil.shapeStick(-gamepad1.right_stick_x, DriveConstants.STICK_DEADBAND, DriveConstants.STICK_EXPONENT)
                    * DriveConstants.TURN_SCALE * scale;
            boolean canFieldCentric = fieldCentric && robot.localization.isAvailable();
            if (canFieldCentric) {
                robot.drivetrain.driveFieldCentric(forward, left, turn,
                        MathUtil.wrapPi(robot.localization.pose().heading() - driverForward));
            } else {
                robot.drivetrain.driveRobotCentric(forward, left, turn);
            }

            if (gamepad1.left_bumper) {
                robot.intake.set(Intake.Mode.OUT);
            } else {
                robot.intake.set(intakeOn ? Intake.Mode.IN : Intake.Mode.OFF);
            }

            // ===== Operador =====
            if (gamepad2.aWasPressed()) spinUp = !spinUp;
            if (gamepad2.xWasPressed()) robot.aim.setAutoAim(!robot.aim.autoAim());
            if (gamepad2.bWasPressed()) robot.aim.setShootOnMove(!robot.aim.shootOnMove());
            if (gamepad2.dpadUpWasPressed()) robot.aim.cyclePreset(1);
            if (gamepad2.dpadDownWasPressed()) robot.aim.cyclePreset(-1);
            if (gamepad2.dpadLeftWasPressed()) toggleOverride(robot, Alliance.Cell.FAR);
            if (gamepad2.dpadRightWasPressed()) toggleOverride(robot, Alliance.Cell.AUDIENCE);
            if (gamepad2.rightBumperWasPressed()) robot.aim.adjustTrim(1);
            if (gamepad2.leftBumperWasPressed()) robot.aim.adjustTrim(-1);
            if (gamepad2.startWasPressed()) flowerUnlocked = !flowerUnlocked;
            if (gamepad2.backWasPressed()) robot.turret.clearFaults();

            boolean endgame = matchTimer.seconds() >= MechanismConstants.ENDGAME_START_S;
            if (gamepad2.yWasPressed() && (endgame || flowerUnlocked)) robot.flower.toggle();

            double manualTurret = MathUtil.shapeStick(-gamepad2.right_stick_x, DriveConstants.STICK_DEADBAND, 1.0);
            boolean fire = gamepad2.right_trigger > 0.5;
            robot.updateAim(manualTurret, spinUp, fire);

            robot.writeOutputs();

            // ===== Avisos =====
            if (robot.diagnostics.consumeNewCritical()) {
                gamepad1.rumble(400);
                gamepad2.rumble(400);
            }
            if (endgame && !endgameWarned) {
                endgameWarned = true;
                gamepad1.rumbleBlips(3);
                gamepad2.rumbleBlips(3);
            }
            AimAndShoot.Status status = robot.aim.status();
            if (status != lastLedStatus) {
                setLed(gamepad2, status);
                lastLedStatus = status;
            }

            telemetry.addData("Alianza", "%s  |  %s  |  %.0f s", alliance.label,
                    canFieldCentric ? "field-centric" : "robot-centric", matchTimer.seconds());
            telemetry.addData("Flower", endgame ? "último minuto: habilitado"
                    : flowerUnlocked ? "desbloqueado" : "bloqueado hasta el último minuto");
            robot.addTelemetry(telemetry);
            telemetry.update();
        }
        robot.stop();
    }

    private static void toggleOverride(Robot robot, Alliance.Cell cell) {
        robot.tracker.setOverride(robot.tracker.override() == cell ? null : cell);
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
