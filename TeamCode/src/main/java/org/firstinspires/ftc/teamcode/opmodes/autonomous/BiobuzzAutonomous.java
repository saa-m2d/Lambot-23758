package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.AimAndShoot;
import org.firstinspires.ftc.teamcode.config.AutoConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

import java.util.ArrayList;
import java.util.List;

/**
 * Autónomo de BIOBUZZ, igual para las dos alianzas (cambian las posiciones de AutoConstants).
 *
 * Completo (Pedro tuneado y poses configuradas):
 *   ir a la posición de tiro → disparar precargadas → ir a recoger → recoger → regresar → disparar
 *   → salir (LEAVE).
 * Sencillo (si falta algo de lo anterior):
 *   disparar precargadas desde la posición de inicio → avanzar para salir de la zona.
 *
 * La torreta apunta sola todo el tiempo. Al terminar se guarda la posición del robot y el ángulo
 * de la torreta para el TeleOp.
 */
public abstract class BiobuzzAutonomous extends LinearOpMode {

    protected abstract Alliance alliance();

    private enum StepType { FOLLOW, SHOOT, SIMPLE_LEAVE }

    private static final class Step {
        final StepType type;
        final String label;
        final Path path;
        final Intake.Mode intake;
        final boolean spinUp;

        Step(StepType type, String label, Path path, Intake.Mode intake, boolean spinUp) {
            this.type = type;
            this.label = label;
            this.path = path;
            this.intake = intake;
            this.spinUp = spinUp;
        }
    }

    @Override
    public void runOpMode() {
        telemetry = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        Alliance alliance = alliance();
        boolean red = alliance == Alliance.RED;

        // Pedro solo si está tuneado y hay trayectorias medidas
        Follower follower = null;
        Localizer localizer = null;
        if (Constants.FORESIGHT_TUNED && AutoConstants.PATHS_CONFIGURED) {
            localizer = Constants.createLocalizer(hardwareMap);
            follower = Constants.createFollower(hardwareMap, localizer);
        }
        Robot robot = new Robot(hardwareMap, alliance, localizer, follower == null, 0);

        Pose start = pose(red ? AutoConstants.RED_START : AutoConstants.BLUE_START);
        robot.localization.setPose(start);
        if (AutoConstants.USE_CELL_PRIORS) {
            double[] far = red ? AutoConstants.RED_FAR_CELL : AutoConstants.BLUE_FAR_CELL;
            double[] audience = red ? AutoConstants.RED_AUDIENCE_CELL : AutoConstants.BLUE_AUDIENCE_CELL;
            robot.tracker.setPrior(Alliance.Cell.FAR, far[0], far[1]);
            robot.tracker.setPrior(Alliance.Cell.AUDIENCE, audience[0], audience[1]);
        }
        List<Step> steps = follower != null ? fullPlan(red, start) : simplePlan();
        double driverForward = Math.toRadians(red ? AutoConstants.RED_DRIVER_FORWARD_DEG : AutoConstants.BLUE_DRIVER_FORWARD_DEG);

        while (opModeInInit()) {
            robot.readInputs();
            robot.diagnostics.update(robot, 0);
            telemetry.addData("Autónomo", "%s  |  %s", alliance.label, follower != null ? "COMPLETO (Pedro)"
                    : !Constants.FORESIGHT_TUNED ? "SENCILLO: Pedro sin tunear" : "SENCILLO: poses sin configurar");
            telemetry.addLine("Torreta en la marca de home y robot en la posición de inicio.");
            robot.addTelemetry(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) {
            robot.stop();
            return;
        }

        ElapsedTime matchTimer = new ElapsedTime();
        ElapsedTime stepTimer = new ElapsedTime();
        ElapsedTime shootTimer = new ElapsedTime();
        boolean shooting = false;
        int index = 0;
        boolean stepStarted = false;

        while (opModeIsActive() && matchTimer.seconds() < AutoConstants.AUTO_END_S) {
            if (follower != null) follower.update();
            robot.readInputs();

            Step step = index < steps.size() ? steps.get(index) : null;
            boolean spinUp = false, fire = false;
            if (step != null) {
                if (!stepStarted) {
                    stepStarted = true;
                    stepTimer.reset();
                    shooting = false;
                    if (step.type == StepType.FOLLOW && follower != null) follower.follow(step.path);
                }
                robot.intake.set(step.intake);
                spinUp = step.spinUp;
                boolean done;
                switch (step.type) {
                    case FOLLOW:
                        done = follower == null || !follower.isBusy() || stepTimer.seconds() > AutoConstants.PATH_TIMEOUT_S;
                        break;
                    case SHOOT:
                        // Solo dispara con un tiro calculado: a ciegas se perderían las pelotas
                        fire = robot.aim.isUsingSolution();
                        if (!shooting && robot.aim.status() == AimAndShoot.Status.FIRING) {
                            shooting = true;
                            shootTimer.reset();
                        }
                        done = (shooting && shootTimer.seconds() > AutoConstants.SHOOT_TIME_S)
                                || stepTimer.seconds() > AutoConstants.SHOOT_TIMEOUT_S;
                        break;
                    case SIMPLE_LEAVE:
                        robot.drivetrain.driveRobotCentric(AutoConstants.SIMPLE_LEAVE_FORWARD, AutoConstants.SIMPLE_LEAVE_LEFT, 0);
                        done = stepTimer.seconds() > AutoConstants.SIMPLE_LEAVE_TIME_S;
                        if (done) robot.drivetrain.driveRobotCentric(0, 0, 0);
                        break;
                    default:
                        done = true;
                }
                if (done) {
                    index++;
                    stepStarted = false;
                }
            } else {
                robot.intake.set(Intake.Mode.OFF);
                if (robot.drivetrain != null) robot.drivetrain.driveRobotCentric(0, 0, 0);
            }

            robot.updateAim(0, spinUp, fire);
            robot.writeOutputs();

            telemetry.addData("Paso", step == null ? "terminado" : (index + 1) + "/" + steps.size() + ": " + step.label);
            telemetry.addData("Tiempo", "%.1f s", matchTimer.seconds());
            robot.addTelemetry(telemetry);
            telemetry.update();
        }

        PoseStorage.save(robot.localization.pose(), robot.turret.angleDeg(), alliance, driverForward);
        if (follower != null) follower.stop();
        robot.stop();
    }

    private List<Step> fullPlan(boolean red, Pose start) {
        Pose shoot = pose(red ? AutoConstants.RED_SHOOT : AutoConstants.BLUE_SHOOT);
        Pose pickupStart = pose(red ? AutoConstants.RED_PICKUP_START : AutoConstants.BLUE_PICKUP_START);
        Pose pickupEnd = pose(red ? AutoConstants.RED_PICKUP_END : AutoConstants.BLUE_PICKUP_END);
        Pose leave = pose(red ? AutoConstants.RED_LEAVE : AutoConstants.BLUE_LEAVE);

        List<Step> steps = new ArrayList<>();
        steps.add(new Step(StepType.FOLLOW, "ir a tirar", line(start, shoot), Intake.Mode.OFF, true));
        if (AutoConstants.SHOOT_PRELOAD) {
            steps.add(new Step(StepType.SHOOT, "disparar precargadas", null, Intake.Mode.OFF, true));
        }
        steps.add(new Step(StepType.FOLLOW, "ir a recoger", line(shoot, pickupStart), Intake.Mode.OFF, false));
        steps.add(new Step(StepType.FOLLOW, "recoger", line(pickupStart, pickupEnd), Intake.Mode.IN, false));
        steps.add(new Step(StepType.FOLLOW, "regresar a tirar", line(pickupEnd, shoot), Intake.Mode.IN, true));
        steps.add(new Step(StepType.SHOOT, "disparar", null, Intake.Mode.IN, true));
        steps.add(new Step(StepType.FOLLOW, "salir (LEAVE)", line(shoot, leave), Intake.Mode.OFF, false));
        return steps;
    }

    private List<Step> simplePlan() {
        List<Step> steps = new ArrayList<>();
        if (AutoConstants.SHOOT_PRELOAD) {
            steps.add(new Step(StepType.SHOOT, "disparar precargadas", null, Intake.Mode.OFF, true));
        }
        steps.add(new Step(StepType.SIMPLE_LEAVE, "salir (LEAVE)", null, Intake.Mode.OFF, false));
        return steps;
    }

    private static Path line(Pose from, Pose to) {
        return Paths.line(from, to).linear(from, to);
    }

    private static Pose pose(double[] values) {
        return new Pose(values[0], values[1], Math.toRadians(values[2]));
    }
}
