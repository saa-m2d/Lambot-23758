package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.commands.AimAndShoot;
import org.firstinspires.ftc.teamcode.commands.ShotSolver;
import org.firstinspires.ftc.teamcode.commands.TargetTracker;
import org.firstinspires.ftc.teamcode.config.VisionConstants;
import org.firstinspires.ftc.teamcode.diagnostics.ErrorDetectionSystem;
import org.firstinspires.ftc.teamcode.subsystems.BatteryMonitor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FlowerMechanism;
import org.firstinspires.ftc.teamcode.subsystems.HardwareCheck;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Localization;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseHistory;

import java.util.List;

/**
 * Todo el robot. Cada ciclo del OpMode:
 *   1. {@link #readInputs()}: lee sensores, ve la CELL, calcula el tiro.
 *   2. el OpMode decide (sticks, botones o el plan del autónomo).
 *   3. {@link #writeOutputs()}: cada subsistema aplica lo decidido y se revisan fallas.
 */
public class Robot {
    public final HardwareCheck hardware;
    public final BatteryMonitor battery;
    /** null en el autónomo con PedroPathing (Pedro maneja el chasis). */
    public final Drivetrain drivetrain;
    public final Localization localization;
    public final Intake intake;
    public final Indexer indexer;
    public final Shooter shooter;
    public final Turret turret;
    public final Vision vision;
    public final FlowerMechanism flower;

    public final TargetTracker tracker;
    public final ShotSolver solver = new ShotSolver();
    public final AimAndShoot aim;
    public final ErrorDetectionSystem diagnostics = new ErrorDetectionSystem();

    private final List<LynxModule> hubs;
    private ShotSolver.Solution solution = ShotSolver.Solution.invalid();
    private Vision.CellObservation observation;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs;

    /**
     * @param externalLocalizer el localizador del Follower de Pedro (autónomo), o null
     * @param useDrivetrain     false si Pedro maneja el chasis
     * @param turretHintDeg     ángulo esperado de la torreta al encender (0 = en home)
     */
    public Robot(HardwareMap hardwareMap, Alliance alliance, Localizer externalLocalizer,
                 boolean useDrivetrain, double turretHintDeg) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        hardware = new HardwareCheck(hardwareMap);
        battery = new BatteryMonitor(hardwareMap);
        drivetrain = useDrivetrain ? new Drivetrain(hardware) : null;
        localization = new Localization(hardware, externalLocalizer, externalLocalizer == null);
        intake = new Intake(hardware);
        indexer = new Indexer(hardware);
        shooter = new Shooter(hardware, battery);
        turret = new Turret(hardware, turretHintDeg);
        vision = new Vision(hardware);
        flower = new FlowerMechanism(hardware);

        tracker = new TargetTracker(alliance);
        aim = new AimAndShoot(turret, shooter, indexer);
    }

    /** Lee sensores, actualiza el objetivo y calcula el tiro. */
    public void readInputs() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        battery.update();
        localization.update();
        shooter.readSensors();
        turret.readSensors();
        localization.record(turret.angleDeg());

        Pose pose = localization.pose();
        vision.update(Math.toDegrees(pose.heading()) + 90 + turret.angleDeg());
        observation = vision.observe(tracker.alliance());
        tracker.update(observation, localization, localization.omega(), turret.rateDegS());
        if (VisionConstants.USE_MEGATAG) applyMegaTag2();

        solution = localization.isAvailable()
                ? solver.solve(localization.pose(), localization.vx(), localization.vy(), localization.omega(),
                tracker.target(), aim.shootOnMove(), aim.rpmTrim())
                : ShotSolver.Solution.invalid();
    }

    /** Aplica las salidas de todos los subsistemas y revisa fallas. */
    public void writeOutputs() {
        if (drivetrain != null) drivetrain.periodic();
        intake.periodic();
        shooter.periodic();
        turret.periodic();
        indexer.periodic();
        flower.periodic();
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        diagnostics.update(this, loopMs);
    }

    /** Apunta, prepara y dispara según lo que pide el OpMode. */
    public void updateAim(double manualTurret, boolean spinUp, boolean fire) {
        aim.update(solution, observation, manualTurret, spinUp, fire);
    }

    public ShotSolver.Solution solution() {
        return solution;
    }

    public Vision.CellObservation observation() {
        return observation;
    }

    public double loopMs() {
        return loopMs;
    }

    /** Corrige un poco la posición con MegaTag 2 si la lectura es creíble. */
    private void applyMegaTag2() {
        Pose3D megaTag = vision.megaTag2();
        if (megaTag == null || vision.megaTagTagCount() < 1) return;
        PoseHistory.Sample sample = localization.at(vision.captureNanos());
        if (sample == null) return;
        Position axis = megaTag.getPosition().toUnit(DistanceUnit.INCH);
        double heading = sample.pose.heading();
        double robotX = axis.x - (VisionConstants.TURRET_AXIS_FORWARD_IN * Math.cos(heading)
                - VisionConstants.TURRET_AXIS_LEFT_IN * Math.sin(heading));
        double robotY = axis.y - (VisionConstants.TURRET_AXIS_FORWARD_IN * Math.sin(heading)
                + VisionConstants.TURRET_AXIS_LEFT_IN * Math.cos(heading));
        if (Math.abs(robotX) > VisionConstants.FIELD_HALF_IN || Math.abs(robotY) > VisionConstants.FIELD_HALF_IN) return;
        double dx = robotX - sample.pose.x();
        double dy = robotY - sample.pose.y();
        if (Math.hypot(dx, dy) > VisionConstants.MEGATAG_MAX_JUMP_IN) return;
        localization.nudge(dx * VisionConstants.MEGATAG_BLEND, dy * VisionConstants.MEGATAG_BLEND);
    }

    /**
     * Pone la posición del robot (incluido el heading) con MegaTag 1. Solo con el robot quieto
     * viendo tags. Borra los objetivos aprendidos (cambia el marco de coordenadas).
     *
     * @return true si había una lectura válida
     */
    public boolean relocalizeWithMegaTag1() {
        Pose3D megaTag = vision.megaTag1();
        if (megaTag == null || vision.megaTagTagCount() < 1) return false;
        Position axis = megaTag.getPosition().toUnit(DistanceUnit.INCH);
        double cameraYawDeg = megaTag.getOrientation().getYaw(AngleUnit.DEGREES);
        double heading = Math.toRadians(cameraYawDeg - 90 - turret.angleDeg());
        double robotX = axis.x - (VisionConstants.TURRET_AXIS_FORWARD_IN * Math.cos(heading)
                - VisionConstants.TURRET_AXIS_LEFT_IN * Math.sin(heading));
        double robotY = axis.y - (VisionConstants.TURRET_AXIS_FORWARD_IN * Math.sin(heading)
                + VisionConstants.TURRET_AXIS_LEFT_IN * Math.cos(heading));
        localization.setPose(new Pose(robotX, robotY, heading));
        tracker.clear();
        return true;
    }

    /** Todo sin potencia. */
    public void stop() {
        if (drivetrain != null) drivetrain.stop();
        intake.stop();
        shooter.stop();
        indexer.stop();
        turret.stop();
        flower.stop();
        vision.stop();
    }

    public void addTelemetry(Telemetry telemetry) {
        diagnostics.addTelemetry(telemetry);
        aim.addTelemetry(telemetry, solution);
        telemetry.addData("Objetivo", "CELL %s%s  %s",
                tracker.targetCell() == null ? "ninguna" : tracker.targetCell().label,
                tracker.override() != null ? " (fijada a mano)" : "",
                tracker.target() == null ? "sin posición" : tracker.isFresh() ? "vista" : "solo odometría");
        localization.addTelemetry(telemetry);
        turret.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        indexer.addTelemetry(telemetry);
        intake.addTelemetry(telemetry);
        flower.addTelemetry(telemetry);
        vision.addTelemetry(telemetry);
        if (drivetrain != null) drivetrain.addTelemetry(telemetry);
        telemetry.addData("Batería / ciclo", "%.1f V  |  %.0f ms", battery.voltage(), loopMs);
    }
}
