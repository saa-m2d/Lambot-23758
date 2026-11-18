package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.config.RobotMap;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * Verifica la Limelight 3A: conexión, AprilTags a la vista, MegaTag 1 y MegaTag 2.
 *
 * La torreta no se mueve en esta prueba. Déjala quieta todo el tiempo: MegaTag 2 necesita saber
 * hacia dónde mira la CÁMARA, y aquí eso se calcula como heading del Pinpoint + un ajuste fijo.
 *
 * gamepad1
 *   X: sincronizar. Toma el heading que calcula MegaTag 1 y lo usa como ajuste para MegaTag 2.
 *      Hazlo con el robot quieto y viendo bien al menos un tag.
 *   D-pad arriba/abajo: cambiar de pipeline (0 a 9)
 *
 * Después de sincronizar, gira el chasis: la posición de MT2 debe mantenerse casi igual y parecerse
 * a la de MT1. Si MT2 salta o se aleja, el heading que se le manda no es el de la cámara.
 *
 * Sin el Pinpoint en la configuración, la prueba funciona pero solo muestra MegaTag 1.
 */
@TeleOp(name = "Test 4: Limelight", group = "Test")
public class VisionTestOpMode extends LinearOpMode {

    static final int POLL_RATE_HZ = 100;
    static final long STALE_MS = 200;        // un resultado más viejo que esto ya no sirve
    static final int MAX_PIPELINE = 9;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.tryGet(Limelight3A.class, RobotMap.LIMELIGHT);
        if (limelight == null) {
            while (!isStopRequested()) {
                telemetry.addLine("FALLA: no hay un dispositivo \"" + RobotMap.LIMELIGHT + "\" en la configuración.");
                telemetry.addLine("Conéctala al USB 3.0 (azul) del Control Hub y usa Scan en la configuración.");
                telemetry.update();
                sleep(100);
            }
            return;
        }
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, RobotMap.PINPOINT);
        if (pinpoint != null) pinpoint.resetPosAndIMU();

        int pipeline = 0;
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.pipelineSwitch(pipeline);
        limelight.start();

        telemetry.addLine("Limelight encontrada. Robot quieto y PLAY.");
        telemetry.addData("Pinpoint", pinpoint != null ? "encontrado" : "no está: solo se mostrará MegaTag 1");
        telemetry.update();
        waitForStart();

        double yawOffset = 0;        // heading de la cámara en el campo - heading del Pinpoint
        boolean synced = false;

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed() && pipeline < MAX_PIPELINE) limelight.pipelineSwitch(++pipeline);
            if (gamepad1.dpadDownWasPressed() && pipeline > 0) limelight.pipelineSwitch(--pipeline);
            boolean syncRequested = gamepad1.xWasPressed();

            double pinpointHeading = 0;
            if (pinpoint != null) {
                pinpoint.update();
                pinpointHeading = pinpoint.getHeading(AngleUnit.DEGREES);
                // MegaTag 2 usa este heading en vez de calcularlo con los tags
                limelight.updateRobotOrientation(AngleUnit.normalizeDegrees(pinpointHeading + yawOffset));
            }

            LLStatus status = limelight.getStatus();
            LLResult result = limelight.getLatestResult();
            boolean fresh = result != null && result.getStaleness() < STALE_MS;
            boolean valid = fresh && result.isValid();

            telemetry.addData("Conexión", limelight.isConnected() ? "OK" : "FALLA: sin comunicación (cable USB)");
            telemetry.addData("Limelight", "%s  %.0f fps  %.0f °C  CPU %.0f %%",
                    status.getName(), status.getFps(), status.getTemp(), status.getCpu());
            telemetry.addData("Pipeline", "%d (%s)", status.getPipelineIndex(), status.getPipelineType());

            if (!fresh) {
                telemetry.addLine("Sin resultados recientes. ¿La Limelight está corriendo un pipeline?");
            } else if (!valid) {
                telemetry.addLine("No ve ningún objetivo.");
            } else {
                telemetry.addData("Objetivo principal", "tx %.1f°  ty %.1f°  área %.2f %%",
                        result.getTx(), result.getTy(), result.getTa());
                telemetry.addData("Latencia", "%.0f ms", result.getCaptureLatency() + result.getTargetingLatency());

                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    telemetry.addData("  Tag " + tag.getFiducialId(), "%s   tx %.1f°  ty %.1f°",
                            describeTag(tag.getFiducialId()), tag.getTargetXDegrees(), tag.getTargetYDegrees());
                    // Posición respecto al eje de la torreta, como la usa el apuntado. Revisa con
                    // cinta: "adelante" y "distancia" deben coincidir con lo medido, y el ángulo
                    // debe ser casi igual a -tx.
                    double[] turret = Vision.cameraToTurret(tag.getTargetPoseCameraSpace());
                    if (turret != null) {
                        telemetry.addData("    desde la torreta", "adelante %.1f  izq %.1f  arriba %.1f in  |  distancia %.1f in  ángulo %.1f°",
                                turret[0], turret[1], turret[2], Math.hypot(turret[0], turret[1]),
                                Math.toDegrees(Math.atan2(turret[1], turret[0])));
                    } else {
                        telemetry.addData("    desde la torreta", "sin posición 3D (¿Full 3D Targeting en Yes?)");
                    }
                }

                Pose3D mt1 = result.getBotpose();
                telemetry.addData("MegaTag 1", "%s  (%d tags)", format(mt1), result.getBotposeTagCount());

                if (syncRequested && mt1 != null && pinpoint != null) {
                    yawOffset = AngleUnit.normalizeDegrees(
                            mt1.getOrientation().getYaw(AngleUnit.DEGREES) - pinpointHeading);
                    synced = true;
                }
                if (pinpoint != null) {
                    telemetry.addData("MegaTag 2", synced ? format(result.getBotpose_MT2()) : "presiona X para sincronizar");
                }
            }
            if (pinpoint != null) {
                telemetry.addData("Heading Pinpoint", "%.1f°   ajuste %.1f°", pinpointHeading, yawOffset);
            }
            telemetry.update();
        }
        limelight.stop();
    }

    /** A qué CELL pertenece un tag de BIOBUZZ (manual, sección 9.9). */
    static String describeTag(int id) {
        if (id >= 30 && id <= 33) return "CELL roja, lado opuesto al público";
        if (id >= 34 && id <= 37) return "CELL roja, lado del público";
        if (id >= 38 && id <= 41) return "CELL azul, lado del público";
        if (id >= 42 && id <= 45) return "CELL azul, lado opuesto al público";
        return "no es un tag de BIOBUZZ";
    }

    static String format(Pose3D pose) {
        if (pose == null) return "sin dato";
        Position p = pose.getPosition().toUnit(DistanceUnit.CM);
        return String.format("x %.0f  y %.0f cm  yaw %.1f°", p.x, p.y, pose.getOrientation().getYaw(AngleUnit.DEGREES));
    }
}
