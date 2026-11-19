package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.config.DriveConstants;
import org.firstinspires.ftc.teamcode.config.RobotMap;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.HardwareCheck;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/*
 * Verifica el goBILDA Pinpoint y sus dos pods. El robot se puede mover con el chasis o a mano.
 *
 * gamepad1 (control REV: A = ✕, B = ○)
 *   A: poner la posición en cero y recalibrar el giroscopio (robot QUIETO, tarda ~0.25 s)
 *   B: recalibrar solo el giroscopio (robot QUIETO)
 *   Stick izquierdo:         mover (field-centric: "adelante" = hacia donde miraba al presionar A)
 *   Stick derecho X:         girar
 *   D-pad izquierda/derecha: girar en su lugar despacio (para la prueba de las 10 vueltas)
 *   L2 (mantener):           más despacio
 *   Back (Share):            cambiar field-centric / robot-centric
 *
 * Qué debe pasar al mover el robot:
 *   hacia ADELANTE          -> X sube
 *   hacia la IZQUIERDA      -> Y sube
 *   girar ANTIHORARIO       -> heading sube
 * Si X o Y bajan, cambia la dirección de ese pod en RobotMap (PINPOINT_X_POD_DIRECTION /
 * PINPOINT_Y_POD_DIRECTION): de ahí la toman esta prueba, el TeleOp y PedroPathing.
 *
 * Prueba de offsets: gira el robot 10 vueltas sobre su centro. X y Y deben regresar cerca de 0.
 * Si se alejan mucho, los offsets de RobotMap están mal medidos.
 */
@TeleOp(name = "Test 3: Pinpoint", group = "Test")
public class PinpointTestOpMode extends LinearOpMode {

    // El Pinpoint actualiza a unos 1500 Hz. Fuera de este rango, algo anda mal con el módulo.
    static final double MIN_FREQUENCY_HZ = 900;
    static final double MAX_FREQUENCY_HZ = 2000;

    static final double DRIVE_MAX = 0.8;     // potencia máxima del chasis en esta prueba
    static final double SLOW_SCALE = 0.4;    // con L2
    static final double SPIN_POWER = 0.25;   // giro en su lugar con el D-pad

    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, RobotMap.PINPOINT);
        if (pinpoint == null) {
            while (!isStopRequested()) {
                telemetry.addLine("FALLA: no hay un dispositivo \"" + RobotMap.PINPOINT + "\" en la configuración.");
                telemetry.addLine("Debe ir en Expansion Hub, I2C bus 0, tipo goBILDA Pinpoint Odometry Computer.");
                telemetry.update();
                sleep(100);
            }
            return;
        }

        Drivetrain drivetrain = new Drivetrain(new HardwareCheck(hardwareMap));

        pinpoint.setOffsets(RobotMap.PINPOINT_X_POD_OFFSET_MM, RobotMap.PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(RobotMap.PINPOINT_X_POD_DIRECTION, RobotMap.PINPOINT_Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint encontrado. Deja el robot quieto y presiona PLAY.");
        telemetry.addData("Versión", pinpoint.getDeviceVersion());
        telemetry.addData("Offsets (mm)", "X pod %.2f, Y pod %.2f",
                pinpoint.getXOffset(DistanceUnit.MM), pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        int badReads = 0;
        int resets = 0;                     // veces que dejó de estar READY sin que se pidiera
        boolean wasReady = false;
        double turns = 0;                   // vueltas acumuladas desde el último cero
        double lastHeading = 0;
        boolean resetRequested = false;
        boolean fieldCentric = true;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                pinpoint.resetPosAndIMU();
                turns = 0;
                resetRequested = true;
            }
            if (gamepad1.bWasPressed()) {
                pinpoint.recalibrateIMU();
                resetRequested = true;
            }

            // ---------- Chasis ----------
            double scale = DRIVE_MAX * (gamepad1.left_trigger > 0.5 ? SLOW_SCALE : 1.0);
            double forward = MathUtil.shapeStick(-gamepad1.left_stick_y, DriveConstants.STICK_DEADBAND, 1) * scale;
            double left = MathUtil.shapeStick(-gamepad1.left_stick_x, DriveConstants.STICK_DEADBAND, 1) * scale;
            double turn = MathUtil.shapeStick(-gamepad1.right_stick_x, DriveConstants.STICK_DEADBAND, 1) * scale;
            if (gamepad1.dpad_left) turn = SPIN_POWER;
            if (gamepad1.dpad_right) turn = -SPIN_POWER;
            if (gamepad1.backWasPressed()) fieldCentric = !fieldCentric;

            pinpoint.update();
            boolean pinpointReady = pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
            if (fieldCentric && pinpointReady) {
                drivetrain.driveFieldCentric(forward, left, turn, pinpoint.getHeading(AngleUnit.RADIANS));
            } else {
                drivetrain.driveRobotCentric(forward, left, turn);
            }
            drivetrain.periodic();
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
            double x = pinpoint.getPosX(DistanceUnit.CM);
            double y = pinpoint.getPosY(DistanceUnit.CM);
            double heading = pinpoint.getHeading(AngleUnit.DEGREES);
            double frequency = pinpoint.getFrequency();
            if (status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ
                    || Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(heading)) {
                badReads++;
            }

            boolean ready = status == GoBildaPinpointDriver.DeviceStatus.READY;
            if (wasReady && !ready && !resetRequested) resets++;
            if (ready) resetRequested = false;
            wasReady = ready;
            turns += MathUtil.wrap180(heading - lastHeading) / 360.0;
            lastHeading = heading;

            telemetry.addData("Manejo", "%s  |  máx %.0f %%  (Share cambia; ✕ define \"adelante\")",
                    fieldCentric && ready ? "FIELD-CENTRIC" : fieldCentric ? "robot-centric (Pinpoint no listo)" : "robot-centric",
                    scale * 100);
            telemetry.addData("Estado", "%s  ->  %s", status, explain(status));
            if (resets > 0) {
                telemetry.addLine(String.format("AVISO: el Pinpoint se reinició solo %d vez/veces: le falta energía. "
                        + "Las posiciones no son confiables.", resets));
            }
            if (status == GoBildaPinpointDriver.DeviceStatus.READY
                    && (frequency < MIN_FREQUENCY_HZ || frequency > MAX_FREQUENCY_HZ)) {
                telemetry.addLine("AVISO: frecuencia interna fuera de rango; revisa el cable I2C y la alimentación");
            }
            telemetry.addData("X (adelante)", "%.1f cm   (%.2f in)", x, x / 2.54);
            telemetry.addData("Y (izquierda)", "%.1f cm   (%.2f in)", y, y / 2.54);
            telemetry.addData("Heading", "%.1f°   vueltas desde el cero: %.2f", heading, turns);
            telemetry.addData("Velocidad", "X %.0f  Y %.0f cm/s   giro %.0f °/s",
                    pinpoint.getVelX(DistanceUnit.CM), pinpoint.getVelY(DistanceUnit.CM),
                    pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Encoders crudos", "X pod %d   Y pod %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
            telemetry.addData("Pinpoint", "%.0f Hz", frequency);
            telemetry.addData("Ciclo del OpMode", "%.1f ms", loopMs);
            telemetry.addData("Lecturas malas", badReads);
            telemetry.addLine("✕: cero + recalibrar   ○: recalibrar   sticks: mover   D-pad ←/→: girar despacio");
            telemetry.update();
        }
        drivetrain.stop();
    }

    /** Qué significa cada estado y qué revisar. */
    static String explain(GoBildaPinpointDriver.DeviceStatus status) {
        switch (status) {
            case READY:                    return "OK";
            case CALIBRATING:              return "calibrando, no muevas el robot";
            case NOT_READY:                return "arrancando; si no cambia, revisa la alimentación";
            case FAULT_X_POD_NOT_DETECTED: return "FALLA: no detecta el pod X (forward). Revisa su cable";
            case FAULT_Y_POD_NOT_DETECTED: return "FALLA: no detecta el pod Y (strafe). Revisa su cable";
            case FAULT_NO_PODS_DETECTED:   return "FALLA: no detecta ningún pod. Revisa ambos cables";
            case FAULT_IMU_RUNAWAY:        return "FALLA: el giroscopio se desbocó. Recalibra con el robot quieto";
            case FAULT_BAD_READ:           return "FALLA: lectura I2C corrupta. Revisa el cable I2C";
            default:                       return "estado desconocido";
        }
    }
}
