package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotMap;

/*
 * Verifica el goBILDA Pinpoint y sus dos pods. Los motores no se usan: el robot se empuja a mano.
 *
 * gamepad1
 *   A: poner la posición en cero y recalibrar el giroscopio (robot QUIETO, tarda ~0.25 s)
 *   B: recalibrar solo el giroscopio (robot QUIETO)
 *
 * Qué debe pasar al empujar el robot a mano:
 *   hacia ADELANTE          -> X sube
 *   hacia la IZQUIERDA      -> Y sube
 *   girar ANTIHORARIO       -> heading sube
 * Si X o Y bajan, cambia la dirección de ese pod (X_POD_DIRECTION / Y_POD_DIRECTION aquí abajo)
 * y anota el valor correcto: se usará igual en el resto del código.
 *
 * Prueba de offsets: gira el robot 10 vueltas sobre su centro. X y Y deben regresar cerca de 0.
 * Si se alejan mucho, los offsets de RobotMap están mal medidos.
 */
@TeleOp(name = "Test 3: Pinpoint", group = "Test")
public class PinpointTestOpMode extends LinearOpMode {

    // Valores por confirmar en el robot con la prueba de arriba
    static final GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    static final GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    // El Pinpoint actualiza a unos 1500 Hz. Fuera de este rango, algo anda mal con el módulo.
    static final double MIN_FREQUENCY_HZ = 900;
    static final double MAX_FREQUENCY_HZ = 2000;

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

        pinpoint.setOffsets(RobotMap.PINPOINT_X_POD_OFFSET_MM, RobotMap.PINPOINT_Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(X_POD_DIRECTION, Y_POD_DIRECTION);
        pinpoint.resetPosAndIMU();

        telemetry.addLine("Pinpoint encontrado. Deja el robot quieto y presiona PLAY.");
        telemetry.addData("Versión", pinpoint.getDeviceVersion());
        telemetry.addData("Offsets (mm)", "X pod %.2f, Y pod %.2f",
                pinpoint.getXOffset(DistanceUnit.MM), pinpoint.getYOffset(DistanceUnit.MM));
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        int badReads = 0;

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) pinpoint.resetPosAndIMU();
            if (gamepad1.bWasPressed()) pinpoint.recalibrateIMU();

            pinpoint.update();
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

            telemetry.addData("Estado", "%s  ->  %s", status, explain(status));
            if (status == GoBildaPinpointDriver.DeviceStatus.READY
                    && (frequency < MIN_FREQUENCY_HZ || frequency > MAX_FREQUENCY_HZ)) {
                telemetry.addLine("AVISO: frecuencia interna fuera de rango; revisa el cable I2C y la alimentación");
            }
            telemetry.addData("X (adelante)", "%.1f cm   (%.2f in)", x, x / 2.54);
            telemetry.addData("Y (izquierda)", "%.1f cm   (%.2f in)", y, y / 2.54);
            telemetry.addData("Heading", "%.1f°", heading);
            telemetry.addData("Velocidad", "X %.0f  Y %.0f cm/s   giro %.0f °/s",
                    pinpoint.getVelX(DistanceUnit.CM), pinpoint.getVelY(DistanceUnit.CM),
                    pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));
            telemetry.addData("Encoders crudos", "X pod %d   Y pod %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());
            telemetry.addData("Pinpoint", "%.0f Hz", frequency);
            telemetry.addData("Ciclo del OpMode", "%.1f ms", loopMs);
            telemetry.addData("Lecturas malas", badReads);
            telemetry.addLine("A: cero + recalibrar   B: recalibrar giroscopio");
            telemetry.update();
        }
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
