package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/*
 * Prueba del chasis para revisar puertos y direcciones. Usar con las ruedas sin tocar el piso.
 *
 * Mantener un botón de gamepad1 mueve un solo motor. Cada rueda debe ser la indicada y girar
 * como si empujara el robot hacia adelante; si no, corrige Drivetrain:
 *   Y: frontLeft    B: frontRight
 *   X: backLeft     A: backRight
 *
 * Sticks sin botones: manejo relativo al robot, sin usar el giroscopio.
 */
@TeleOp(name = "Drive Test", group = "Test")
public class DriveTest extends LinearOpMode {

    static final double TEST_POWER = 0.3;    // potencia baja al probar un motor solo
    static final double DRIVE_SCALE = 0.5;   // velocidad reducida al manejar en esta prueba

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            drivetrain.update();

            // Elegir qué se mueve; "active" dice en la telemetría qué se está probando
            String active;
            if (gamepad1.y) {
                drivetrain.setMotorPowers(TEST_POWER, 0, 0, 0);
                active = "frontLeft";
            } else if (gamepad1.x) {
                drivetrain.setMotorPowers(0, TEST_POWER, 0, 0);
                active = "backLeft";
            } else if (gamepad1.b) {
                drivetrain.setMotorPowers(0, 0, TEST_POWER, 0);
                active = "frontRight";
            } else if (gamepad1.a) {
                drivetrain.setMotorPowers(0, 0, 0, TEST_POWER);
                active = "backRight";
            } else {
                drivetrain.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, DRIVE_SCALE);
                active = "manejo relativo al robot";
            }

            telemetry.addData("Activo", active);
            drivetrain.addTelemetry(telemetry);
            telemetry.update();
        }
    }
}
