package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * TeleOp de Froggy (LamBot 23758): chasis mecanum field-centric, doble intake y shooter de 2 flywheels.
 *
 * Configuración del Driver Station (los nombres deben coincidir exactamente):
 *   Control Hub   motor 0: frontLeft      motor 1: backLeft
 *                 motor 2: frontRight     motor 3: backRight
 *                 I2C bus 1: pinpoint  (tipo "goBILDA Pinpoint Odometry Computer")
 *   Expansion Hub motor 0: intakeFront    motor 1: intakeBack
 *                 motor 2: shooterBack (rodillo + rueda trasera)
 *                 motor 3: shooterFront (rueda delantera)
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

    // TODO: verificar en el robot. Si un motor gira al revés, cambia su dirección aquí.
    static final DcMotorSimple.Direction LEFT_DRIVE_DIRECTION   = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction RIGHT_DRIVE_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction INTAKE_FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction INTAKE_BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction SHOOTER_BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction SHOOTER_FRONT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // Ticks por vuelta del eje de salida del flywheel. Yellow Jacket 1:1 (6000 RPM) = 28.
    // Si el motor tiene caja de engranes, multiplica 28 por la relación (ej. 3.7:1 -> 103.6).
    static final double FLYWHEEL_TICKS_PER_REV = 28.0;
    static final double DEFAULT_SHOOTER_RPM = 3000;
    static final double SHOOTER_RPM_STEP = 100;
    static final double SHOOTER_RPM_TOLERANCE = 100;   // qué tan cerca del objetivo cuenta como "listo"

    static final double INTAKE_POWER = 1.0;
    static final double FEED_POWER = 1.0;
    static final double SLOW_MODE_SCALE = 0.35;
    static final double STRAFE_CORRECTION = 1.1;       // compensa que el mecanum avanza menos de lado

    DcMotor frontLeft, backLeft, frontRight, backRight;
    DcMotor intakeFront, intakeBack;
    DcMotorEx shooterBack, shooterFront;
    GoBildaPinpointDriver pinpoint;

    double headingOffset = 0;
    double shooterTargetRpm = DEFAULT_SHOOTER_RPM;
    boolean intakeOn = false;
    boolean flywheelsOn = false;

    // Estado anterior de los botones, para detectar cuándo se acaban de presionar
    boolean lastRightBumper, lastX, lastDpadUp, lastDpadDown;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Listo. Deja el robot quieto mientras calibra el Pinpoint.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();
            drive();
            boolean shooterReady = shooter();
            intake(shooterReady);
            telemetry.update();
        }
    }

    void initHardware() {
        frontLeft  = initDriveMotor("frontLeft", LEFT_DRIVE_DIRECTION);
        backLeft   = initDriveMotor("backLeft", LEFT_DRIVE_DIRECTION);
        frontRight = initDriveMotor("frontRight", RIGHT_DRIVE_DIRECTION);
        backRight  = initDriveMotor("backRight", RIGHT_DRIVE_DIRECTION);

        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");
        intakeBack  = hardwareMap.get(DcMotor.class, "intakeBack");
        intakeFront.setDirection(INTAKE_FRONT_DIRECTION);
        intakeBack.setDirection(INTAKE_BACK_DIRECTION);

        shooterBack  = initFlywheel("shooterBack", SHOOTER_BACK_DIRECTION);
        shooterFront = initFlywheel("shooterFront", SHOOTER_FRONT_DIRECTION);

        // Solo usamos el giroscopio interno del Pinpoint para el heading; los pods de odometría
        // no se configuran todavía. Calibra el giroscopio, así que el robot debe estar quieto.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
    }

    DcMotor initDriveMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        return motor;
    }

    DcMotorEx initFlywheel(String name, DcMotorSimple.Direction direction) {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // que se detenga sola, sin frenar
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return motor;
    }

    /** Mecanum field-centric: "adelante" en el stick siempre es hacia el lado contrario a los pilotos. */
    void drive() {
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        if (gamepad1.options) {
            headingOffset = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
        }
        double heading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS) - headingOffset;

        // Rotar el vector del stick al marco del robot
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        rotX *= STRAFE_CORRECTION;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double scale = gamepad1.right_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;

        frontLeft.setPower((rotY + rotX + rx) / denominator * scale);
        backLeft.setPower((rotY - rotX + rx) / denominator * scale);
        frontRight.setPower((rotY - rotX - rx) / denominator * scale);
        backRight.setPower((rotY + rotX - rx) / denominator * scale);

        telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(heading));
    }

    /** Controla los flywheels y regresa true cuando ambos están a la velocidad objetivo. */
    boolean shooter() {
        if (gamepad2.x && !lastX) flywheelsOn = !flywheelsOn;
        if (gamepad2.dpad_up && !lastDpadUp) shooterTargetRpm += SHOOTER_RPM_STEP;
        if (gamepad2.dpad_down && !lastDpadDown) shooterTargetRpm = Math.max(0, shooterTargetRpm - SHOOTER_RPM_STEP);
        lastX = gamepad2.x;
        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;

        boolean shooting = gamepad2.right_trigger > 0.5;
        boolean spinning = flywheelsOn || shooting;
        double targetTicksPerSec = spinning ? rpmToTicksPerSec(shooterTargetRpm) : 0;
        shooterBack.setVelocity(targetTicksPerSec);
        shooterFront.setVelocity(targetTicksPerSec);

        double backRpm = ticksPerSecToRpm(shooterBack.getVelocity());
        double frontRpm = ticksPerSecToRpm(shooterFront.getVelocity());
        boolean ready = spinning
                && Math.abs(backRpm - shooterTargetRpm) < SHOOTER_RPM_TOLERANCE
                && Math.abs(frontRpm - shooterTargetRpm) < SHOOTER_RPM_TOLERANCE;

        telemetry.addData("Shooter objetivo (RPM)", "%.0f", shooterTargetRpm);
        telemetry.addData("Shooter trasero / delantero (RPM)", "%.0f / %.0f", backRpm, frontRpm);
        telemetry.addData("Shooter", ready ? "LISTO" : (spinning ? "acelerando..." : "apagado"));
        return ready;
    }

    void intake(boolean shooterReady) {
        if (gamepad2.right_bumper && !lastRightBumper) intakeOn = !intakeOn;
        lastRightBumper = gamepad2.right_bumper;

        boolean shooting = gamepad2.right_trigger > 0.5;
        double power;
        String state;
        if (gamepad2.left_bumper) {
            power = -INTAKE_POWER;
            state = "reversa";
        } else if (shooting) {
            // Mientras los flywheels aceleran, las pelotas esperan para no salir lentas
            power = shooterReady ? FEED_POWER : 0;
            state = shooterReady ? "alimentando shooter" : "esperando shooter";
        } else if (intakeOn) {
            power = INTAKE_POWER;
            state = "recogiendo";
        } else {
            power = 0;
            state = "apagado";
        }
        intakeFront.setPower(power);
        intakeBack.setPower(power);
        telemetry.addData("Intake", state);
    }

    static double rpmToTicksPerSec(double rpm) {
        return rpm * FLYWHEEL_TICKS_PER_REV / 60.0;
    }

    static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / FLYWHEEL_TICKS_PER_REV;
    }
}
