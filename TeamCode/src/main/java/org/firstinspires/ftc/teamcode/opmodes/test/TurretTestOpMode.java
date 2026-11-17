package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotMap;
import org.firstinspires.ftc.teamcode.config.TurretConfig;

/*
 * Prueba de la torreta en modo continuo (CR) con el encoder analógico de turretLeft.
 *
 * ANTES DE INIT
 *   - Los dos servos de la torreta con el firmware CR y configurados como
 *     "Continuous Rotation Servo". turretEnc como "Analog Input" (Control Hub, analógico 0).
 *   - En el programador Axon, "Lose PPM Protection" en Release: así el servo que no se usa queda
 *     suelto y no frena al otro.
 *   - Pon la torreta en la marca de home. Si HOME_VOLTAGE aún no está medido, la posición que
 *     tenga al presionar INIT se toma como home.
 *
 * gamepad1
 *   Stick derecho X:        mover la torreta (derecha = horario), máximo MAX_TEST_POWER
 *   D-pad izquierda/derecha: elegir qué servo empuja: solo izquierdo / solo derecho / ambos
 *   X / Y:                  invertir turretLeft / turretRight     (con el stick suelto)
 *   A:                      invertir el encoder                   (con el stick suelto)
 *   Start:                  confirmar direcciones y desbloquear el rango completo
 *   Back:                   la posición actual pasa a ser home (y muestra su voltaje)
 *   B:                      reiniciar el voltaje mínimo y máximo
 *
 * SEGURIDAD
 *   - Hasta presionar Start, la torreta solo se mueve ±30° alrededor de home.
 *   - Con el rango completo, se detiene 5° antes de cada límite (-40° y +360°).
 *   - Si con potencia positiva el ángulo baja (o al revés), la potencia se corta: alguna dirección
 *     está mal. Se libera al cambiar una dirección (X, Y o A) o con Back.
 *   - Cambiar una dirección vuelve a bloquear el rango completo.
 *
 * PROCEDIMIENTO (anota los resultados en la hoja de CHECKLIST.md)
 *   1. Modo "solo izquierdo". Stick un poco a la izquierda: la torreta debe girar ANTIHORARIO
 *      (vista desde arriba). Si gira horario, presiona X.
 *   2. Mismo movimiento: el ángulo debe SUBIR. Si baja, presiona A.
 *   3. Modo "solo derecho". Stick a la izquierda: debe girar antihorario. Si no, presiona Y.
 *   4. Modo "ambos". Mueve despacio a los dos lados: debe girar suave, sin tirones ni ruido.
 *      Si todo está bien, presiona Start.
 *   5. Gira una vuelta completa del servo (180° de torreta) y anota V mín y V máx.
 *   6. Lleva la torreta a la marca de home, presiona Back y anota HOME_VOLTAGE.
 *   7. Recorre el rango completo despacio y compara los límites con las marcas físicas.
 *   Copia los valores a TurretConfig.java.
 */
@TeleOp(name = "Test 5: Torreta", group = "Test")
public class TurretTestOpMode extends LinearOpMode {

    static final double MAX_TEST_POWER = 0.3;
    static final double STICK_DEADBAND = 0.05;
    static final double SAFE_RANGE_DEG = 30;      // rango permitido antes de confirmar direcciones
    static final double MISMATCH_DEG = 8;         // movimiento al revés que cuenta como falla
    static final double STALL_SECONDS = 0.7;      // con potencia y sin moverse este tiempo -> aviso
    static final double STALL_DEG = 2;
    static final double DRIFT_DEG = 3;            // sin potencia y se movió esto -> aviso

    enum Drive {
        LEFT_ONLY("solo turretLeft (turretRight suelto)"),
        RIGHT_ONLY("solo turretRight (turretLeft suelto)"),
        BOTH("AMBOS servos");

        final String label;
        Drive(String label) { this.label = label; }
    }

    CRServoImplEx left, right;
    AnalogInput encoder;

    // Direcciones: empiezan con lo que diga TurretConfig y se pueden cambiar en vivo
    boolean leftReversed = TurretConfig.LEFT_REVERSED;
    boolean rightReversed = TurretConfig.RIGHT_REVERSED;
    boolean encoderReversed = TurretConfig.ENCODER_REVERSED;

    Drive drive = Drive.LEFT_ONLY;   // se empieza con un solo servo: si algo está mal, no pelean
    boolean confirmed = false;
    boolean fault = false;

    // Encoder: grados de servo acumulados (sin invertir), con 0 = home
    double servoDegFromHome;
    double lastRawServoDeg;
    double homeVoltage;
    double vMin = Double.MAX_VALUE, vMax = -Double.MAX_VALUE;

    // Seguimiento de cada tramo con potencia en un mismo sentido
    int lastSign = 0;
    double segmentStartDeg;
    final ElapsedTime segmentTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        left = hardwareMap.tryGet(CRServoImplEx.class, RobotMap.TURRET_LEFT);
        right = hardwareMap.tryGet(CRServoImplEx.class, RobotMap.TURRET_RIGHT);
        encoder = hardwareMap.tryGet(AnalogInput.class, RobotMap.TURRET_ENCODER);

        if (left == null || right == null || encoder == null) {
            while (!isStopRequested()) {
                telemetry.addLine("FALLA: falta hardware en la configuración:");
                if (left == null) telemetry.addLine("  " + RobotMap.TURRET_LEFT + " (Continuous Rotation Servo, Control Hub servo 4)");
                if (right == null) telemetry.addLine("  " + RobotMap.TURRET_RIGHT + " (Continuous Rotation Servo, Expansion Hub servo 1)");
                if (encoder == null) telemetry.addLine("  " + RobotMap.TURRET_ENCODER + " (Analog Input, Control Hub analógico 0)");
                telemetry.addLine("Si el servo existe pero como tipo \"Servo\", cámbialo a Continuous Rotation Servo.");
                telemetry.update();
                sleep(100);
            }
            return;
        }

        PwmControl.PwmRange crRange = new PwmControl.PwmRange(TurretConfig.CR_PWM_MIN_US, TurretConfig.CR_PWM_MAX_US);
        left.setPwmRange(crRange);
        right.setPwmRange(crRange);
        left.setPwmDisable();
        right.setPwmDisable();

        // Ángulo inicial: se asume que la torreta está a menos de 90° de home
        double v0 = encoder.getVoltage();
        homeVoltage = TurretConfig.HOME_VOLTAGE >= 0 ? TurretConfig.HOME_VOLTAGE : v0;
        lastRawServoDeg = rawServoDeg(v0);
        servoDegFromHome = wrap180(lastRawServoDeg - rawServoDeg(homeVoltage));

        telemetry.addLine("Torreta suelta. Manos lejos al presionar PLAY.");
        telemetry.addData("Home", TurretConfig.HOME_VOLTAGE >= 0
                ? String.format("HOME_VOLTAGE = %.3f V (de TurretConfig)", homeVoltage)
                : String.format("sin medir: se usa la posición actual (%.3f V)", homeVoltage));
        telemetry.addData("Ángulo inicial", "%.1f°", turretDeg());
        telemetry.update();
        waitForStart();

        applyDriveMode();
        while (opModeIsActive()) {
            readEncoder();
            double angle = turretDeg();
            boolean stickReleased = Math.abs(gamepad1.right_stick_x) < STICK_DEADBAND;

            handleButtons(stickReleased);

            // Stick a la derecha = horario = ángulo negativo
            double power = stickReleased ? 0 : -gamepad1.right_stick_x * MAX_TEST_POWER;
            double low = confirmed ? TurretConfig.MIN_DEG + TurretConfig.LIMIT_MARGIN_DEG : -SAFE_RANGE_DEG;
            double high = confirmed ? TurretConfig.MAX_DEG - TurretConfig.LIMIT_MARGIN_DEG : SAFE_RANGE_DEG;
            boolean atLimit = (angle >= high && power > 0) || (angle <= low && power < 0);
            if (atLimit || fault) power = 0;

            // Cada vez que cambia el sentido de la potencia empieza un tramo nuevo
            int sign = (int) Math.signum(power);
            if (sign != lastSign) {
                lastSign = sign;
                segmentStartDeg = angle;
                segmentTimer.reset();
            }
            double moved = angle - segmentStartDeg;
            if (sign != 0 && sign * moved < -MISMATCH_DEG) {
                fault = true;
                power = 0;
            }

            setPower(power);

            // ---------- Telemetría ----------
            telemetry.addData("Empuja", drive.label);
            telemetry.addData("Rango", confirmed
                    ? String.format("COMPLETO %.0f° a %.0f°", TurretConfig.MIN_DEG, TurretConfig.MAX_DEG)
                    : String.format("SEGURO ±%.0f° (Start para desbloquear)", SAFE_RANGE_DEG));
            if (fault) {
                telemetry.addLine("FALLA: la torreta se movió al revés de lo que se pidió. Potencia cortada.");
                telemetry.addLine("  Si GIRÓ al revés: invierte el servo (X o Y). Si el ÁNGULO va al revés: A.");
            } else if (atLimit) {
                telemetry.addLine("En el límite: solo se puede mover hacia el otro lado.");
            } else if (sign != 0 && Math.abs(moved) < STALL_DEG && segmentTimer.seconds() > STALL_SECONDS) {
                telemetry.addLine("AVISO: hay potencia pero el ángulo no cambia. ¿Firmware CR? ¿Cable del encoder?");
            } else if (sign == 0 && Math.abs(moved) > DRIFT_DEG) {
                telemetry.addLine("AVISO: la torreta se mueve sola sin potencia. Ajusta Neutral en el programador.");
            }
            telemetry.addData("Potencia", "%.2f", power);
            telemetry.addData("Ángulo torreta", "%.1f°", angle);
            telemetry.addData("Servo", "%.1f° de la vuelta  (%.2f vueltas desde home)",
                    lastRawServoDeg, servoDegFromHome / 360.0);
            telemetry.addData("Voltaje", "%.3f V   (mín %.3f / máx %.3f)", encoder.getVoltage(), vMin, vMax);
            telemetry.addData("Home", "%.3f V", homeVoltage);
            telemetry.addData("Direcciones", "izq %s | der %s | encoder %s",
                    leftReversed ? "REVERSA" : "normal", rightReversed ? "REVERSA" : "normal",
                    encoderReversed ? "REVERSA" : "normal");
            telemetry.addLine("Stick der: mover | D-pad: servo | X/Y/A: invertir | Start: confirmar | Back: home");
            telemetry.update();
        }
        setPower(0);
        left.setPwmDisable();
        right.setPwmDisable();
    }

    void handleButtons(boolean stickReleased) {
        // Los "WasPressed" se leen siempre para que un toque con el stick movido no quede guardado
        boolean modeNext = gamepad1.dpadRightWasPressed();
        boolean modePrev = gamepad1.dpadLeftWasPressed();
        boolean flipLeft = gamepad1.xWasPressed();
        boolean flipRight = gamepad1.yWasPressed();
        boolean flipEncoder = gamepad1.aWasPressed();
        boolean confirm = gamepad1.startWasPressed();
        boolean rehome = gamepad1.backWasPressed();
        if (gamepad1.bWasPressed()) {
            vMin = Double.MAX_VALUE;
            vMax = -Double.MAX_VALUE;
        }
        if (!stickReleased) return;   // nada cambia mientras la torreta se mueve

        if (modeNext || modePrev) {
            Drive[] modes = Drive.values();
            int step = modeNext ? 1 : modes.length - 1;
            drive = modes[(drive.ordinal() + step) % modes.length];
            applyDriveMode();
        }
        if (flipLeft) { leftReversed = !leftReversed; directionsChanged(); }
        if (flipRight) { rightReversed = !rightReversed; directionsChanged(); }
        if (flipEncoder) { encoderReversed = !encoderReversed; directionsChanged(); }
        if (confirm && !fault) confirmed = true;
        if (rehome) {
            homeVoltage = encoder.getVoltage();
            servoDegFromHome = 0;
            fault = false;
            lastSign = 0;
            telemetry.log().add("HOME_VOLTAGE = %.3f  (cópialo a TurretConfig)", homeVoltage);
        }
    }

    void directionsChanged() {
        fault = false;
        confirmed = false;
        lastSign = 0;
    }

    /** El servo que no se usa queda sin señal, y con "Release" queda suelto. */
    void applyDriveMode() {
        setPower(0);
        if (drive == Drive.RIGHT_ONLY) left.setPwmDisable(); else left.setPwmEnable();
        if (drive == Drive.LEFT_ONLY) right.setPwmDisable(); else right.setPwmEnable();
    }

    /** Potencia positiva = antihorario, ya con la dirección de cada servo aplicada. */
    void setPower(double power) {
        if (drive != Drive.RIGHT_ONLY) left.setPower(leftReversed ? -power : power);
        if (drive != Drive.LEFT_ONLY) right.setPower(rightReversed ? -power : power);
    }

    /** Lee el encoder y suma lo que giró el servo, contando las vueltas al pasar de 360° a 0°. */
    void readEncoder() {
        double v = encoder.getVoltage();
        vMin = Math.min(vMin, v);
        vMax = Math.max(vMax, v);
        double raw = rawServoDeg(v);
        servoDegFromHome += wrap180(raw - lastRawServoDeg);
        lastRawServoDeg = raw;
    }

    /** Ángulo de la torreta desde home. Positivo = antihorario (si las direcciones están bien). */
    double turretDeg() {
        double sign = encoderReversed ? -1 : 1;
        return sign * servoDegFromHome / TurretConfig.GEAR_RATIO;
    }

    /** Ángulo del servo dentro de su vuelta, 0 a 360, según el voltaje. */
    static double rawServoDeg(double volts) {
        double v = Range.clip(volts, TurretConfig.ENCODER_V_MIN, TurretConfig.ENCODER_V_MAX);
        return (v - TurretConfig.ENCODER_V_MIN) / (TurretConfig.ENCODER_V_MAX - TurretConfig.ENCODER_V_MIN) * 360.0;
    }

    /** Lleva un ángulo al rango -180 a 180. */
    static double wrap180(double degrees) {
        double d = (degrees + 180.0) % 360.0;
        if (d < 0) d += 360.0;
        return d - 180.0;
    }
}
