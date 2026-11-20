package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotMap;
import org.firstinspires.ftc.teamcode.config.TurretConfig;

/*
 * Prueba de la torreta en modo continuo (CR) con los dos encoders analógicos (4.º cable de cada
 * servo). El ángulo y los límites salen del encoder izquierdo; el derecho se compara con él.
 * Si solo uno está conectado, la prueba funciona con ese.
 *
 * ANTES DE INIT
 *   - Los dos servos de la torreta con el firmware CR y configurados como
 *     "Continuous Rotation Servo".
 *   - turretEncLeft:  Analog Input, Control Hub analógico 0.
 *     turretEncRight: Analog Input, Expansion Hub analógico 2.
 *   - En el programador Axon, "Lose PPM Protection" en Release: así el servo que no se usa queda
 *     suelto y no frena al otro.
 *   - Pon la torreta en la marca de home. Si un HOME_VOLTAGE aún no está medido, la posición que
 *     tenga al presionar INIT se toma como home.
 *
 * gamepad1
 *   Stick derecho X:         mover la torreta (derecha = horario), máximo MAX_TEST_POWER
 *   D-pad izquierda/derecha: elegir qué servo empuja: solo izquierdo / solo derecho / ambos
 *   X / Y:                   invertir servo turretLeft / turretRight   (con el stick suelto)
 *   A / B:                   invertir encoder izquierdo / derecho      (con el stick suelto)
 *   Start:                   confirmar direcciones y desbloquear el rango completo
 *   Back:                    la posición actual pasa a ser home (y muestra los voltajes)
 *   Bumper derecho:          reiniciar los voltajes mínimo y máximo
 *
 * SEGURIDAD
 *   - Hasta presionar Start, la torreta solo se mueve ±30° alrededor de home.
 *   - Con el rango completo, se detiene 5° antes de cada límite (-40° y +360°).
 *   - Si con potencia positiva el ángulo baja (o al revés), la potencia se corta: alguna dirección
 *     está mal. Se libera al cambiar una dirección (X, Y, A o B) o con Back.
 *   - Cambiar una dirección vuelve a bloquear el rango completo.
 *
 * CALIBRACIÓN AUTOMÁTICA (recomendada): torreta en home al presionar INIT, PLAY, y L1 con el
 * stick suelto.
 *   1. El servo izquierdo empuja un poco y pregunta: ¿la torreta se fue hacia ATRÁS del robot?
 *      (△ sí / ○ no / ✕ repetir). Es la única pregunta: de ahí salen los dos encoders.
 *   2. El servo derecho empuja un poco y se compara solo.
 *   3. Gira 200° y regresa para medir el voltaje mínimo y máximo de cada encoder.
 *   Las líneas "Para TurretConfig" muestran todos los valores a copiar. Home = la posición al
 *   presionar INIT. Luego revisa a mano con el stick y sigue con el paso 8.
 *
 * PROCEDIMIENTO (anota los resultados en la hoja de CHECKLIST.md)
 *   1. Modo "solo izquierdo". Stick un poco a la izquierda: la torreta debe girar ANTIHORARIO
 *      (vista desde arriba). Si gira horario, presiona X.
 *   2. Mismo movimiento: el ángulo del encoder izquierdo debe SUBIR. Si baja, presiona A.
 *   3. Mismo movimiento: el ángulo del encoder derecho también debe SUBIR. Si baja, presiona B.
 *   4. Modo "solo derecho". Stick a la izquierda: debe girar antihorario. Si no, presiona Y.
 *   5. Modo "ambos". Mueve despacio a los dos lados: debe girar suave, sin tirones ni ruido, y los
 *      dos encoders deben coincidir. Si todo está bien, presiona Start.
 *   6. Gira una vuelta completa del servo (~128° de torreta) y anota V mín y V máx de cada encoder.
 *   7. Lleva la torreta a la marca de home, presiona Back y anota los dos HOME_VOLTAGE.
 *   8. Recorre el rango completo despacio y compara los límites con las marcas físicas.
 *   Copia los valores a TurretConfig.java.
 */
@TeleOp(name = "Test 5: Torreta", group = "Test")
public class TurretTestOpMode extends LinearOpMode {

    static final double MAX_TEST_POWER = 0.6;
    static final double STICK_DEADBAND = 0.05;
    static final double SAFE_RANGE_DEG = 30;      // rango permitido antes de confirmar direcciones
    static final double MISMATCH_DEG = 8;         // movimiento al revés que cuenta como falla
    static final double STALL_SECONDS = 0.7;      // con potencia y sin moverse este tiempo -> aviso
    static final double STALL_DEG = 2;
    static final double DRIFT_DEG = 3;            // sin potencia y se movió esto -> aviso
    static final double DETECT_POWER = 0.25;      // calibración automática: empujón de cada servo
    static final double DETECT_SECONDS = 0.4;
    static final double DETECT_MIN_DEG = 3;
    static final double SWEEP_DEG = 200;          // más de 180° = una vuelta completa de servo
    static final double SWEEP_POWER = 0.35;
    static final double SWEEP_MAX_BOOST = 0.35;   // potencia extra si deja de avanzar (máximo 0.70)
    static final double SWEEP_TIMEOUT_S = 12;
    static final long ENCODER_WAKE_MS = 300;       // tiempo para que el Axon empiece a mandar su posición

    enum Drive {
        LEFT_ONLY("solo turretLeft (turretRight suelto)"),
        RIGHT_ONLY("solo turretRight (turretLeft suelto)"),
        BOTH("AMBOS servos");

        final String label;
        Drive(String label) { this.label = label; }
    }

    /**
     * Un encoder analógico de servo. Cuenta las vueltas del servo desde home y las convierte a
     * grados de torreta.
     */
    static class TurretEncoder {
        final String label;
        final AnalogInput input;
        final double vMinConfig, vMaxConfig;
        boolean reversed;
        double homeVoltage;
        double servoDegFromHome;   // grados de servo acumulados desde home, sin invertir
        double lastRawDeg;
        double vMin = Double.MAX_VALUE, vMax = -Double.MAX_VALUE;

        TurretEncoder(String label, AnalogInput input, boolean reversed,
                      double vMinConfig, double vMaxConfig, double homeVoltageConfig) {
            this.label = label;
            this.input = input;
            this.reversed = reversed;
            this.vMinConfig = vMinConfig;
            this.vMaxConfig = vMaxConfig;
            // Ángulo inicial: se asume que la torreta está cerca de home (menos de media vuelta de servo)
            double v = input.getVoltage();
            homeVoltage = homeVoltageConfig >= 0 ? homeVoltageConfig : v;
            lastRawDeg = rawDeg(v);
            servoDegFromHome = wrap180(lastRawDeg - rawDeg(homeVoltage));
        }

        /** Lee el voltaje y suma lo que giró el servo, contando las vueltas al pasar de 360° a 0°. */
        void read() {
            double v = input.getVoltage();
            vMin = Math.min(vMin, v);
            vMax = Math.max(vMax, v);
            double raw = rawDeg(v);
            servoDegFromHome += wrap180(raw - lastRawDeg);
            lastRawDeg = raw;
        }

        /** Grados de torreta desde home. Positivo = antihorario (si la dirección está bien). */
        double turretDeg() {
            return (reversed ? -1 : 1) * servoDegFromHome / TurretConfig.GEAR_RATIO;
        }

        void rehome() {
            homeVoltage = input.getVoltage();
            lastRawDeg = rawDeg(homeVoltage);
            servoDegFromHome = 0;
        }

        void resetVoltageRange() {
            vMin = Double.MAX_VALUE;
            vMax = -Double.MAX_VALUE;
        }

        /** Ángulo del servo dentro de su vuelta, 0 a 360, según el voltaje. */
        double rawDeg(double volts) {
            double v = Range.clip(volts, vMinConfig, vMaxConfig);
            return (v - vMinConfig) / (vMaxConfig - vMinConfig) * 360.0;
        }

        void addTelemetry(Telemetry telemetry) {
            telemetry.addData("Encoder " + label, "%.1f°  |  %.3f V (mín %.3f / máx %.3f)  home %.3f V  %s",
                    turretDeg(), input.getVoltage(), vMin, vMax, homeVoltage, reversed ? "REVERSA" : "normal");
        }
    }

    CRServoImplEx left, right;
    TurretEncoder encoderLeft, encoderRight;   // cualquiera puede ser null si no está configurado
    TurretEncoder primary;                     // el que manda: el izquierdo si existe

    // Direcciones de los servos: empiezan con lo que diga TurretConfig y se cambian en vivo
    boolean leftReversed = TurretConfig.LEFT_REVERSED;
    boolean rightReversed = TurretConfig.RIGHT_REVERSED;

    Drive drive = Drive.LEFT_ONLY;   // se empieza con un solo servo: si algo está mal, no pelean
    boolean confirmed = false;
    String calibrationResult = "";
    boolean fault = false;

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
        AnalogInput inputLeft = hardwareMap.tryGet(AnalogInput.class, RobotMap.TURRET_ENCODER_LEFT);
        AnalogInput inputRight = hardwareMap.tryGet(AnalogInput.class, RobotMap.TURRET_ENCODER_RIGHT);

        if (left == null || right == null || (inputLeft == null && inputRight == null)) {
            while (!isStopRequested()) {
                telemetry.addLine("FALLA: falta hardware en la configuración:");
                if (left == null) telemetry.addLine("  " + RobotMap.TURRET_LEFT + " (Continuous Rotation Servo, Control Hub servo 4)");
                if (right == null) telemetry.addLine("  " + RobotMap.TURRET_RIGHT + " (Continuous Rotation Servo, Expansion Hub servo 1)");
                if (inputLeft == null) telemetry.addLine("  " + RobotMap.TURRET_ENCODER_LEFT + " (Analog Input, Control Hub analógico 0)");
                if (inputRight == null) telemetry.addLine("  " + RobotMap.TURRET_ENCODER_RIGHT + " (Analog Input, Expansion Hub analógico 2)");
                telemetry.addLine("Si un servo existe pero como tipo \"Servo\", cámbialo a Continuous Rotation Servo.");
                telemetry.update();
                sleep(100);
            }
            return;
        }

        PwmControl.PwmRange crRange = new PwmControl.PwmRange(TurretConfig.CR_PWM_MIN_US, TurretConfig.CR_PWM_MAX_US);
        left.setPwmRange(crRange);
        right.setPwmRange(crRange);
        // El Axon solo manda su posición (4.º cable) mientras recibe pulso: sin señal da 0 V.
        // Se le manda "detenido" (potencia 0) y se espera a que la lectura se estabilice antes de
        // tomar home. La torreta no se mueve.
        left.setPower(0);
        right.setPower(0);
        left.setPwmEnable();
        right.setPwmEnable();
        sleep(ENCODER_WAKE_MS);

        if (inputLeft != null) {
            encoderLeft = new TurretEncoder("izq", inputLeft, TurretConfig.LEFT_ENCODER_REVERSED,
                    TurretConfig.LEFT_ENCODER_V_MIN, TurretConfig.LEFT_ENCODER_V_MAX, TurretConfig.LEFT_HOME_VOLTAGE);
        }
        if (inputRight != null) {
            encoderRight = new TurretEncoder("der", inputRight, TurretConfig.RIGHT_ENCODER_REVERSED,
                    TurretConfig.RIGHT_ENCODER_V_MIN, TurretConfig.RIGHT_ENCODER_V_MAX, TurretConfig.RIGHT_HOME_VOLTAGE);
        }
        primary = encoderLeft != null ? encoderLeft : encoderRight;

        // PASO A (antes de PLAY, servos sueltos): revisar los encoders girando la torreta a mano.
        while (opModeInInit()) {
            if (encoderLeft != null) encoderLeft.read();
            if (encoderRight != null) encoderRight.read();
            if (gamepad1.aWasPressed() && encoderLeft != null) encoderLeft.reversed = !encoderLeft.reversed;
            if (gamepad1.bWasPressed() && encoderRight != null) encoderRight.reversed = !encoderRight.reversed;

            telemetry.addLine("Pon la torreta en la marca de HOME (apuntando a la IZQUIERDA del robot).");
            telemetry.addLine("Presiona PLAY y después L1: CALIBRACIÓN AUTOMÁTICA.");
            telemetry.addLine("(Solo te hará una pregunta. Manos lejos de la torreta.)");
            if (encoderLeft != null) telemetry.addData("Encoder izq", "%.1f°  %s", encoderLeft.turretDeg(), encoderLeft.reversed ? "(REVERSA)" : "");
            if (encoderRight != null) telemetry.addData("Encoder der", "%.1f°  %s", encoderRight.turretDeg(), encoderRight.reversed ? "(REVERSA)" : "");
            if (encoderLeft == null) telemetry.addLine("AVISO: falta turretEncLeft; se usa solo el derecho.");
            if (encoderRight == null) telemetry.addLine("AVISO: falta turretEncRight; no habrá comparación.");
            addConfigLine();
            telemetry.update();
        }
        if (isStopRequested()) return;

        applyDriveMode();
        while (opModeIsActive()) {
            if (encoderLeft != null) encoderLeft.read();
            if (encoderRight != null) encoderRight.read();
            double angle = primary.turretDeg();
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
                telemetry.addLine("  Si GIRÓ al revés: invierte el servo (X o Y). Si el ÁNGULO va al revés: encoder (A o B).");
            } else if (atLimit) {
                telemetry.addLine("En el límite: solo se puede mover hacia el otro lado.");
            } else if (sign != 0 && Math.abs(moved) < STALL_DEG && segmentTimer.seconds() > STALL_SECONDS) {
                telemetry.addLine("AVISO: hay potencia pero el ángulo no cambia. ¿Firmware CR? ¿Cable del encoder?");
            } else if (sign == 0 && Math.abs(moved) > DRIFT_DEG) {
                telemetry.addLine("AVISO: la torreta se mueve sola sin potencia. Ajusta Neutral en el programador.");
            }
            if (encoderLeft != null && encoderRight != null) {
                double difference = encoderRight.turretDeg() - encoderLeft.turretDeg();
                telemetry.addData("Diferencia entre encoders", "%.1f°", difference);
                if (Math.abs(difference) > TurretConfig.ENCODER_MISMATCH_DEG) {
                    telemetry.addLine("AVISO: los encoders no coinciden. Si uno va al revés, inviértelo (A o B);"
                            + " si la diferencia crece sola, un engrane patina o un cable falla.");
                }
            }
            telemetry.addData("Potencia", "%.2f", power);
            telemetry.addData("Ángulo torreta", "%.1f°  (encoder %s)", angle, primary.label);
            if (encoderLeft != null) encoderLeft.addTelemetry(telemetry);
            if (encoderRight != null) encoderRight.addTelemetry(telemetry);
            telemetry.addData("Servos", "izq %s | der %s",
                    leftReversed ? "REVERSA" : "normal", rightReversed ? "REVERSA" : "normal");
            addConfigLine();
            telemetry.addLine("Stick der: mover | L1: detectar servos | D-pad: servo | □/△: servos | ✕/○: encoders | Options: confirmar | Share: home");
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
        boolean flipEncoderLeft = gamepad1.aWasPressed();
        boolean flipEncoderRight = gamepad1.bWasPressed();
        boolean confirm = gamepad1.startWasPressed();
        boolean rehome = gamepad1.backWasPressed();
        boolean detect = gamepad1.leftBumperWasPressed();
        if (gamepad1.rightBumperWasPressed()) {
            if (encoderLeft != null) encoderLeft.resetVoltageRange();
            if (encoderRight != null) encoderRight.resetVoltageRange();
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
        if (flipEncoderLeft && encoderLeft != null) { encoderLeft.reversed = !encoderLeft.reversed; directionsChanged(); }
        if (flipEncoderRight && encoderRight != null) { encoderRight.reversed = !encoderRight.reversed; directionsChanged(); }
        if (detect) autoCalibrate();
        if (confirm && !fault) confirmed = true;
        if (rehome) {
            if (encoderLeft != null) encoderLeft.rehome();
            if (encoderRight != null) encoderRight.rehome();
            fault = false;
            lastSign = 0;
            telemetry.log().add("LEFT_HOME_VOLTAGE = %s   RIGHT_HOME_VOLTAGE = %s  (cópialos a TurretConfig)",
                    encoderLeft != null ? String.format("%.3f", encoderLeft.homeVoltage) : "-",
                    encoderRight != null ? String.format("%.3f", encoderRight.homeVoltage) : "-");
        }
    }

    // ===================== Calibración automática (L1) =====================

    /**
     * Saca sola las 4 direcciones y el voltaje mínimo/máximo de cada encoder. Hace UNA sola
     * pregunta: hacia dónde se movió la torreta en el primer empujón, porque es la única forma de
     * saber el sentido real. Todo lo demás se compara contra eso.
     *
     * Debe empezar con la torreta en home (apuntando a la izquierda del robot). Antihorario visto
     * desde arriba = desde la izquierda hacia ATRÁS del robot.
     */
    void autoCalibrate() {
        if (left == null || right == null) {
            calibrationResult = "faltan servos en la configuración";
            return;
        }
        TurretEncoder other = primary == encoderLeft ? encoderRight : encoderLeft;
        primary.reversed = false;                 // se mide en crudo
        if (other != null) other.reversed = false;
        leftReversed = false;
        rightReversed = false;

        // 1. Empujón con el servo izquierdo, potencia positiva sin invertir
        double primaryStart = rawDeg(primary);
        double otherStart = other != null ? rawDeg(other) : 0;
        pulse(left, DETECT_POWER);
        double primaryDelta = rawDeg(primary) - primaryStart;
        double otherDelta = other != null ? rawDeg(other) - otherStart : 0;
        if (Math.abs(primaryDelta) < DETECT_MIN_DEG) {
            pulse(left, -DETECT_POWER);
            calibrationResult = "FALLA: el servo izquierdo no movió la torreta (¿firmware CR? ¿cable del encoder?)";
            directionsChanged();
            return;
        }

        // 2. La única pregunta
        Boolean towardRear = askTowardRear();
        pulse(left, -DETECT_POWER);   // regresa a home
        if (towardRear == null) return;

        boolean leftPlusIsCcw = towardRear;
        leftReversed = !leftPlusIsCcw;   // potencia positiva debe ser antihorario
        double ccwPrimarySign = leftPlusIsCcw ? Math.signum(primaryDelta) : -Math.signum(primaryDelta);
        primary.reversed = ccwPrimarySign < 0;   // antihorario debe hacer subir el ángulo
        if (other != null && Math.abs(otherDelta) >= DETECT_MIN_DEG) {
            double ccwOtherSign = leftPlusIsCcw ? Math.signum(otherDelta) : -Math.signum(otherDelta);
            other.reversed = ccwOtherSign < 0;
        }

        // 3. Empujón con el servo derecho: se compara con el encoder principal (ya sin dudas)
        double rightStart = rawDeg(primary);
        pulse(right, DETECT_POWER);
        double rightDelta = rawDeg(primary) - rightStart;
        pulse(right, -DETECT_POWER);
        boolean rightOk = Math.abs(rightDelta) >= DETECT_MIN_DEG;
        if (rightOk) rightReversed = Math.signum(rightDelta) != ccwPrimarySign;

        // 4. Barrido de más de una vuelta de servo (~128° de torreta) para medir voltajes, y regreso
        directionsChanged();
        drive = Drive.BOTH;
        applyDriveMode();
        if (encoderLeft != null) encoderLeft.resetVoltageRange();
        if (encoderRight != null) encoderRight.resetVoltageRange();
        boolean swept = moveTo(SWEEP_DEG) && moveTo(0);
        setPower(0);

        calibrationResult = (rightOk ? "" : "AVISO: el servo derecho no movió la torreta. ")
                + (swept ? "LISTO: revisa que gire bien y manda foto de las líneas \"Para TurretConfig\"."
                : "El barrido se detuvo (ver arriba). Las direcciones sí quedaron.");
        telemetry.log().add("Calibración: %s", calibrationResult);
    }

    /** Grados de servo acumulados, sin inversión (después de leer los encoders). */
    double rawDeg(TurretEncoder encoder) {
        readEncoders();
        return encoder.servoDegFromHome;
    }

    void readEncoders() {
        if (encoderLeft != null) encoderLeft.read();
        if (encoderRight != null) encoderRight.read();
    }

    /** Un servo solo, con potencia directa (sin inversión) un momento; el otro queda suelto. */
    void pulse(CRServoImplEx servo, double power) {
        CRServoImplEx other = servo == left ? right : left;
        other.setPower(0);
        other.setPwmEnable();   // con señal (detenido) para que su encoder siga leyendo
        servo.setPwmEnable();
        ElapsedTime timer = new ElapsedTime();
        servo.setPower(power);
        while (opModeIsActive() && timer.seconds() < DETECT_SECONDS) {
            readEncoders();
            idle();
        }
        servo.setPower(0);
    }

    /** true = se fue hacia atrás del robot, false = hacia el frente, null = se detuvo el OpMode. */
    Boolean askTowardRear() {
        while (opModeIsActive()) {
            if (gamepad1.yWasPressed()) return true;
            if (gamepad1.bWasPressed()) return false;
            if (gamepad1.aWasPressed()) {          // repetir el movimiento para verlo otra vez
                pulse(left, -DETECT_POWER);
                pulse(left, DETECT_POWER);
            }
            readEncoders();
            telemetry.addLine("CALIBRACIÓN: la torreta se acaba de mover un poco.");
            telemetry.addLine("Home apunta a la IZQUIERDA del robot.");
            telemetry.addLine("¿Se movió hacia la parte de ATRÁS del robot?");
            telemetry.addLine("   △ = sí, hacia ATRÁS");
            telemetry.addLine("   ○ = no, hacia el FRENTE");
            telemetry.addLine("   ✕ = repetir el movimiento");
            telemetry.update();
        }
        return null;
    }

    /** Lleva la torreta a un ángulo, despacio, cuidando límites y atorones. */
    boolean moveTo(double targetDeg) {
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime stallTimer = new ElapsedTime();
        double lastAngle = primary.turretDeg();
        double boost = 0;
        while (opModeIsActive()) {
            readEncoders();
            double angle = primary.turretDeg();
            double error = targetDeg - angle;
            if (Math.abs(error) < 3) {
                setPower(0);
                return true;
            }
            if (angle > SWEEP_DEG + 15 || angle < TurretConfig.MIN_DEG + TurretConfig.LIMIT_MARGIN_DEG) {
                setPower(0);
                telemetry.log().add("Barrido detenido: la torreta salió del rango esperado (%.0f°). ¿Respuesta al revés?", angle);
                return false;
            }
            if (timeout.seconds() > SWEEP_TIMEOUT_S) {
                setPower(0);
                telemetry.log().add("Barrido detenido: tardó demasiado.");
                return false;
            }
            // Si deja de avanzar, se le da más potencia poco a poco; solo se rinde a potencia máxima
            if (Math.abs(angle - lastAngle) > 1) {
                lastAngle = angle;
                stallTimer.reset();
            } else if (stallTimer.seconds() > 0.4 && boost < SWEEP_MAX_BOOST) {
                boost = Math.min(SWEEP_MAX_BOOST, boost + 0.1);
                stallTimer.reset();
            } else if (stallTimer.seconds() > 1.0) {
                setPower(0);
                telemetry.log().add("Barrido detenido: no avanza ni con potencia %.2f (¿atorada o la cadena?).", SWEEP_POWER + boost);
                return false;
            }
            double power = Math.min(SWEEP_POWER, 0.12 + Math.abs(error) * 0.01) + boost;
            setPower(Math.copySign(power, error));
            telemetry.addData("Calibración", "barrido: %.0f° → %.0f°  potencia %.2f  (no toques la torreta)",
                    angle, targetDeg, power);
            if (encoderLeft != null) encoderLeft.addTelemetry(telemetry);
            if (encoderRight != null) encoderRight.addTelemetry(telemetry);
            telemetry.update();
        }
        setPower(0);
        return false;
    }

    /** Las líneas que se copian a TurretConfig. */
    void addConfigLine() {
        telemetry.addData("Para TurretConfig", "LEFT_REVERSED=%b  RIGHT_REVERSED=%b  LEFT_ENCODER_REVERSED=%s  RIGHT_ENCODER_REVERSED=%s",
                leftReversed, rightReversed,
                encoderLeft != null ? String.valueOf(encoderLeft.reversed) : "-",
                encoderRight != null ? String.valueOf(encoderRight.reversed) : "-");
        telemetry.addData("Para TurretConfig (voltajes)", "izq mín %s máx %s home %.3f  |  der mín %s máx %s home %s",
                voltage(encoderLeft, true), voltage(encoderLeft, false),
                encoderLeft != null ? encoderLeft.homeVoltage : 0,
                voltage(encoderRight, true), voltage(encoderRight, false),
                encoderRight != null ? String.format("%.3f", encoderRight.homeVoltage) : "-");
        if (!calibrationResult.isEmpty()) telemetry.addData("Calibración", calibrationResult);
    }

    static String voltage(TurretEncoder encoder, boolean min) {
        if (encoder == null || encoder.vMin > encoder.vMax) return "-";
        return String.format("%.3f", min ? encoder.vMin : encoder.vMax);
    }

    void directionsChanged() {
        fault = false;
        confirmed = false;
        lastSign = 0;
    }

    /** El servo que no se usa queda sin señal, y con "Release" queda suelto. */
    void applyDriveMode() {
        // Los dos siguen con señal (el que no empuja, en "detenido"): sin señal su encoder da 0 V
        setPower(0);
        left.setPower(0);
        right.setPower(0);
        left.setPwmEnable();
        right.setPwmEnable();
    }

    /** Potencia positiva = antihorario, ya con la dirección de cada servo aplicada. */
    void setPower(double power) {
        if (drive != Drive.RIGHT_ONLY) left.setPower(leftReversed ? -power : power);
        if (drive != Drive.LEFT_ONLY) right.setPower(rightReversed ? -power : power);
    }

    /** Lleva un ángulo al rango -180 a 180. */
    static double wrap180(double degrees) {
        double d = (degrees + 180.0) % 360.0;
        if (d < 0) d += 360.0;
        return d - 180.0;
    }
}
