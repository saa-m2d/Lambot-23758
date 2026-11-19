package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotMap;

/*
 * Calibración de los servos de posición: tope (Axon) y flower (2 Swyft).
 * Sirve para descubrir en qué modo está cada servo y para encontrar sus posiciones.
 *
 * La TORRETA NO va aquí: sus servos están en modo continuo y esta prueba los haría girar sin
 * límites. Para la torreta usa "Test 5: Torreta".
 *
 * Estos servos se configuran como tipo "Servo" en el Driver Hub.
 *
 * Todos empiezan SIN energía. Un servo sin energía se puede mover a mano.
 * Al energizar, el servo SALTA a la posición que marca la pantalla: ajusta primero esa posición
 * a algo razonable y mantén las manos lejos.
 *
 * gamepad1 (en el control REV: A = ✕, B = ○, X = □, Y = △, Start = Options)
 *
 *   Siempre:
 *     B:      soltar TODOS los servos (paro de emergencia)
 *     X:      cambiar entre modo "un servo" y modo "flower juntos"
 *
 *   Modo "un servo":
 *     D-pad izquierda/derecha:  elegir servo
 *     A:                        energizar / soltar el servo elegido
 *     D-pad arriba/abajo:       ajuste fino   (±0.0025 = 5 µs)
 *     Bumper derecho/izquierdo: ajuste grueso (±0.025 = 50 µs)
 *     Y:                        ir a 0.5 (centro; en un servo continuo es "detenido")
 *     Start:                    guardar la posición actual en la lista de abajo (para anotarla)
 *
 *   Modo "flower juntos": mueve flowerLeft y flowerRight a la vez, cada uno entre SUS valores de
 *   arriba y abajo (RobotMap.FLOWER_*). Recorrido 0 = arriba, 1 = abajo. Sirve para oír si los
 *   dos servos pelean en algún punto del camino.
 *     A:                        energizar / soltar los dos flower
 *     Stick izquierdo Y:        mover el recorrido poco a poco (arriba = hacia "arriba")
 *     D-pad arriba/abajo:       recorrido ∓0.05
 *     Bumper izquierdo/derecho: ir a arriba (0) / ir a abajo (1)
 *     Y:                        ir a la mitad (0.5)
 *     Start:                    guardar el recorrido y las dos posiciones en la lista de abajo
 *   Si en algún punto zumban: B, vuelve al modo "un servo" y ajusta flowerRight con el izquierdo
 *   energizado hasta que deje de zumbar; anota ese número.
 *
 * ¿Posición o continuo? Energiza en 0.5 y sube un poco la posición:
 *   - se mueve un poco y se queda quieto  -> modo posición
 *   - gira sin parar                      -> modo continuo (CR). Regresa a 0.5 para detenerlo.
 *
 * Posiciones medidas en el robot con esta prueba (2026-11-17), copiadas a RobotMap:
 *   tope abierto = 0.700 (1900 µs), tope cerrado = 0.8975 (2295 µs)
 *   flowerLeft arriba = 0.830, abajo = 0.3281; flowerRight arriba = 0.200, abajo = 0.7019
 */
@TeleOp(name = "Test 2: Servos", group = "Test")
public class ServoCalibrationOpMode extends LinearOpMode {

    static final double FINE_STEP = 0.0025;
    static final double COARSE_STEP = 0.025;
    static final double TOGETHER_STEP = 0.05;
    /** Con el stick al tope, el recorrido completo de arriba a abajo tarda 2 segundos. */
    static final double TOGETHER_STICK_RATE = 0.5;
    // Recorrido nominal del Axon MK2 en modo posición: 0.175° por µs (350° entre 500 y 2500 µs).
    // Es el dato de fábrica; el ángulo real hay que medirlo en el robot.
    static final double AXON_DEGREES_PER_US = 0.175;

    static class Entry {
        final String name, port;
        final boolean isAxon;
        ServoImplEx servo;          // null si no está en la configuración
        double position = 0.5;
        boolean energized = false;

        Entry(String name, String port, boolean isAxon) {
            this.name = name;
            this.port = port;
            this.isAxon = isAxon;
        }

        double microseconds() {
            return RobotMap.SERVO_PWM_MIN_US + position * (RobotMap.SERVO_PWM_MAX_US - RobotMap.SERVO_PWM_MIN_US);
        }
    }

    final Entry tope        = new Entry(RobotMap.TOPE,         "Expansion Hub servo 0", true);
    final Entry flowerLeft  = new Entry(RobotMap.FLOWER_LEFT,  "Control Hub servo 5",   false);
    final Entry flowerRight = new Entry(RobotMap.FLOWER_RIGHT, "Expansion Hub servo 2", false);
    final Entry[] entries = {tope, flowerLeft, flowerRight};

    int selected = 0;
    boolean together = false;
    /** Modo "flower juntos": 0 = arriba, 1 = abajo. */
    double travel = 0;

    // Botones del ciclo actual. Se leen una sola vez por ciclo para que un botón presionado en
    // un modo no se aplique después en el otro.
    boolean pressA, pressB, pressX, pressY, pressStart;
    boolean pressUp, pressDown, pressLeft, pressRight, pressLB, pressRB;

    @Override
    public void runOpMode() {
        for (Entry e : entries) {
            e.servo = hardwareMap.tryGet(ServoImplEx.class, e.name);
            if (e.servo == null) continue;
            e.servo.setPwmRange(new PwmControl.PwmRange(RobotMap.SERVO_PWM_MIN_US, RobotMap.SERVO_PWM_MAX_US));
            e.servo.setPwmDisable();   // nada se mueve hasta que el piloto lo energice
        }

        telemetry.addLine("Todos los servos están sueltos. Lee las instrucciones en el código.");
        for (Entry e : entries) {
            telemetry.addData(e.name, e.servo != null ? "encontrado (" + e.port + ")" : "NO ESTÁ EN LA CONFIGURACIÓN");
        }
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();
            readButtons();

            if (pressB) releaseAll();
            if (pressX) together = !together;

            if (together) {
                controlTogether(dt);
            } else {
                controlSingle();
            }
            showTelemetry();
        }
        releaseAll();
    }

    void readButtons() {
        pressA = gamepad1.aWasPressed();
        pressB = gamepad1.bWasPressed();
        pressX = gamepad1.xWasPressed();
        pressY = gamepad1.yWasPressed();
        pressStart = gamepad1.startWasPressed();
        pressUp = gamepad1.dpadUpWasPressed();
        pressDown = gamepad1.dpadDownWasPressed();
        pressLeft = gamepad1.dpadLeftWasPressed();
        pressRight = gamepad1.dpadRightWasPressed();
        pressLB = gamepad1.leftBumperWasPressed();
        pressRB = gamepad1.rightBumperWasPressed();
    }

    void controlSingle() {
        if (pressRight) selected = (selected + 1) % entries.length;
        if (pressLeft) selected = (selected + entries.length - 1) % entries.length;
        Entry e = entries[selected];

        if (pressA) setEnergized(e, !e.energized);

        double step = 0;
        if (pressUp) step += FINE_STEP;
        if (pressDown) step -= FINE_STEP;
        if (pressRB) step += COARSE_STEP;
        if (pressLB) step -= COARSE_STEP;
        if (pressY) step = 0.5 - e.position;
        if (step != 0) apply(e, Range.clip(e.position + step, 0, 1));

        if (pressStart) {
            telemetry.log().add("%s = %.4f (%.0f µs)", e.name, e.position, e.microseconds());
        }
    }

    void controlTogether(double dt) {
        // Stick arriba (valor negativo) acerca el mecanismo a "arriba" (recorrido 0).
        double target = travel + gamepad1.left_stick_y * TOGETHER_STICK_RATE * dt;
        if (pressUp) target -= TOGETHER_STEP;
        if (pressDown) target += TOGETHER_STEP;
        if (pressLB) target = 0;
        if (pressRB) target = 1;
        if (pressY) target = 0.5;
        travel = Range.clip(target, 0, 1);

        apply(flowerLeft, lerp(RobotMap.FLOWER_LEFT_UP, RobotMap.FLOWER_LEFT_DOWN, travel));
        apply(flowerRight, lerp(RobotMap.FLOWER_RIGHT_UP, RobotMap.FLOWER_RIGHT_DOWN, travel));

        if (pressA) {
            boolean on = !(flowerLeft.energized && flowerRight.energized);
            setEnergized(flowerLeft, on);
            setEnergized(flowerRight, on);
        }

        if (pressStart) {
            telemetry.log().add("juntos %.2f: %s = %.4f, %s = %.4f", travel,
                    flowerLeft.name, flowerLeft.position, flowerRight.name, flowerRight.position);
        }
    }

    void showTelemetry() {
        if (together) {
            telemetry.addLine(String.format("MODO FLOWER JUNTOS  recorrido %.2f  (0 = arriba, 1 = abajo)", travel));
            telemetry.addLine("A: energizar/soltar los dos   X: volver a un servo");
        } else {
            telemetry.addLine("MODO UN SERVO   X: cambiar a flower juntos");
        }
        for (int i = 0; i < entries.length; i++) {
            Entry each = entries[i];
            String state = each.servo == null ? "NO ESTÁ EN LA CONFIGURACIÓN"
                    : String.format("%s  pos %.4f  %.0f µs%s", each.energized ? "ENERGIZADO" : "suelto",
                            each.position, each.microseconds(),
                            each.isAxon ? String.format("  (~%+.1f° del centro)",
                                    (each.microseconds() - 1500) * AXON_DEGREES_PER_US) : "");
            boolean marked = together ? (each == flowerLeft || each == flowerRight) : i == selected;
            telemetry.addData((marked ? "> " : "  ") + each.name, state);
        }
        telemetry.update();
    }

    static double lerp(double from, double to, double t) {
        return from + (to - from) * t;
    }

    void setEnergized(Entry e, boolean on) {
        if (e.servo == null) return;
        if (on) {
            e.servo.setPosition(e.position);
            e.servo.setPwmEnable();
        } else {
            e.servo.setPwmDisable();
        }
        e.energized = on;
    }

    void releaseAll() {
        for (Entry e : entries) setEnergized(e, false);
    }

    void apply(Entry e, double position) {
        e.position = position;
        if (e.servo != null && e.energized) e.servo.setPosition(position);
    }
}
