package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.RobotMap;

/*
 * Calibración de los 5 servos: torreta (2 Axon), tope (Axon) y flower (2 Swyft).
 * Sirve para descubrir en qué modo está cada servo y para encontrar sus posiciones.
 *
 * Para esta prueba, TODOS los servos se configuran como tipo "Servo" en el Driver Hub.
 *
 * Todos empiezan SIN energía. Un servo sin energía se puede mover a mano.
 * Al energizar, el servo SALTA a la posición que marca la pantalla: ajusta primero esa posición
 * a algo razonable y mantén las manos lejos.
 *
 * gamepad1
 *   D-pad izquierda/derecha: elegir servo
 *   A:                       energizar / soltar el servo elegido
 *   D-pad arriba/abajo:      ajuste fino   (±0.0025 = 5 µs)
 *   Bumper derecho/izquierdo: ajuste grueso (±0.025 = 50 µs)
 *   Y:                       ir a 0.5 (centro; en un servo continuo es "detenido")
 *   B:                       soltar TODOS los servos (paro de emergencia)
 *   Start:                   guardar la posición actual en la lista de abajo (para anotarla)
 *   X:                       (solo torreta) unir / separar los dos servos de la torreta
 *   Botón del stick izq.:    (solo torreta) cambiar entre "en espejo" y "mismo sentido"
 *
 * ¿Posición o continuo? Energiza en 0.5 y sube un poco la posición:
 *   - se mueve un poco y se queda quieto  -> modo posición
 *   - gira sin parar                      -> modo continuo (CR). Regresa a 0.5 para detenerlo.
 *
 * TORRETA: los dos servos mueven el mismo eje. NUNCA energices los dos sin haberlos unido antes,
 * porque pelearían entre sí. El procedimiento está en CHECKLIST.md, sección 5.
 */
@TeleOp(name = "Test 2: Servos", group = "Test")
public class ServoCalibrationOpMode extends LinearOpMode {

    static final double FINE_STEP = 0.0025;
    static final double COARSE_STEP = 0.025;
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

    static final int TURRET_LEFT = 0, TURRET_RIGHT = 1;
    final Entry[] entries = {
            new Entry(RobotMap.TURRET_LEFT,  "Control Hub servo 4",   true),
            new Entry(RobotMap.TURRET_RIGHT, "Expansion Hub servo 1", true),
            new Entry(RobotMap.TOPE,         "Expansion Hub servo 0", true),
            new Entry(RobotMap.FLOWER_LEFT,  "Control Hub servo 5",   false),
            new Entry(RobotMap.FLOWER_RIGHT, "Expansion Hub servo 2", false),
    };

    int selected = 0;

    // Unión de la torreta: derecho = (espejo ? 1 - izquierdo : izquierdo) + turretOffset
    boolean turretLinked = false;
    boolean turretMirrored = true;
    double turretOffset = 0;

    @Override
    public void runOpMode() {
        for (Entry e : entries) {
            e.servo = hardwareMap.tryGet(ServoImplEx.class, e.name);
            if (e.servo == null) continue;
            e.servo.setPwmRange(new PwmControl.PwmRange(RobotMap.SERVO_PWM_MIN_US, RobotMap.SERVO_PWM_MAX_US));
            e.servo.setPwmDisable();   // nada se mueve hasta que el piloto lo energice
        }
        AnalogInput turretEncoder = hardwareMap.tryGet(AnalogInput.class, RobotMap.TURRET_ENCODER);

        telemetry.addLine("Todos los servos están sueltos. Lee las instrucciones en el código.");
        for (Entry e : entries) {
            telemetry.addData(e.name, e.servo != null ? "encontrado (" + e.port + ")" : "NO ESTÁ EN LA CONFIGURACIÓN");
        }
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadRightWasPressed()) selected = (selected + 1) % entries.length;
            if (gamepad1.dpadLeftWasPressed()) selected = (selected + entries.length - 1) % entries.length;
            Entry e = entries[selected];
            boolean isTurret = selected == TURRET_LEFT || selected == TURRET_RIGHT;

            if (gamepad1.bWasPressed()) releaseAll();

            if (gamepad1.aWasPressed()) {
                if (isTurret && turretLinked) {
                    setTurretEnergized(!e.energized);
                } else if (isTurret && !e.energized && other(selected).energized) {
                    // Evita que los dos servos de la torreta peleen sin estar unidos
                    telemetry.log().add("Suelta primero %s, o une la torreta con X", other(selected).name);
                } else {
                    setEnergized(e, !e.energized);
                }
            }

            if (isTurret && gamepad1.xWasPressed()) toggleTurretLink();
            if (isTurret && !turretLinked && gamepad1.leftStickButtonWasPressed()) turretMirrored = !turretMirrored;

            double step = 0;
            if (gamepad1.dpadUpWasPressed()) step += FINE_STEP;
            if (gamepad1.dpadDownWasPressed()) step -= FINE_STEP;
            if (gamepad1.rightBumperWasPressed()) step += COARSE_STEP;
            if (gamepad1.leftBumperWasPressed()) step -= COARSE_STEP;
            if (gamepad1.yWasPressed()) step = 0.5 - e.position;
            if (step != 0) move(e, isTurret, step);

            if (gamepad1.startWasPressed()) {
                telemetry.log().add("%s = %.4f (%.0f µs)", e.name, e.position, e.microseconds());
            }

            for (int i = 0; i < entries.length; i++) {
                Entry each = entries[i];
                String state = each.servo == null ? "NO ESTÁ EN LA CONFIGURACIÓN"
                        : String.format("%s  pos %.4f  %.0f µs%s", each.energized ? "ENERGIZADO" : "suelto",
                                each.position, each.microseconds(),
                                each.isAxon ? String.format("  (~%+.1f° del centro)",
                                        (each.microseconds() - 1500) * AXON_DEGREES_PER_US) : "");
                telemetry.addData((i == selected ? "> " : "  ") + each.name, state);
            }
            telemetry.addData("Torreta", turretLinked
                    ? String.format("UNIDA (%s, offset %+.4f)", turretMirrored ? "espejo" : "mismo sentido", turretOffset)
                    : String.format("separada (al unir: %s)", turretMirrored ? "espejo" : "mismo sentido"));
            if (turretEncoder != null) {
                telemetry.addData("Encoder torreta", "%.3f V", turretEncoder.getVoltage());
            }
            telemetry.update();
        }
        releaseAll();
    }

    Entry other(int turretIndex) {
        return entries[turretIndex == TURRET_LEFT ? TURRET_RIGHT : TURRET_LEFT];
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

    void setTurretEnergized(boolean on) {
        setEnergized(entries[TURRET_LEFT], on);
        setEnergized(entries[TURRET_RIGHT], on);
    }

    void releaseAll() {
        for (Entry e : entries) setEnergized(e, false);
    }

    /**
     * Une los servos de la torreta usando sus posiciones actuales como referencia: se asume que en
     * este momento los dos están apuntando la torreta al mismo lugar.
     */
    void toggleTurretLink() {
        setTurretEnergized(false);   // siempre se une o separa con los dos sueltos
        turretLinked = !turretLinked;
        if (turretLinked) {
            double left = entries[TURRET_LEFT].position;
            turretOffset = entries[TURRET_RIGHT].position - (turretMirrored ? 1 - left : left);
            telemetry.log().add("Torreta unida: %s, offset %+.4f", turretMirrored ? "espejo" : "mismo sentido", turretOffset);
        }
    }

    /** Posición del servo derecho que corresponde a una posición del izquierdo. */
    double rightFromLeft(double left) {
        return (turretMirrored ? 1 - left : left) + turretOffset;
    }

    void move(Entry e, boolean isTurret, double step) {
        if (isTurret && turretLinked) {
            // Con la torreta unida se mueve el izquierdo y el derecho lo sigue
            double direction = (selected == TURRET_RIGHT && turretMirrored) ? -1 : 1;
            double left = Range.clip(entries[TURRET_LEFT].position + step * direction, 0, 1);
            double right = rightFromLeft(left);
            if (right < 0 || right > 1) return;   // el derecho se saldría de su recorrido
            apply(entries[TURRET_LEFT], left);
            apply(entries[TURRET_RIGHT], right);
        } else {
            apply(e, Range.clip(e.position + step, 0, 1));
        }
    }

    void apply(Entry e, double position) {
        e.position = position;
        if (e.servo != null && e.energized) e.servo.setPosition(position);
    }
}
