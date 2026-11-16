package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.RobotMap;

/*
 * Prueba los 8 motores uno por uno y avisa si alguno falla.
 *
 * ANTES DE EMPEZAR: sube el robot a unos bloques (ruedas al aire) y saca las pelotas.
 *
 * gamepad1
 *   D-pad izquierda/derecha: elegir motor
 *   Stick izquierdo Y:       potencia manual (arriba = positiva), limitada a MAX_MANUAL_POWER
 *   A (mantener):            potencia fija de prueba (+SWEEP_POWER)
 *   X:                       barrido automático: prueba todos, uno tras otro, y muestra una tabla
 *   B:                       detener todo / cancelar el barrido
 *
 * Aquí todos los motores van en dirección FORWARD, sin invertir. Anota hacia dónde gira cada uno
 * con potencia positiva: con eso se definen las direcciones en los subsistemas.
 */
@TeleOp(name = "Test 1: Motores", group = "Test")
public class MotorTestOpMode extends LinearOpMode {

    static final double MAX_MANUAL_POWER = 0.6;      // tope de la potencia manual
    static final double SWEEP_POWER = 0.35;          // potencia del barrido y del botón A
    static final double SWEEP_SECONDS = 1.2;         // cuánto dura cada motor en el barrido
    static final double SETTLE_SECONDS = 0.4;        // espera antes de juzgar (el motor acelera)
    static final double MIN_POWER_TO_JUDGE = 0.2;    // con menos potencia no se diagnostica
    static final double NO_CURRENT_AMPS = 0.05;      // menos que esto = no hay motor conectado
    static final double OVERCURRENT_AMPS = 7.0;      // más que esto = atorado o en corto
    static final double MIN_TICKS_PER_SECOND = 20;   // menos que esto = el encoder no se mueve

    /** Un motor de la lista, con lo que se sabe de él. */
    static class Entry {
        final String name, port;
        final boolean hasEncoder;
        DcMotorEx motor;            // null si no está en la configuración
        double peakAmps;            // corriente máxima vista
        String sweepResult = "-";   // resultado del último barrido

        Entry(String name, String port, boolean hasEncoder) {
            this.name = name;
            this.port = port;
            this.hasEncoder = hasEncoder;
        }
    }

    final Entry[] entries = {
            new Entry(RobotMap.FRONT_LEFT,    "Control Hub 2",   false),
            new Entry(RobotMap.BACK_LEFT,     "Control Hub 3",   false),
            new Entry(RobotMap.FRONT_RIGHT,   "Expansion Hub 3", false),
            new Entry(RobotMap.BACK_RIGHT,    "Expansion Hub 2", false),
            new Entry(RobotMap.SHOOTER_LEFT,  "Control Hub 0",   true),
            new Entry(RobotMap.SHOOTER_RIGHT, "Control Hub 1",   true),
            new Entry(RobotMap.INTAKE_LEFT,   "Expansion Hub 1", false),
            new Entry(RobotMap.INTAKE_RIGHT,  "Expansion Hub 0", false),
    };

    int selected = 0;
    final ElapsedTime poweredTimer = new ElapsedTime();   // tiempo que lleva el motor con potencia
    boolean wasPowered = false;

    @Override
    public void runOpMode() {
        for (Entry e : entries) {
            e.motor = hardwareMap.tryGet(DcMotorEx.class, e.name);
            if (e.motor == null) continue;
            e.motor.setDirection(DcMotor.Direction.FORWARD);
            e.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            e.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            e.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("ROBOT SOBRE BLOQUES, ruedas al aire.");
        addConfigSummary();
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpadRightWasPressed()) select(selected + 1);
            if (gamepad1.dpadLeftWasPressed()) select(selected - 1);
            if (gamepad1.xWasPressed()) runSweep();

            Entry e = entries[selected];
            double power = gamepad1.a ? SWEEP_POWER : -gamepad1.left_stick_y * MAX_MANUAL_POWER;
            if (gamepad1.b || Math.abs(power) < 0.05) power = 0;
            if (e.motor != null) e.motor.setPower(power);

            // El reloj corre solo mientras hay potencia suficiente para diagnosticar
            boolean powered = Math.abs(power) >= MIN_POWER_TO_JUDGE;
            if (powered && !wasPowered) poweredTimer.reset();
            wasPowered = powered;

            telemetry.addData("Motor", "%d/%d  %s  (%s)", selected + 1, entries.length, e.name, e.port);
            if (e.motor == null) {
                telemetry.addLine("FALLA: no está en la configuración con ese nombre");
            } else {
                double amps = e.motor.getCurrent(CurrentUnit.AMPS);
                double ticksPerSecond = e.motor.getVelocity();
                e.peakAmps = Math.max(e.peakAmps, amps);

                telemetry.addData("Potencia", "%.2f", power);
                telemetry.addData("Corriente", "%.2f A  (máx %.2f A)", amps, e.peakAmps);
                if (e.hasEncoder) {
                    telemetry.addData("Encoder", "%d ticks", e.motor.getCurrentPosition());
                    telemetry.addData("Velocidad", "%.0f RPM",
                            ticksPerSecond * 60.0 / RobotMap.SHOOTER_TICKS_PER_REV);
                }
                boolean settled = powered && poweredTimer.seconds() >= SETTLE_SECONDS;
                telemetry.addData("Diagnóstico",
                        settled ? diagnose(e, power, amps, ticksPerSecond) : "aplica potencia para diagnosticar");
            }
            telemetry.addLine();
            telemetry.addLine("Barrido (X):");
            for (Entry each : entries) telemetry.addData("  " + each.name, each.sweepResult);
            telemetry.update();
        }
        stopAll();
    }

    void select(int index) {
        stopAll();
        selected = (index + entries.length) % entries.length;
        wasPowered = false;
    }

    void stopAll() {
        for (Entry e : entries) if (e.motor != null) e.motor.setPower(0);
    }

    /** Dice qué anda mal con un motor que lleva un rato con potencia, o "OK". */
    static String diagnose(Entry e, double power, double amps, double ticksPerSecond) {
        if (amps < NO_CURRENT_AMPS) {
            return "FALLA: sin corriente. Cable de potencia suelto, puerto equivocado o motor dañado";
        }
        if (amps > OVERCURRENT_AMPS || e.motor.isOverCurrent()) {
            return String.format("FALLA: sobrecorriente (%.1f A). Mecanismo atorado o en corto", amps);
        }
        if (e.hasEncoder) {
            if (Math.abs(ticksPerSecond) < MIN_TICKS_PER_SECOND) {
                return "FALLA: hay corriente pero el encoder no cuenta. Cable de encoder suelto o eje trabado";
            }
            if (Math.signum(ticksPerSecond) != Math.signum(power)) {
                return "AVISO: el encoder cuenta al revés que el motor (cables de potencia invertidos)";
            }
        }
        return String.format("OK (%.2f A)", amps);
    }

    /** Prende cada motor un momento, uno por uno, y guarda el diagnóstico de cada uno. */
    void runSweep() {
        stopAll();
        for (Entry e : entries) {
            if (!opModeIsActive() || gamepad1.b) break;
            if (e.motor == null) {
                e.sweepResult = "FALLA: no está en la configuración";
                continue;
            }
            telemetry.addData("Barrido", "probando %s (%s)...  B cancela", e.name, e.port);
            telemetry.update();

            e.motor.setPower(SWEEP_POWER);
            ElapsedTime timer = new ElapsedTime();
            double ampsSum = 0, ticksSum = 0;
            int samples = 0;
            while (opModeIsActive() && !gamepad1.b && timer.seconds() < SWEEP_SECONDS) {
                if (timer.seconds() >= SETTLE_SECONDS) {
                    double amps = e.motor.getCurrent(CurrentUnit.AMPS);
                    e.peakAmps = Math.max(e.peakAmps, amps);
                    ampsSum += amps;
                    ticksSum += e.motor.getVelocity();
                    samples++;
                }
                idle();
            }
            e.motor.setPower(0);
            e.sweepResult = samples == 0 ? "cancelado"
                    : diagnose(e, SWEEP_POWER, ampsSum / samples, ticksSum / samples);
            sleep(250);   // deja que se detenga antes del siguiente
        }
        stopAll();
        wasPowered = false;
    }

    void addConfigSummary() {
        for (Entry e : entries) {
            telemetry.addData(e.name, e.motor != null ? "encontrado (" + e.port + ")" : "NO ESTÁ EN LA CONFIGURACIÓN");
        }
    }
}
