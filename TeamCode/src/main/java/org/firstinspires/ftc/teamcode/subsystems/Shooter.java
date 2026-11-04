package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Shooter de 2 flywheels independientes, con el control de velocidad de cooperRobot:
 * feedforward (kS + kV) más corrección proporcional sobre el RPM filtrado.
 *
 * Inactivo, los flywheels giran hacia atrás para que ninguna pelota entre al shooter.
 * Activo, aceleran hacia adelante al RPM objetivo.
 *
 * Hay que llamar a update() una vez por ciclo; es lo que realmente mueve los motores.
 *
 * Configuración:
 *   Expansion Hub motor 2: shooterBack (rodillo + rueda trasera)
 *                 motor 3: shooterFront (rueda delantera)
 */
public class Shooter {

    // TODO: verificar en el robot. Si un flywheel gira al revés, cambia su dirección aquí.
    static final DcMotorSimple.Direction BACK_DIRECTION  = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction FRONT_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // Ticks por vuelta del eje de salida del flywheel. Yellow Jacket 1:1 (6000 RPM) = 28.
    // Si el motor tiene caja de engranes, multiplica 28 por la relación (ej. 3.7:1 -> 103.6).
    static final double TICKS_PER_REV = 28.0;

    // RPM de cada modo
    static final double DEFAULT_BACK_RPM = 2000;     // objetivo inicial de shooterBack al disparar
    static final double DEFAULT_FRONT_RPM = 2000;    // objetivo inicial de shooterFront al disparar
    static final double MIN_TARGET_RPM = 500;        // evita alimentar pelotas con el objetivo en 0
    static final double IDLE_RPM = -300;             // reversa constante cuando no se dispara

    // Fracción del objetivo que cuenta como "listo" (0.96 = 96 %)
    static final double READY_TOLERANCE = 0.96;

    // Ganancias de cooperRobot, como punto de partida. TODO: ajustarlas en este robot.
    static final double KS_IDLE = 0.3;           // potencia mínima para vencer la fricción en reversa
    static final double KV_IDLE = 0.00028;       // potencia por cada RPM en reversa
    static final double KS_ACTIVE = 0.16;        // potencia mínima para vencer la fricción al disparar
    static final double KV_ACTIVE = 0.00036;     // potencia por cada RPM al disparar
    static final double KP = 0.0008;             // corrección por cada RPM de error

    static final double RPM_FILTER = 0.85;       // peso de la lectura anterior; más alto = más suave
    static final double RPM_DEADBAND = 10;       // errores menores a esto se ignoran

    private final Flywheel back, front;
    private double backTargetRpm = DEFAULT_BACK_RPM;
    private double frontTargetRpm = DEFAULT_FRONT_RPM;
    private boolean active = false;

    public Shooter(HardwareMap hardwareMap) {
        back  = new Flywheel(hardwareMap, "shooterBack", BACK_DIRECTION);
        front = new Flywheel(hardwareMap, "shooterFront", FRONT_DIRECTION);
    }

    /** Lee los encoders y aplica la potencia a cada flywheel. Llamar una vez por ciclo. */
    public void update() {
        double kS = active ? KS_ACTIVE : KS_IDLE;
        double kV = active ? KV_ACTIVE : KV_IDLE;
        back.update(active ? backTargetRpm : IDLE_RPM, kS, kV);
        front.update(active ? frontTargetRpm : IDLE_RPM, kS, kV);
    }

    /** true: acelerar hacia adelante al RPM objetivo. false: reversa constante. */
    public void setActive(boolean active) {
        this.active = active;
    }

    /** Sube o baja el RPM al que dispara shooterBack. */
    public void adjustBackTargetRpm(double deltaRpm) {
        backTargetRpm = Math.max(MIN_TARGET_RPM, backTargetRpm + deltaRpm);
    }

    /** Sube o baja el RPM al que dispara shooterFront. */
    public void adjustFrontTargetRpm(double deltaRpm) {
        frontTargetRpm = Math.max(MIN_TARGET_RPM, frontTargetRpm + deltaRpm);
    }

    /** true cuando está activo y cada flywheel llegó a su propio RPM objetivo. */
    public boolean isReady() {
        return active
                && back.filteredRpm >= backTargetRpm * READY_TOLERANCE
                && front.filteredRpm >= frontTargetRpm * READY_TOLERANCE;
    }

    public void addTelemetry(Telemetry telemetry) {
        // Los objetivos se muestran siempre, para poder ajustarlos antes de entrar al modo disparo
        telemetry.addData("Objetivo disparo trasero / delantero (RPM)", "%.0f / %.0f", backTargetRpm, frontTargetRpm);
        telemetry.addData("Actual trasero / delantero (RPM)", "%.0f / %.0f", back.filteredRpm, front.filteredRpm);
        telemetry.addData("Shooter potencia", "%.2f / %.2f", back.power, front.power);
        telemetry.addData("Shooter", isReady() ? "LISTO" : (active ? "acelerando..." : "reversa"));
    }

    /** Un motor con su propio filtro y control, porque cada flywheel carga un peso distinto. */
    private static class Flywheel {
        private final DcMotorEx motor;
        double filteredRpm = 0;
        double power = 0;

        Flywheel(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction) {
            motor = hardwareMap.get(DcMotorEx.class, name);
            motor.setDirection(direction);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // que se detenga sola, sin frenar
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // Sin el control interno del hub: la potencia se calcula aquí, y el encoder se sigue leyendo.
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        void update(double targetRpm, double kS, double kV) {
            // Suavizar la lectura, porque la velocidad del encoder trae ruido
            double rawRpm = motor.getVelocity() * 60.0 / TICKS_PER_REV;
            filteredRpm = RPM_FILTER * filteredRpm + (1 - RPM_FILTER) * rawRpm;

            double error = targetRpm - filteredRpm;
            if (Math.abs(error) < RPM_DEADBAND) error = 0;

            // Feedforward: la potencia que debería bastar. KP corrige lo que falte o sobre.
            double feedforward = kS * Math.signum(targetRpm) + kV * targetRpm;
            power = Math.max(-1.0, Math.min(1.0, feedforward + KP * error));
            motor.setPower(power);
        }
    }
}
