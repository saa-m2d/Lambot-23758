package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Modo disparo: coordina el shooter y el intake.
 *   IDLE:      flywheels en reversa; el intake lo controla quien usa esta clase.
 *   PREPARING: todo gira hacia atrás un momento para alejar las pelotas de los flywheels.
 *   READY:     los flywheels aceleran y el intake espera detenido.
 *   SHOOTING:  se pidió disparar; el intake solo empuja mientras los flywheels estén a su RPM.
 *
 * start() entra al modo disparo y cancel() lo termina desde cualquier estado.
 */
public class ShootSequence {

    // Sin sensor de pelotas, la preparación dura un tiempo fijo.
    // Muy largo saca las pelotas del robot; muy corto no las aleja de los flywheels.
    static final double PREPARE_TIME_SECONDS = 0.08;

    private enum State { IDLE, PREPARING, READY, SHOOTING }

    private final Shooter shooter;
    private final Intake intake;
    private final ElapsedTime prepareTimer = new ElapsedTime();
    private State state = State.IDLE;

    public ShootSequence(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    /** Entra al modo disparo, empezando por la preparación. */
    public void start() {
        if (isActive()) return;
        state = State.PREPARING;
        prepareTimer.reset();
    }

    /** Sale del modo disparo y regresa los flywheels a reversa. */
    public void cancel() {
        state = State.IDLE;
    }

    /**
     * Avanza la secuencia. Llamar una vez por ciclo.
     * @param feedRequested true mientras el piloto pide disparar
     */
    public void update(boolean feedRequested) {
        switch (state) {
            case IDLE:
                shooter.setActive(false);
                break;

            case PREPARING:
                shooter.setActive(false);
                intake.reverse();
                if (prepareTimer.seconds() >= PREPARE_TIME_SECONDS) state = State.READY;
                break;

            case READY:
            case SHOOTING:
                shooter.setActive(true);
                state = feedRequested ? State.SHOOTING : State.READY;
                if (!feedRequested) {
                    intake.stop();
                } else if (shooter.isReady()) {
                    intake.feed();
                } else {
                    // El RPM bajó o aún no llega: las pelotas esperan para no salir lentas
                    intake.holdForShooter();
                }
                break;
        }
    }

    /** true si está en modo disparo; ahí la secuencia controla el intake. */
    public boolean isActive() {
        return state != State.IDLE;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Disparo", state);
    }
}
