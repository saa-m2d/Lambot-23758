package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.MechanismConstants;
import org.firstinspires.ftc.teamcode.config.RobotMap;

/**
 * El tope (Axon en modo posición) que deja pasar las pelotas al shooter. Solo abre cuando se lo
 * piden Y quien lo pide ya revisó que el shooter está listo. Respeta tiempos mínimos abierto y
 * cerrado para que una pelota pase completa y el shooter recupere RPM entre tiros.
 */
public class Indexer implements Subsystem {
    private final ServoImplEx tope;
    private boolean wantOpen;
    private boolean open;
    private boolean started;
    private final ElapsedTime sinceChange = new ElapsedTime();
    private int releases;

    public Indexer(HardwareCheck hardware) {
        tope = hardware.get(ServoImplEx.class, RobotMap.TOPE, "servo Expansion Hub 0");
        if (tope != null) {
            tope.setPwmRange(new PwmControl.PwmRange(RobotMap.SERVO_PWM_MIN_US, RobotMap.SERVO_PWM_MAX_US));
        }
    }

    @Override
    public boolean isAvailable() {
        return tope != null;
    }

    /** true = dejar pasar pelotas en este ciclo (ya revisado que el shooter está listo). */
    public void setOpen(boolean open) {
        wantOpen = open;
    }

    public boolean isOpen() {
        return open;
    }

    /** Cuántas veces se ha abierto (aprox. pelotas lanzadas si se abre una vez por pelota). */
    public int releases() {
        return releases;
    }

    @Override
    public void periodic() {
        if (tope == null) return;
        if (!started) {
            // Al primer ciclo el tope va a cerrado, aunque al encender estuviera en otro lado
            tope.setPosition(RobotMap.TOPE_CLOSED);
            tope.setPwmEnable();
            started = true;
            sinceChange.reset();
        }
        double minTime = open ? MechanismConstants.TOPE_MIN_OPEN_S : MechanismConstants.TOPE_MIN_CLOSED_S;
        if (wantOpen != open && sinceChange.seconds() >= minTime) {
            open = wantOpen;
            sinceChange.reset();
            if (open) releases++;
        }
        tope.setPosition(open ? RobotMap.TOPE_OPEN : RobotMap.TOPE_CLOSED);
    }

    @Override
    public void stop() {
        wantOpen = false;
        if (tope != null && started) {
            open = false;
            tope.setPosition(RobotMap.TOPE_CLOSED);
        }
    }

    @Override
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Tope", tope == null ? "FALTA HARDWARE" : (open ? "ABIERTO" : "cerrado"));
    }
}
