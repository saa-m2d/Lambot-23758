package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Voltaje de la batería. Leerlo es lento, así que se lee unas 4 veces por segundo y se suaviza.
 */
public class BatteryMonitor {
    private static final double READ_PERIOD_S = 0.25;
    private static final double FILTER = 0.5;

    private final VoltageSensor sensor;
    private final ElapsedTime sinceRead = new ElapsedTime();
    private double voltage = 12.0;
    private boolean hasReading;

    public BatteryMonitor(HardwareMap hardwareMap) {
        VoltageSensor best = null;
        for (VoltageSensor candidate : hardwareMap.voltageSensor) {
            best = candidate;
            break;
        }
        sensor = best;
        update(true);
    }

    public void update() {
        update(false);
    }

    private void update(boolean force) {
        if (sensor == null) return;
        if (!force && sinceRead.seconds() < READ_PERIOD_S) return;
        sinceRead.reset();
        double reading = sensor.getVoltage();
        if (reading < 5) return;   // lectura inválida
        voltage = hasReading ? voltage + FILTER * (reading - voltage) : reading;
        hasReading = true;
    }

    /** Voltaje suavizado; 12 V si no hay sensor. */
    public double voltage() {
        return voltage;
    }

    public boolean hasReading() {
        return hasReading;
    }
}
