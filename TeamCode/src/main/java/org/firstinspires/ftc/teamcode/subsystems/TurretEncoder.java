package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * El 4.º cable de un Axon de la torreta: da la posición del servo dentro de una vuelta (0-3.3 V).
 * Esta clase cuenta las vueltas para dar el ángulo de la torreta desde home.
 *
 * Una vuelta del servo = 360°/GEAR_RATIO de torreta (~128°), así que al encender el ángulo solo
 * se conoce "módulo una vuelta de servo". Se resuelve con una pista: el ángulo que se espera tener (0 = en home al empezar el
 * autónomo; el ángulo guardado por el autónomo al empezar el TeleOp).
 */
public class TurretEncoder {
    private final AnalogInput input;
    private final boolean reversed;
    private final double vMin, vMax;
    private double servoDegFromHome;   // grados de servo acumulados, sin invertir
    private double lastRawDeg;
    private double voltage;

    /**
     * @param homeVoltageConfig voltaje medido en home, o negativo si no se ha medido
     * @param hintTurretDeg     ángulo de torreta que se espera tener ahora
     */
    public TurretEncoder(AnalogInput input, boolean reversed, double vMin, double vMax,
                         double homeVoltageConfig, double hintTurretDeg) {
        this.input = input;
        this.reversed = reversed;
        this.vMin = vMin;
        this.vMax = vMax;
        voltage = input.getVoltage();
        double raw = rawDeg(voltage);
        double hintServoDeg = sign() * hintTurretDeg * TurretConfig.GEAR_RATIO;
        if (homeVoltageConfig < 0) {
            // Sin home medido: se confía en la pista
            servoDegFromHome = hintServoDeg;
        } else {
            double relative = MathUtil.wrap180(raw - rawDeg(homeVoltageConfig));
            long turns = Math.round((hintServoDeg - relative) / 360.0);
            servoDegFromHome = relative + 360.0 * turns;
        }
        lastRawDeg = raw;
    }

    private double sign() {
        return reversed ? -1 : 1;
    }

    /** Lee el voltaje y suma lo que giró el servo desde la última lectura. */
    public void read() {
        voltage = input.getVoltage();
        double raw = rawDeg(voltage);
        servoDegFromHome += MathUtil.wrap180(raw - lastRawDeg);
        lastRawDeg = raw;
    }

    /** Grados de torreta desde home. Positivo = antihorario. */
    public double turretDeg() {
        return sign() * servoDegFromHome / TurretConfig.GEAR_RATIO;
    }

    public double voltage() {
        return voltage;
    }

    private double rawDeg(double volts) {
        double v = Range.clip(volts, vMin, vMax);
        return (v - vMin) / (vMax - vMin) * 360.0;
    }
}
