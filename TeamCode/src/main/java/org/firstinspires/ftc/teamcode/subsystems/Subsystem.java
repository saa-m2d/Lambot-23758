package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Un mecanismo del robot. En cada ciclo, primero se leen los sensores de todos, luego el OpMode
 * decide qué hacer y al final cada subsistema aplica sus salidas con {@link #periodic()}.
 */
public interface Subsystem {
    /** true si todo su hardware está en la configuración. Si no, el subsistema no hace nada. */
    boolean isAvailable();

    /** Aplica las salidas (potencias, posiciones). Una vez por ciclo, al final. */
    void periodic();

    /** Deja el mecanismo sin potencia. */
    void stop();

    void addTelemetry(Telemetry telemetry);
}
