package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * Procedimientos de tuneo de PedroPathing que aparecen en la herramienta de tuneo.
 * Orden: Mecanum (direcciones, ya medidas), Pinpoint, Foresight y al final Tests.
 * Los resultados de Foresight se copian a Constants.java.
 */
public class Tuning {
    @Tuner(name = "1. Mecanum")
    static Procedure mecanum() {
        return new MecanumTuner();
    }

    @Tuner(name = "2. Pinpoint")
    static Procedure pinpoint() {
        return new PinpointTuner();
    }

    @Tuner(name = "3. Foresight")
    static Procedure foresight() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    @Tuner(name = "4. Pruebas")
    static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer,
                () -> new Foresight(Constants.foresightConfig()));
    }
}
