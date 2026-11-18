package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "Furby Auto AZUL", group = "Competencia", preselectTeleOp = "Furby TeleOp")
public class BlueAutonomous extends BiobuzzAutonomous {
    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
