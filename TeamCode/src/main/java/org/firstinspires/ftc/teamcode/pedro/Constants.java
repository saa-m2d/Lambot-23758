package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.RobotMap;

/**
 * Configuración de PedroPathing 3.0 para Furby: chasis, localizador y algoritmo (Foresight).
 *
 * Chasis y Pinpoint salen de RobotMap (ya medidos con las pruebas). Foresight necesita los valores
 * del tuneo: corre el "Foresight Tuner" (ver Tuning.java), copia cada número que entrega en la
 * constante con el mismo nombre de abajo y pon FORESIGHT_TUNED = true. Mientras sea false, el
 * autónomo no usa Pedro y hace su versión sencilla (disparar y salir).
 */
public class Constants {
    public static final String PINPOINT_NAME = RobotMap.PINPOINT;

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(RobotMap.FRONT_LEFT);
        c.frontRightName.set(RobotMap.FRONT_RIGHT);
        c.backLeftName.set(RobotMap.BACK_LEFT);
        c.backRightName.set(RobotMap.BACK_RIGHT);
        c.frontLeftDirection.set(RobotMap.FRONT_LEFT_DIRECTION);
        c.frontRightDirection.set(RobotMap.FRONT_RIGHT_DIRECTION);
        c.backLeftDirection.set(RobotMap.BACK_LEFT_DIRECTION);
        c.backRightDirection.set(RobotMap.BACK_RIGHT_DIRECTION);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PINPOINT_NAME);
        c.xPodDirection.set(RobotMap.PINPOINT_X_POD_DIRECTION);
        c.yPodDirection.set(RobotMap.PINPOINT_Y_POD_DIRECTION);
        c.xPodOffset.set(RobotMap.PINPOINT_X_POD_OFFSET_MM);
        c.yPodOffset.set(RobotMap.PINPOINT_Y_POD_OFFSET_MM);
        c.offsetUnits.set(DistanceUnit.MM);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
    });

    // ---------- Resultados del Foresight Tuner ----------
    public static boolean FORESIGHT_TUNED = false;

    public static double FORWARD_TRANSLATIONAL_PRIMARY_KP = 0;
    public static double FORWARD_TRANSLATIONAL_SECONDARY_KP = 0;
    public static double STRAFE_TRANSLATIONAL_PRIMARY_KP = 0;
    public static double STRAFE_TRANSLATIONAL_SECONDARY_KP = 0;
    public static double COAST_KV = 0;
    public static double BRAKE_KV = 0;
    public static double HEADING_KP = 0;
    public static double HEADING_BRAKING_LINEAR = 0;
    public static double HEADING_BRAKING_QUADRATIC = 0;
    public static double FORWARD_BRAKING_LINEAR = 0;
    public static double FORWARD_BRAKING_QUADRATIC = 0;
    public static double STRAFE_BRAKING_LINEAR = 0;
    public static double STRAFE_BRAKING_QUADRATIC = 0;
    public static double MAX_FORWARD_VELOCITY = 0;
    public static double MAX_STRAFE_VELOCITY = 0;
    public static double NATURAL_FORWARD_DECELERATION = 0;
    public static double NATURAL_STRAFE_DECELERATION = 0;

    /** Igual a lo que genera el Foresight Tuner, con los valores de arriba. */
    public static ForesightConfig foresightConfig() {
        return new ForesightConfig(c -> {
            Controller primaryForward = Controller.proportional(FORWARD_TRANSLATIONAL_PRIMARY_KP);
            Controller secondaryForward = Controller.proportional(FORWARD_TRANSLATIONAL_SECONDARY_KP);
            Controller primaryLateral = Controller.proportional(STRAFE_TRANSLATIONAL_PRIMARY_KP);
            Controller secondaryLateral = Controller.proportional(STRAFE_TRANSLATIONAL_SECONDARY_KP);

            c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryLateral).put(2.5, primaryLateral));

            c.coast.set(Controller.proportionalFeedforward(COAST_KV));
            c.brake.set(Controller.proportionalFeedforward(BRAKE_KV));

            c.headingFeedback.set(Controller.proportional(HEADING_KP));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(HEADING_BRAKING_LINEAR, HEADING_BRAKING_QUADRATIC));

            c.linearBrakeCoefficients.set(Matrix.diag(FORWARD_BRAKING_LINEAR, STRAFE_BRAKING_LINEAR));
            c.quadraticBrakeCoefficients.set(Matrix.diag(FORWARD_BRAKING_QUADRATIC, STRAFE_BRAKING_QUADRATIC));

            c.maxAchievableForwardVelocity.set(MAX_FORWARD_VELOCITY);
            c.maxAchievableStrafeVelocity.set(MAX_STRAFE_VELOCITY);
            c.naturalForwardDeceleration.set(NATURAL_FORWARD_DECELERATION);
            c.naturalStrafeDeceleration.set(NATURAL_STRAFE_DECELERATION);
        });
    }

    public static Localizer createLocalizer(HardwareMap hardwareMap) {
        return new PinpointLocalizer(hardwareMap, localizerConfig);
    }

    public static Drivetrain createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    /** El Follower de Pedro, o null si Foresight todavía no está tuneado. */
    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        if (!FORESIGHT_TUNED) return null;
        return new Follower(localizer, createDrivetrain(hardwareMap), new Foresight(foresightConfig()));
    }
}
