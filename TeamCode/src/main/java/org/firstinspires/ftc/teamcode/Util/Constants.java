package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Constants {
    public static double kP = 0.05;
    public static double kI = 0.05;
    public static double kD = 0.05;
    public static double kF = 0.000021;

    // Shooter lookup table: distance to the hive (in) -> RPM. A slot with an RPM of 0 is empty.
    // Fill it with ShooterTest, then copy the arrays it prints into here; Dashboard edits are
    // lost when the robot restarts.
    private static final int MAX_LOOKUP_POINTS = 10;
    public static double[] SHOOTER_DISTANCES = new double[MAX_LOOKUP_POINTS];
    public static double[] SHOOTER_RPMS = new double[MAX_LOOKUP_POINTS];

    // Recording within this many inches of an existing point overwrites it instead of adding one.
    public static double SAME_POINT_TOLERANCE_IN = 3;

    // Field geometry from the BIOBUZZ manual (section 9), in Pedro inches: origin at the
    // audience-side corner on the red side, +x toward the blue wall, +y away from the audience.
    // The HIVE pivots are 25.5 in apart at field center, red on the red side.
    public static double RED_HIVE_X = 59.25;
    public static double BLUE_HIVE_X = 84.75;
    // The goal is whichever CELL is up: the starting CELL until the hive tips, then the
    // secondary one. Red starts on its audience-side CELL and blue on its far one (Figure 9-17).
    // Scaled from Figure 9-10; tune these on the real field.
    public static double RED_STARTING_CELL_Y = 56;
    public static double RED_SECONDARY_CELL_Y = 88;
    public static double BLUE_STARTING_CELL_Y = 88;
    public static double BLUE_SECONDARY_CELL_Y = 56;

    // Pedro setup. Every value is a placeholder until the robot exists: device names and
    // directions must match the real config, and the Foresight numbers must be tuned before
    // running paths. Driving with follower.manual() only needs the names and directions.
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig()),
                new Mecanum(hardwareMap, drivetrainConfig()),
                new Foresight(algorithmConfig()));
    }

    private static PinpointConfig localizerConfig() {
        return new PinpointConfig(config -> {
            config.name.set("odo");
            config.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            config.xPodOffset.set(0.0);
            config.yPodOffset.set(0.0);
        });
    }

    private static MecanumConfig drivetrainConfig() {
        return new MecanumConfig(config -> {
            config.frontLeftName.set("frontLeftMotor");
            config.backLeftName.set("backLeftMotor");
            config.frontRightName.set("frontRightMotor");
            config.backRightName.set("backRightMotor");
            config.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            config.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            config.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            config.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        });
    }

    private static ForesightConfig algorithmConfig() {
        return new ForesightConfig(config -> {
            config.headingFeedback.set(Controller.pid(1, 0, 0));
            config.forwardTranslational.set(Controller.pid(0.1, 0, 0));
            config.strafeTranslational.set(Controller.pid(0.1, 0, 0));
            config.brake.set(Controller.pid(0.1, 0, 0));
            config.coast.set(Controller.pid(0.1, 0, 0));
            config.linearBrakeCoefficients.set(Matrix.diag(0.1, 0.1));
            config.quadraticBrakeCoefficients.set(Matrix.diag(0.01, 0.01));
            config.headingBrakeCoefficients.set(Vector2D.cartesian(0.1, 0.01));
            config.maxAchievableForwardVelocity.set(60.0);
            config.maxAchievableStrafeVelocity.set(45.0);
            config.naturalForwardDeceleration.set(50.0);
            config.naturalStrafeDeceleration.set(70.0);
        });
    }
}
