package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
@Config
@TeleOp(name = "Turret Tester", group = "Testers")
public class TurretTester extends LinearOpMode {
    public static boolean red = false;
    @Override
    public void runOpMode() {

        // Initialize follower using your Constants
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72,72,0));

        // Initialize turret
        Turret turret = new Turret(follower, hardwareMap, red);
        Drivetrain drivetrain = new Drivetrain(hardwareMap,follower);
        // Driver Station + Dashboard telemetry
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        telemetry.addLine("========== TURRET TESTER ==========");
        telemetry.addLine("Initialization complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update Pedro localization
            follower.update();

            // Update turret aiming
            turret.targetToGoal();
            drivetrain.combinedDrive(gamepad1);
            Pose pose = follower.pose();

            double robotX = pose.x();
            double robotY = pose.y();
            double headingRad = pose.heading();
            double headingDeg = Math.toDegrees(headingRad);

            double aimAngle = turret.getAimAngleDeg();
            double servoPosition = turret.getServoPosition();

            telemetry.addLine("========== ROBOT LOCALIZATION ==========");
            telemetry.addData("Robot X", "%.2f", robotX);
            telemetry.addData("Robot Y", "%.2f", robotY);
            telemetry.addData("Heading (Radians)", "%.4f", headingRad);
            telemetry.addData("Heading (Degrees)", "%.2f", headingDeg);

            telemetry.addLine("========== TARGET SELECTION ==========");
            telemetry.addData("Selected Goal",
                    robotY > 70 ? "TOP" : "BOTTOM");
            telemetry.addData("Goal X",
                    robotY > 70 ? 58 : 82);
            telemetry.addData("Goal Y",
                    robotY > 70 ? 86 : 55);

            telemetry.addLine("========== TURRET AIMING ==========");
            telemetry.addData("Required Field Angle", "%.2f°",
                    aimAngle + headingDeg);
            telemetry.addData("Turret Error", "%.2f°", aimAngle);
            telemetry.addData("Servo Position", "%.4f", servoPosition);
            telemetry.addData("Calculated Servo Target", "%.4f",
                    0.5 + (Turret.SERVO_REVERSED ? -1 : 1)
                            * aimAngle / Turret.SERVO_RANGE_DEG);

            telemetry.update();
        }
    }
}