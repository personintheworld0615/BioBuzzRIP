package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Drivetrain {
    private final Follower follower = null;

    private static double kp = 3.0;


    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;



    public Drivetrain(Follower follower) {

        rightBack = hardwareMap.get(DcMotor.class, "backRightMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightMotor");
        leftBack = hardwareMap.get(DcMotor.class, "backLeftMotor");
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftMotor");

    }



    // ----------------------------------------
    // MANUAL DRIVE (converted from your old Drive.java)
    // ----------------------------------------
    public void manualDrive(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        leftFront.setPower((y + x + rx) / denominator);
        leftBack.setPower((y - x + rx) / denominator);
        rightFront.setPower((y - x - rx) / denominator);
        rightBack.setPower((y + x - rx) / denominator);
    }
    public Pose getPose(){
        return follower.pose();
    }

    public void combinedDrive(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        follower.manual(forward, lateral, turn);
        follower.update();

    }




}

