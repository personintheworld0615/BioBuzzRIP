package org.firstinspires.ftc.teamcode.robot.subsystem;


import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Drivetrain {
    private Follower follower;

    private static double kp = 3.0;


    public DcMotorEx leftFront;
    public DcMotorEx rightFront;
    public DcMotorEx leftBack;
    public DcMotorEx rightBack;



    public Drivetrain(HardwareMap hardwareMap, Follower follower) {

        rightBack = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        leftBack = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        this.follower = follower;

    }


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

    public void drive(Gamepad gamepad1){
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x*1.1, -gamepad1.right_stick_x);
    }
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }


    //kind or redundant because any class that passes in follower (which is required to pass in) can jsut use follower.pose
    public Pose getPose(){
        return follower.pose();
    }


    // WTH is the pnt of this?
    public void combinedDrive(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        follower.manual(forward, lateral, turn);
        follower.update();

    }




}

