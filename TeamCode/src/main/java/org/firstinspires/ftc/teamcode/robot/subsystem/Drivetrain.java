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


    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }




    public void combinedDrive(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double lateral = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        follower.manual(forward, lateral, turn);

    }
    public void manual(Gamepad gamepad) {
        double forward = -gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = gamepad.right_stick_x;

        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn),
                1.0
        );

        double frontLeftPower = (forward + strafe + turn) / denominator;
        double backLeftPower = (forward - strafe + turn) / denominator;
        double frontRightPower = (forward - strafe - turn) / denominator;
        double backRightPower = (forward + strafe - turn) / denominator;

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public String getTelemetry() {
        return "LF power: " + leftFront.getPower() + " vel: " + leftFront.getVelocity() + "\n"
                + "RF power: " + rightFront.getPower() + " vel: " + rightFront.getVelocity() + "\n"
                + "LB power: " + leftBack.getPower() + " vel: " + leftBack.getVelocity() + "\n"
                + "RB power: " + rightBack.getPower() + " vel: " + rightBack.getVelocity();
    }




}

