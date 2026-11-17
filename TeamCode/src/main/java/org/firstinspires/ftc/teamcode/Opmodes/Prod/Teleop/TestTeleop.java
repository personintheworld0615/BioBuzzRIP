package org.firstinspires.ftc.teamcode.Opmodes.Prod.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "RealTeleop")
public class TestTeleop extends LinearOpMode {
    public static final double LIMIT = 0.99; //Limit to the motor power.
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftFront, leftBack, rightFront, rightBack;


        leftFront = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftBack = hardwareMap.get(DcMotor.class, "backLeftMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightMotor");
        rightBack = hardwareMap.get(DcMotor.class, "backRightMotor");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        telemetry.addLine("Waiting for Start");
        while(opModeIsActive()){

            //loop
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

            double frontLeftPower = ((y + x + rx) / denominator) * LIMIT;
            double backLeftPower = ((y - x + rx) / denominator) * LIMIT;
            double frontRightPower = ((y - x - rx) / denominator) * LIMIT;
            double backRightPower = ((y + x - rx) / denominator) * LIMIT;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);
            telemetry.addData("Limit: ", LIMIT);
            telemetry.addData("FL Power: ", frontLeftPower);
            telemetry.addData("BL Power: ", backLeftPower);
            telemetry.addData("FR Power: ", frontRightPower);
            telemetry.addData("BR Power: ", backRightPower);

        }

    }



}
