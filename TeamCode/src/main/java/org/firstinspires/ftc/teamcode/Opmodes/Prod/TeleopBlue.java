package org.firstinspires.ftc.teamcode.Opmodes.Prod;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Teleop Blue", group = "Competition")
public class TeleopBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot main = new Robot(
                hardwareMap,
                null,
                gamepad1,
                gamepad2
        );

        waitForStart();

        while (opModeIsActive()) {
            main.update();
        }
    }
}