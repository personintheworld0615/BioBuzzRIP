package org.firstinspires.ftc.teamcode.Opmodes.Prod;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name = "Teleop Blue", group = "Competition")
public class TeleopBlue extends LinearOpMode {
    private Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.create(hardwareMap);
        Robot robot = new Robot(
                hardwareMap,
                follower,
                gamepad1,
                gamepad2
        );

        waitForStart();

        while (opModeIsActive()) {
            robot.update();
        }
    }
}