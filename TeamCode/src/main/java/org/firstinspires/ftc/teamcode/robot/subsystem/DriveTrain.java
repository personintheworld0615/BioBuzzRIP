package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveTrain {
    Follower follower;
    Gamepad gamepad1;
    Gamepad gamepad2;
    public DriveTrain(Follower follower, Gamepad gamepad1, Gamepad gamepad2){
        this.follower = follower;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }
    public Command drive(){
        Command driverobot = Command.build()
                .setExecute(() -> {
                    follower.manual(
                            -gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    );
                    follower.update();
                });
        return driverobot;
    }

}
