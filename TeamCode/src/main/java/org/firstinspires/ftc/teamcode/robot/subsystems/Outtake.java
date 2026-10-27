package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Outtake {
    Follower follower;
    Gamepad gamepad1;
    Gamepad gamepad2;

    public Outtake(Follower follower, Gamepad gamepad1, Gamepad gamepad2){
        this.follower = follower;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }
    public Command fullyOutake(){
        return null;
    }

}
