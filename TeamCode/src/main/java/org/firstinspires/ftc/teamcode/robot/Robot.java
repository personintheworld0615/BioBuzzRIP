package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.Constants;

public class Robot {
    private Follower follower;
    public Robot(){
        follower = Constants.create(hardwareMap);

    }
    private void init(){
        
    }
}
