package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret{
    Servo servo;
    Follower follower;
    Pose goal;
    public Turret(Follower follower){
        this.follower = follower;
        servo = hardwareMap.get(Servo.class,"Turret");
        goal = new Pose(10,10);
    }
    public void targetToGoal(){
       double error = getError();
        double ticks = errorToServo(error);
        servo.setPosition(ticks);
    }

    private double errorToServo(double error){
        double zeroPoint = 0.5; //adjust these
        double ticks = (error/360) +zeroPoint;
        return ticks;
    }
    private double getError(){
        double angletoGoal = getAngleRequired();
        Pose currentPose= follower.pose();
        double heading= currentPose.heading();
        heading = radtoDegrees(heading);
        double erro = angletoGoal-heading;
        if(erro>180){
            erro -=360;
        }
        if(erro<-180){
            erro+=360;
        }
        return erro;

    }
    private double getAngleRequired(){
         Pose pose = follower.pose();
         double dletax = goal.x() - pose.x();
         double detlay = goal.y()- pose.y();
         return radtoDegrees(Math.atan2(detlay,dletax)); //converts it to 0-360
    }
    public Command aimCommand() {
        return Command.build()
                .setExecute(this::targetToGoal)
                .requiring(servo);
    }

    public Command manualCommand() {
        return Command.build()
                .setExecute(() -> servo.setPosition(0.5))
                .requiring(servo);
    }
    private double radtoDegrees(double radians) {
        double degrees = Math.toDegrees(radians);
        return ((degrees % 360) + 360) % 360;//converts to 0-360
    }
}

