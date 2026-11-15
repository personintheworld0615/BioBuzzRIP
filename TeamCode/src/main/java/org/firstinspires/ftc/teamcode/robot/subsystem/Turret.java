package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret{
    Servo servo;
    Follower follower;
    Pose goal;

    // Tune these with ShooterTest, then copy them in here.
    // Servo position where the turret points straight forward.
    public static double SERVO_CENTER = 0.5;
    // Turret degrees covered by the servo's full 0 to 1 travel.
    public static double SERVO_RANGE_DEG = 360;
    // Tick if a positive (counterclockwise) angle turns the turret clockwise.
    public static boolean SERVO_REVERSED = false;
    // Share of the camera's bearing added to the vision trim per frame. Lower it if the turret
    // overshoots, the servo is still moving when the next frame arrives.
    public static double VISION_GAIN = 0.5;
    // Largest correction vision may add on top of the odometry aim.
    public static double VISION_MAX_TRIM_DEG = 30;

    private double visionTrimDeg = 0;



    // YO YOU GOTTA PASS IN HARDAREMAP :SOB
    public Turret(Follower follower, HardwareMap hw){
        this.follower = follower;
        servo = hw.get(Servo.class,"Turret");
        goal = new Pose(10,10); // IS THIS RIGHT?
    }

    public void targetToGoal(){
       double error = getError();
        double ticks = errorToServo(error);
        servo.setPosition(ticks);
    }

    private double errorToServo(double error){
        double direction = SERVO_REVERSED ? -1 : 1;
        double ticks = Range.clip(SERVO_CENTER + direction * error / SERVO_RANGE_DEG, 0, 1); // it shoudln't ever be greater than 1 if the math worsk out and the error clips properly, but just in case
        return ticks;
    }

    public void setGoal(Pose goal) {
        this.goal = goal;
    }

    // Turret angle from robot-forward that points at the goal, counterclockwise positive.
    public double getAimAngleDeg() {
        return getError();
    }

    public double getVisionTrimDeg() {
        return visionTrimDeg;
    }

    public double getServoPosition() {
        return servo.getPosition();
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
         double diffX = goal.x() - pose.x();
         double diffY = goal.y()- pose.y();
         return radtoDegrees(Math.atan2(diffY,diffX)); // atan2 converts to degrees using trig, and then radToDegrees turnit into rads
    }
    public Command aimCommand() {
        return Command.build()
                .setExecute(this::targetToGoal) // why are WE so bipolar bro ong you use this access thing her and then for manual command you use ()->
                .requiring(servo);
    }
    // Aims with odometry, then trims out what odometry gets wrong using the camera. The camera
    // rides on the turret, so its bearing to the hive is the turret's remaining aim error.
    public Command aimWithVisionCommand(Camera camera) {
        return Command.build()
                .setStart(() -> visionTrimDeg = 0)
                .setExecute(() -> {
                    if (camera.hasFreshHiveTarget()) {
                        double trim = visionTrimDeg + camera.getHiveBearingDeg() * VISION_GAIN;
                        visionTrimDeg = Range.clip(trim, -VISION_MAX_TRIM_DEG, VISION_MAX_TRIM_DEG);
                    }
                    servo.setPosition(errorToServo(getError() + visionTrimDeg));
                })
                .setEnd(endCondition -> visionTrimDeg = 0)
                .requiring(servo);
    }

    public Command defaultCommand(){
        return Command.build()
                .setExecute(() -> servo.setPosition(SERVO_CENTER))
                .requiring(servo);
    }

    public Command manualCommand(int pos) {
        return Command.build()
                .setExecute(() -> servo.setPosition(pos))
                .requiring(servo);
    }

    // YO ARAV WHY THE FUCK DO WE HAVE OUR OWN CONVERSION FUNCTION BRO TELL ME THIS SHIT AINT AI
    private double radtoDegrees(double radians) {
        double degrees = Math.toDegrees(radians);
        return ((degrees % 360) + 360) % 360;//converts to 0-360
    }
}

