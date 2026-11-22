
package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class Turret {
    Servo servo;
    Follower follower;

    Pose goal;
    Pose topGoal;
    Pose bottomGoal;

    public static double SERVO_CENTER = 0.5;
    public static double SERVO_RANGE_DEG = 360;
    public static boolean SERVO_REVERSED = false;

    // YO YOU GOTTA PASS IN HARDWAREMAP
    public Turret(Follower follower, HardwareMap hw, boolean red) {
        this.follower = follower;
        servo = hw.get(Servo.class, "Turret");

        if (red) {
            topGoal = new Pose(58, 86);
            bottomGoal = new Pose(58, 55);
        } else {
            topGoal = new Pose(82, 86);
            bottomGoal = new Pose(82, 55);
        }
    }

    public void targetToGoal() {
        double error = getError();
        double ticks = errorToServo(error);
        servo.setPosition(ticks);
    }

    private double errorToServo(double error) {
        // this part is the only thing im not confident on
        double direction = SERVO_REVERSED ? -1 : 1;
        double ticks = Range.clip(
                SERVO_CENTER + direction * error / SERVO_RANGE_DEG,
                0,
                1
        );
        return ticks;
    }


    public double getAimAngleDeg() {
        return getError();
    }

    public double getServoPosition() {
        return servo.getPosition();
    }

    private double getError() {
        double angleToGoal = getAngleRequired();

        Pose currentPose = follower.pose();
        double heading = radtoDegrees(currentPose.heading());

        double error = angleToGoal - heading;

        if (error > 180) {
            error -= 360;
        }

        if (error < -180) {
            error += 360;
        }

        return error;
    }

    private double getAngleRequired() {
        Pose currentPose = follower.pose();

        // Midline is y = 70.
        // Above midline -> top goal.
        // Below midline -> bottom goal.
        if (currentPose.y() > 70) {
            goal = topGoal;
        } else {
            goal = bottomGoal;
        }

        double diffX = goal.x() - currentPose.x();
        double diffY = goal.y() - currentPose.y();

        return radtoDegrees(Math.atan2(diffY, diffX));
    }

    public Command aimCommand() {
        return Command.build()
                .setExecute(this::targetToGoal)
                .requiring(servo);
    }

    public Command defaultCommand() {
        return Command.build()
                .setExecute(() -> servo.setPosition(SERVO_CENTER))
                .requiring(servo);
    }


    private double radtoDegrees(double radians) {
        double degrees = Math.toDegrees(radians);
        return ((degrees % 360) + 360) % 360;
    }
}
