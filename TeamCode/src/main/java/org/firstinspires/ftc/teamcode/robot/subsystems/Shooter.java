package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Flywheel shooter. Spins up to a target velocity and reports when it is ready to fire.
 */
public class Shooter extends SubsystemBase {
    // TODO: set to the names in the robot configuration
    public static final String MOTOR_NAME = "shooter";

    // TODO: tune
    public static final double VELOCITY_TOLERANCE = 0.0;

    private double targetVelocity = 0.0;

    public Shooter(HardwareMap hardwareMap) {
        // TODO: get shooter motor(s) from hardwareMap and configure velocity control
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        // TODO: command motor velocity
    }

    public void stop() {
        setTargetVelocity(0.0);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocity() {
        // TODO: read motor velocity
        return 0.0;
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(getVelocity() - targetVelocity) <= VELOCITY_TOLERANCE;
    }

    @Override
    public void periodic() {
        // TODO: update velocity controller / telemetry
    }
}
