package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Flywheel shooter. Spins up to a target velocity and reports when it is ready to fire.
 */
public class Shooter extends SubsystemBase {
    // TODO: set to the names in the robot configuration
    public static final String MOTOR_NAME = "shooter";

    // TODO: tune
    public static final double VELOCITY_TOLERANCE = 0.0;
    public static final long FEED_TIME_MS = 500;

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

    /**
     * Sets the target velocity and finishes once the flywheel is at speed.
     * The flywheel keeps spinning after this ends; use {@link #stopShooter()} to stop it.
     */
    public Command spinUp(double velocity) {
        return new FunctionalCommand(
                () -> setTargetVelocity(velocity),
                () -> { },
                interrupted -> { },
                this::isAtTargetVelocity,
                this
        );
    }

    /**
     * Stops the flywheel.
     */
    public Command stopShooter() {
        return new InstantCommand(this::stop, this);
    }

    /**
     * Spins the shooter up, then feeds the transfer for {@link #FEED_TIME_MS}.
     */
    public Command shoot(Transfer transfer, double velocity) {
        return new SequentialCommandGroup(
                spinUp(velocity),
                transfer.feedTransfer().withTimeout(FEED_TIME_MS)
        );
    }

    @Override
    public void periodic() {
        // TODO: update velocity controller / telemetry
    }
}
