package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;

/**
 * Sets the shooter's target velocity and finishes once the flywheel is at speed.
 * The flywheel keeps spinning after this ends; use {@link StopShooter} to stop it.
 */
public class SpinUpShooter extends CommandBase {
    private final Shooter shooter;
    private final double velocity;

    public SpinUpShooter(Shooter shooter, double velocity) {
        this.shooter = shooter;
        this.velocity = velocity;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setTargetVelocity(velocity);
    }

    @Override
    public boolean isFinished() {
        return shooter.isAtTargetVelocity();
    }
}
