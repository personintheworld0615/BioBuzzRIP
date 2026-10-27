package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;

/**
 * Stops the shooter flywheel.
 */
public class StopShooter extends InstantCommand {
    public StopShooter(Shooter shooter) {
        super(shooter::stop, shooter);
    }
}
