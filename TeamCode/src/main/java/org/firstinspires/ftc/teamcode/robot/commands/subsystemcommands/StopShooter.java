package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

/**
 * Stops the shooter flywheel.
 */
public class StopShooter extends InstantCommand {
    public StopShooter(Shooter shooter) {
        super(shooter::stop, shooter);
    }
}
