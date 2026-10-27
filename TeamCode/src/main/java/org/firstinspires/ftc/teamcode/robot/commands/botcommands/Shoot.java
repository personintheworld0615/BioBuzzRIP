package org.firstinspires.ftc.teamcode.robot.commands.botcommands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.FeedTransfer;
import org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands.SpinUpShooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.Transfer;

/**
 * Spins the shooter up, then feeds the transfer for a fixed time.
 */
public class Shoot extends SequentialCommandGroup {
    // TODO: tune
    public static final long FEED_TIME_MS = 500;

    public Shoot(Shooter shooter, Transfer transfer, double velocity) {
        addCommands(
                new SpinUpShooter(shooter, velocity),
                new FeedTransfer(transfer).withTimeout(FEED_TIME_MS)
        );
    }
}
