package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Transfer;

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
