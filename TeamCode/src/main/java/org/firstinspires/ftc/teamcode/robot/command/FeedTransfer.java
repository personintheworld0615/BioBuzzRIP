package org.firstinspires.ftc.teamcode.robot.command;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystem.Transfer;

/**
 * Feeds game pieces from the transfer into the shooter until interrupted.
 */
public class FeedTransfer extends CommandBase {
    private final Transfer transfer;

    public FeedTransfer(Transfer transfer) {
        this.transfer = transfer;
        addRequirements(transfer);
    }

    @Override
    public void initialize() {
        transfer.feed();
    }

    @Override
    public boolean isFinished() {
        // TODO: finish once the game piece has left the transfer, if a sensor is added
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        transfer.stop();
    }
}
