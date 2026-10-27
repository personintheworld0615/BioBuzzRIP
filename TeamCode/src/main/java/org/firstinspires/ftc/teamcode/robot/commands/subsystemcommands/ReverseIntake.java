package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

/**
 * Runs the intake outward (spits game pieces out) until interrupted.
 */
public class ReverseIntake extends CommandBase {
    private final Intake intake;

    public ReverseIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.outtake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
