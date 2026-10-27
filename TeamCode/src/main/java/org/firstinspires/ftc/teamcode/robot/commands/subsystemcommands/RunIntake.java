package org.firstinspires.ftc.teamcode.robot.commands.subsystemcommands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;

/**
 * Runs the intake inward until interrupted.
 */
public class RunIntake extends CommandBase {
    private final Intake intake;

    public RunIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake();
    }

    @Override
    public boolean isFinished() {
        // TODO: finish once a game piece is detected, if desired
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
