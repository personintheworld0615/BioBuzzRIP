package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private final Follower follower;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public DriveTrain(Follower follower, Gamepad gamepad1, Gamepad gamepad2) {
        this.follower = follower;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Drives from gamepad1 sticks. Runs until interrupted; use as the default command in TeleOp.
     */
    public Command drive() {
        return new RunCommand(() -> follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        ), this);
    }

    public Follower getFollower() {
        return follower;
    }

    @Override
    public void periodic() {
        // Runs every loop regardless of which command owns the drivetrain, so both
        // drive() and SolversLib's pedroCommand (FollowPathCommand, etc.) get updated.
        follower.update();
    }
}
