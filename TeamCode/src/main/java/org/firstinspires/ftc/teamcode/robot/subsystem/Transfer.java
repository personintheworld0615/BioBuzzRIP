package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Moves game pieces from the intake into the shooter.
 */
public class Transfer extends SubsystemBase {
    // TODO: set to the names in the robot configuration
    public static final String MOTOR_NAME = "transfer";

    // TODO: tune
    public static final double FEED_POWER = 1.0;

    public Transfer(HardwareMap hardwareMap) {
        // TODO: get transfer motor/servo and any game piece sensor from hardwareMap
    }

    public void feed() {
        setPower(FEED_POWER);
    }

    public void reverse() {
        setPower(-FEED_POWER);
    }

    public void stop() {
        setPower(0.0);
    }

    public boolean hasGamePiece() {
        // TODO: read game piece sensor
        return false;
    }

    private void setPower(double power) {
        // TODO: command transfer motor/servo
    }

    @Override
    public void periodic() {
        // TODO: telemetry
    }
}
