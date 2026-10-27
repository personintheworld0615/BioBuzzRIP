package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Picks game pieces up off the field.
 */
public class Intake extends SubsystemBase {
    // TODO: set to the names in the robot configuration
    public static final String MOTOR_NAME = "intake";

    // TODO: tune
    public static final double INTAKE_POWER = 1.0;

    public Intake(HardwareMap hardwareMap) {
        // TODO: get intake motor from hardwareMap
    }

    public void intake() {
        setPower(INTAKE_POWER);
    }

    public void outtake() {
        setPower(-INTAKE_POWER);
    }

    public void stop() {
        setPower(0.0);
    }

    private void setPower(double power) {
        // TODO: command intake motor
    }

    @Override
    public void periodic() {
        // TODO: telemetry
    }
}
