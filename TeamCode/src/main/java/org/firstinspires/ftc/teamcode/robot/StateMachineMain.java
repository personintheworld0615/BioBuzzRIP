
package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.loop;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

public class StateMachineMain {

    // Subsystems
    public Outtake outtake;
    public Intake intake;
    public Turret turret;
    public Drivetrain drivetrain;

    // Gamepads
    private Gamepad driver;
    private Gamepad operator;

    // Commands
    private Command intakeCommand;
    private Command reverseCommand;

    private Command turretStuff;
    private Command turretFinal;
    private Command drive;

    private Command prepshootingtime;
    private Command shoot;
    private Command outtakestuff;


    // Constructor
    public StateMachineMain(Follower follower, Gamepad gamepad1, Gamepad gamepad2) {

        driver = gamepad1;
        operator = gamepad2;

        outtake = new Outtake();
        intake = new Intake(driver, operator);
        turret = new Turret(follower);
        drivetrain = new Drivetrain(follower);

        initializeCommands();
    }


    // Initialize all commands once
    private void initializeCommands() {

        // Intake
        intakeCommand = intake.intake();

        reverseCommand = intake.reverse()
                .until(() -> operator.leftBumperWasReleased());


        // Turret
        turretStuff = sequential(
                turret.manualCommand().until(() -> driver.rightBumperWasPressed()),
                turret.aimCommand().until(() -> driver.rightBumperWasPressed())
        );

        turretFinal = loop(turretStuff);


        // Drivetrain
        drive = loop(infinite(
                () -> drivetrain.combinedDrive(driver)
        ));


        // Shooting preparation
        prepshootingtime = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, -0.3);
                    outtake.spinToRpm(3200);
                })
                .setPriority(50)
                .requiring(intake, outtake);


        // Shooting
        shoot = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, 1.0);
                    outtake.spinToRpm(3200);
                })
                .setEnd(endCondition -> {
                    intake.setPowers(0, 0);
                    outtake.stop();
                })
                .setPriority(50)
                .requiring(intake, outtake);


        // Complete shooting sequence
        outtakestuff = sequential(

                prepshootingtime.until(() ->
                        operator.right_bumper ||
                                operator.right_trigger <= 0.5
                ),

                conditional(
                        () -> operator.right_bumper &&
                                operator.right_trigger > 0.5,

                        shoot.until(() -> !operator.right_bumper),

                        instant(() -> {
                            intake.stop();
                            outtake.stop();
                        })
                )
        );
    }


    // Handle gamepad inputs
    public void Placeholder(Gamepad gamepad1, Gamepad gamepad2) {



        if (!Scheduler.isScheduled(turretFinal)) {
            Scheduler.schedule(turretFinal);
        }

        if (!Scheduler.isScheduled(drive)) {
            Scheduler.schedule(drive);
        }


        if (operator.left_trigger > 0.5 &&
                !Scheduler.isScheduled(intakeCommand)) {

            Scheduler.schedule(intakeCommand);
        }


        // Reverse
        if (operator.leftBumperWasPressed()) {
            Scheduler.schedule(reverseCommand);
        }


        // Shooting
        if (operator.right_trigger>0.5&&!outtakestuff.isScheduled()) {
            outtakestuff.schedule();
        }
    }
}
