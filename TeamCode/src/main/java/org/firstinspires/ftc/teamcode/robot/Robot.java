package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.loop;
import static com.pedropathing.ivy.groups.Groups.sequential;


import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

public class Robot {

    // Subsystems
    public Shooter shooter;
    public Intake intake;
    public Turret turret;
    public Drivetrain drivetrain;
    public Follower follower;

    // Gamepads
    private Gamepad driver;
    private Gamepad operator;

    // Commands
    private Command intakeCommand;
    private Command reverseCommand;

     private Command turretCommand;
     private Command drive;

     private Command prepshootingtime;
     private Command shoot;
     private Command shooterStuff;


    // Constructor
    public Robot(HardwareMap hardwareMap, Follower follower,
                 Gamepad gamepad1, Gamepad gamepad2, boolean red) {

        driver = gamepad1;
        operator = gamepad2;
        this.follower = follower;
        this.follower.setPose(new Pose(72,72,0));

        shooter = new Shooter(hardwareMap, follower, red);
        intake = new Intake(hardwareMap, driver, operator);
         turret = new Turret(follower, hardwareMap, red);
        drivetrain = new Drivetrain(hardwareMap, this.follower);
        
        initializeCommands();
    }


    // Initialize all commands once
    private void initializeCommands() {

        // Intake (Driver gamepad)
        intakeCommand = intake.intake();

        reverseCommand = intake.reverse()
                .until(() -> driver.leftBumperWasReleased());

        turretCommand = loop(sequential(
                turret.defaultCommand().until(() -> driver.rightBumperWasPressed()),
                turret.aimCommand().until(() -> driver.rightBumperWasPressed())
        ));


        // Drivetrain
        drive = loop(infinite(
                () -> drivetrain.combinedDrive(driver)
        ));

        prepshootingtime = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, -0.3);
                    shooter.spinToRpm();
                })
                .setPriority(50)
                .requiring(intake, shooter);

        shoot = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, 1.0);
                    shooter.spinToRpm();
                })
                .setEnd(endCondition -> {
                    intake.setPowers(0, 0);
                    shooter.stop();
                })
                .setPriority(50)
                .requiring(intake, shooter);

        shooterStuff = sequential(
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
                            shooter.stop();
                        })
                )
        );
    }


    // Handle gamepad inputs
    public void MainLoop() {

        // Drivetrain
        if (!Scheduler.isScheduled(drive)) {
            Scheduler.schedule(drive);
        }

        // Intake (Driver left trigger)
        if (driver.left_trigger > 0.5 && !Scheduler.isScheduled(intakeCommand)) {
            Scheduler.schedule(intakeCommand);

        }

        // Reverse intake (Driver left bumper)
        if (driver.leftBumperWasPressed()) {
            Scheduler.schedule(reverseCommand);
        }

        if (!Scheduler.isScheduled(turretCommand)) {
            Scheduler.schedule(turretCommand);
        }


        if (operator.right_trigger>0.5&&!shooterStuff.isScheduled()) {
            shooterStuff.schedule();
        }
    }


    public void update() {
        follower.update();
        MainLoop();
        Scheduler.execute();
    }
}