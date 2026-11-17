
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

    private Command turretStuff;
    private Command turretFinal;
    private Command drive;

    private Command prepshootingtime;
    private Command shoot;
    private Command outtakestuff;



    // Constructor
    public Robot(HardwareMap hardwareMap, Follower follower, Gamepad gamepad1, Gamepad gamepad2) {

        driver = gamepad1;
        operator = gamepad2;
        this.follower = follower;

        shooter = new Shooter(hardwareMap);
        intake = new Intake(driver, operator);
        turret = new Turret(follower, hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, this.follower);

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
                turret.defaultCommand().until(() -> driver.rightBumperWasPressed()),
                turret.aimCommand().until(() -> driver.rightBumperWasPressed())
        );

        turretFinal = loop(turretStuff);


        // Drivetrain
        drive = loop(infinite(
                () -> drivetrain.manualDrive(driver)
        ));


        // Shooting preparation
        prepshootingtime = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, -0.3);
                    shooter.spinToRpm(3200);
                })
                .setPriority(50)
                .requiring(intake, shooter);


        // Shooting
        shoot = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, 1.0);
                    shooter.spinToRpm(3200);
                })
                //TODO: conditional intake & transfer spin up which depends on calculated requried PRM being atl somewhat reached
                .setEnd(endCondition -> {
                    intake.setPowers(0, 0);
                    shooter.stop();
                })
                .setPriority(50)
                .requiring(intake, shooter);


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
                            shooter.stop();
                        })
                )
        );
    }


    // Handle gamepad inputs
    public void MainLoop() {



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

    public void update() {
//        follower.update();
        MainLoop();
        Scheduler.execute();
    }

}
