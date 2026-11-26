package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.loop;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;


import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystem.ColorSensor;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

public class Robot {

    //Subsystems
    public Shooter shooter;
    public Intake intake;
    public Turret turret;
    public ColorSensor colorSensor;
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
        colorSensor = new ColorSensor(hardwareMap);
        shooter = new Shooter(hardwareMap, follower, red);
        intake = new Intake(hardwareMap, driver, operator);
         turret = new Turret(follower, hardwareMap, red);
        drivetrain = new Drivetrain(hardwareMap, this.follower);
        
        initializeCommands();
    }


    // Initialize all commands once
    private void initializeCommands() {

        // Intake (Driver gamepad)
        intakeCommand = sequential(
                intake.intake().until(
                        () -> colorSensor.isColor(ColorSensor.DetectedColor.YELLOW)
                ),
                conditional(
                        () -> operator.left_trigger > 0.5,
                        intake.intake2(),
                        instant(()->intake.stop())
                )
        );

        reverseCommand = intake.reverse()
                .until(() -> operator.leftBumperWasReleased());

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
                    intake.setPowers(1.0, -0.5);
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

        if (operator.left_trigger > 0.5 && !Scheduler.isScheduled(intakeCommand)) {
            Scheduler.schedule(intakeCommand);

        }

        if (operator.leftBumperWasPressed()) {
            Scheduler.schedule(reverseCommand);
        }

        if (!Scheduler.isScheduled(turretCommand)) {
            Scheduler.schedule(turretCommand);
        }


        if (operator.right_trigger>0.5&&!shooterStuff.isScheduled()) {
            shooterStuff.schedule();
        }
    }
    public Command autonShootSequence(boolean stopFlywheelAfter) {
        ElapsedTime prepTimer = new ElapsedTime();
        ElapsedTime feedTimer = new ElapsedTime();

        Command prep = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, -0.5);
                    shooter.spinToRpm();
                })
//                .setPriority(50)
                .requiring(intake, shooter)
                .until(() -> shooter.isRPMGood() || prepTimer.milliseconds() > 1500);
        waitMs(500);
        Command shoot = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, 1.0);
                    shooter.spinToRpm();
                })
                .setEnd(endCondition -> {
                    intake.setPowers(0, 0);
                    if (stopFlywheelAfter) shooter.stop();
                })
//                .setPriority(51) lwk don't need the priorities cus they're inside sequrential
                .requiring(intake, shooter)
                .until(() -> feedTimer.milliseconds() > 1200);

        return sequential(
                instant(prepTimer::reset),//just to be
                prep,
                instant(()->feedTimer.reset()),// bipolar like you
                shoot
        );
    }

    public Command autoIntakeCommand(double wait) {
        return race(
                sequential(
                intake.intake().until(
                        () -> colorSensor.isColor(ColorSensor.DetectedColor.YELLOW)
                ),
                        intake.intake2()
        ),
        waitMs(wait)
        );
    }
    public void update() {
        follower.update();
        MainLoop();
        Scheduler.execute();
    }
    public void autoUpdate() {
        follower.update();
        Scheduler.execute();
    }
}