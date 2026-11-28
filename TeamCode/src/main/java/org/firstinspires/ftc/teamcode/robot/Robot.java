package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.deadline;
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
    private Command flowerstuff;

    private boolean turretOff = false;


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
        intakeCommand = intake.intake();

        reverseCommand = intake.reverse()
                .until(() -> operator.rightBumperWasReleased());

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
                .setDone(
                        ()-> operator.right_trigger<=0.5
                )
                .setEnd(endCondition -> {
                    intake.setPowers(0, 0);
                    shooter.stop();
                })
                .setPriority(50)
                .requiring(intake, shooter);

        shooterStuff = race(
                sequential(
                        prepshootingtime.until(() ->
                                shooter.isRPMGood()||
                                        operator.right_trigger <= 0.5
                        ),

                        conditional(
                                () -> operator.right_trigger > 0.5,

                                shoot.until(() -> operator.right_trigger <= 0.5),

                                instant(() -> {
                                    intake.stop();
                                    shooter.stop();
                                })
                        )
                ),

                turret.aimCommand().until(() -> operator.right_trigger <= 0.5).unless(()-> turretOff)
        );
    }


    // Handle gamepad inputs
    public void MainLoop() {

        // Drivetrain
        if (!Scheduler.isScheduled(drive)) {
            Scheduler.schedule(drive);
        }

        if (operator.right_trigger > 0.5 && !Scheduler.isScheduled(intakeCommand)) {
            Scheduler.schedule(intakeCommand);

        }

        if (operator.rightBumperWasPressed()) {
            Scheduler.schedule(reverseCommand);
        }
        if(operator.dpadLeftWasPressed()){
            turretOff = !turretOff;
        }




        if (operator.right_trigger>0.5&&!shooterStuff.isScheduled()) {
            shooterStuff.schedule();
        }
    }
    public Command autonShootSequence(boolean stopFlywheelAfter) {
        Command prep = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, -0.5);
                    shooter.spinToRpm(3200);
                })
//                .setPriority(50)
                .requiring(intake, shooter);
        Command shoot = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, 1.0);
                    shooter.spinToRpm(3200);
                })
                .setEnd(endCondition -> {
                    intake.setPowers(0, 0);
                    if (stopFlywheelAfter) shooter.stop();
                })
//                .setPriority(51) lwk don't need the priorities cus they're inside sequrential
                .requiring(intake, shooter);

        return sequential(
                race(
                        prep.until(() -> shooter.isRPMGood(3100,3300)),
                        waitMs(1500)
                ),
                race(
                        shoot,
                        waitMs(1200)
                )
        );
    }

    public Command autoIntakeCommand(double wait) {
        return deadline(
                waitMs(wait),
                intake.intake()
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