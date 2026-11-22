
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

    private Command turretConstantCommand;
//    private Command turretFinal;
    private Command drive;

    private Command preshoot;
    private Command shoot;
    private double target_rpm = 0;
    private Command shooterstuff;



    // Constructor
    public Robot(HardwareMap hardwareMap, Follower follower, Gamepad gamepad1, Gamepad gamepad2) {

        driver = gamepad1;
        operator = gamepad2;
        this.follower = follower;

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap, driver, operator);
        turret = new Turret(follower, hardwareMap);
        drivetrain = new Drivetrain(hardwareMap, this.follower);

        initializeCommands();
    }

    private void initializeCommands() {

        intakeCommand = intake.intake();
        reverseCommand = intake.reverse()
                .until(() -> operator.leftBumperWasReleased());

        turretConstantCommand = loop(
                sequential(
                    turret.defaultCommand().until(() -> driver.rightBumperWasPressed()),
                    turret.aimCommand().until(() -> driver.rightBumperWasPressed())
                )
            );
//        turretFinal = loop(turretConstantCommand);


        drive = loop(infinite(
                () -> drivetrain.manualDrive(driver)
        ));

        preshoot = Command.build()
                .setExecute(() -> {
                    intake.setPowers(1.0, -0.3);
                    shooter.spinToRpm(1000);
                    //reduced to an arbitrary val for now, TODO: change 1000 to the minimum speed in the lookup table
                })
                .setPriority(50)
                .requiring(intake, shooter);

        shoot = Command.build()
                .setExecute(() -> {
                    //TODO: ensure that there is a conditional such that the intake & transfer spin up is dependent on the shooter being relatively close to RPM.
                    intake.setPowers(1.0, 1.0);
                    shooter.spinToRpm(target_rpm);
                })
                .setEnd(endCondition -> {
                    intake.setPowers(0, 0);
                    shooter.stop();
                })
                .setPriority(50)
                .requiring(intake, shooter);


        shooterstuff = sequential(

                preshoot.until(() ->
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

        if (!Scheduler.isScheduled(turretConstantCommand)) {
            Scheduler.schedule(turretConstantCommand);
        }
        if (!Scheduler.isScheduled(drive)) {
            Scheduler.schedule(drive);
        }

        if (operator.left_trigger > 0.2 &&
                !Scheduler.isScheduled(intakeCommand)) {

            Scheduler.schedule(intakeCommand);
        }


        if (operator.leftBumperWasPressed()) {
            Scheduler.schedule(reverseCommand);
        }


        // Shooting
        if (operator.right_trigger>0.5&&!shooterstuff.isScheduled()) {
            shooterstuff.schedule();
        }
    }

    public Command getDriveCommand() {
        return drive;
    }

    // Im pretty sure this is how it should be, because it was set to 3200 arbitrarily before which obv doesn;t make sense.
    public Command shoot(double rpm){
        target_rpm = rpm;
        return shoot;
    }
    public void update() {
        follower.localizer.update();
        MainLoop();
        Scheduler.execute();
    }

}
