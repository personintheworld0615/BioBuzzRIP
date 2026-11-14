package org.firstinspires.ftc.teamcode.robot.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Intake {

    private DcMotor intake;
    private DcMotor transfer;
    private Gamepad gamepad1;
    private Gamepad gamepad2;


    public Intake(Gamepad gamepad1, Gamepad gamepad2) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        // Motor configuration
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Command intake(){
        Command intakeballs = Command.build()
                .setStart(()->{
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .setExecute(()->{
                    intake.setPower(1);
                    transfer.setPower(1);
                })
                .setDone(()-> gamepad2.left_trigger<0.2)
                .setEnd(endCondition -> {
                    if(endCondition == EndCondition.NATURALLY){
                        stop();
                    }
                })
                .requiring(this);

        return intakeballs;
    }
    public Command reverse(){
        Command reverse = Command.build()
                .setStart(()->{
                    intake.setPower(-1);
                    transfer.setPower(-1);
                })
                .setExecute(()->{
                    this.intake.setPower(-1);
                    transfer.setPower(-1);
                })
                .setEnd(endCondition -> stop())
                .setPriority(30)
                .requiring(this);

        return reverse;
    }
    public void stop(){
        intake.setPower(0);
        transfer.setPower(0);
    }
    public void setPowers(double intake, double transfer){
        this.intake.setPower(intake);
        this.transfer.setPower(transfer);
    }
}