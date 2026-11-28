package org.firstinspires.ftc.teamcode.robot.subsystem;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Intake {

    private DcMotor intake;
    private DcMotor transfer;
    private Gamepad driver;
    private Gamepad operator;
    private Servo flowerServo;


    public Intake(HardwareMap hardwareMap , Gamepad gamepad1, Gamepad gamepad2) {
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        transfer = hardwareMap.get(DcMotor.class, "transferMotor");
        flowerServo = hardwareMap.get(Servo.class, "flowerServo");
        driver = gamepad1;
        operator  = gamepad2;
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
                    transfer.setPower(0.7);
                })
                .setExecute(()->{
                    intake.setPower(1);
                    transfer.setPower(0.7);
                })
                .setPriority(30)
                .setDone(()-> operator.right_trigger<=0.5)
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
                .setPriority(50)
                .requiring(this);

        return reverse;
    }
    public Command flowerIntakeDown(){
        return instant(()->flowerServo.setPosition(0.5));
    }
    public Command flowerIntakeUp(){
        return instant(()->flowerServo.setPosition(0.1));
    }

    public void stop(){
        intake.setPower(0);
        transfer.setPower(0);
    }
    public void setPowers(double intake, double transfer){
        this.intake.setPower(intake);
        this.transfer.setPower(transfer);
    }

    public void setIntakePower(double pwr){this.intake.setPower(pwr);}
    public void setTransferpower(double pwr){this.transfer.setPower(pwr);}

    public String getTelemetry() {
        return "Intake power: " + intake.getPower() + "\n"
                + "Transfer power: " + transfer.getPower();
    }
}