package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.PIDFController;
import org.firstinspires.ftc.teamcode.Util.PIDFController;
import org.firstinspires.ftc.teamcode.Util.Constants;

public class Outtake{

    // =======================
    //  Hardware
    // =======================
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public DcMotorEx outtake;
    public DcMotorEx outtake2;
    public Servo linkage;



    private PIDFController pidfController;
    private final ElapsedTime timer = new ElapsedTime();
    //40000 rpm goal

    //0




    public Outtake(HardwareMap hw, Telemetry t) {
        this.hardwareMap = hw;
        this.telemetry = t;
        pidfController = new PIDFController(Constants.kP,Constants.kI,Constants.kD,Constants.kF);
        // Map motors
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake2");
        outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake");

        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Linkage servo
        linkage = hardwareMap.get(Servo.class, "Linkage");

    }

    // =======================
    //  Shooter RPM Functions
    // =======================

    public double getRPM() {
        return Math.abs(outtake.getVelocity() * 2.14);
    }

    public double getRPM2() {
        return Math.abs(outtake2.getVelocity()*2.14);
    }

    public void spinToRpm(double targetRPM) {
        double currRPM = getRPM();

        // PID feedback
        double pidPower = pidfController.calculate(currRPM, targetRPM);


        double power = Range.clip(pidPower , 0, 1);

        double currRPM2 = getRPM2();

        double pidPower2 = pidfController.calculate(currRPM2, targetRPM);

        double power2 = Range.clip(pidPower2, 0, 1);
        outtake.setPower(power);

        outtake2.setPower(power2);


    }

//    public void setState(String state){
//        switch (state) {
//            case ACTIVE
//
//
//        }
//
//    }
    enum Shooter {
        ACTIVE, STOP, PASSIVE
    }





    public void setPIDFConstants(double kp, double ki, double kd, double kf){
        pidfController.setCoefficients(kp,ki,kd,kf);
    }

    public boolean atSpeed(double low, double high) {
        double rpm = getRPM();
        return rpm >= low && rpm <= high;
    }

    public void stop() {
        outtake.setPower(0);
        outtake2.setPower(0);
    }


    public void setLinkage(double pos) {
        linkage.setPosition(pos);
    }


    public void setPower(double p) {
        outtake.setPower(p);
        outtake2.setPower(p);
    }



}
