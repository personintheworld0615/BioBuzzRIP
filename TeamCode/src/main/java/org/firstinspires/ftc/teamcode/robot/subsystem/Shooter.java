package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.Util.PIDFController;
import org.firstinspires.ftc.teamcode.Util.Constants;

public class Shooter {

    // =======================
    //  Hardware
    // =======================

    public DcMotorEx Shooter;
    public DcMotorEx Shooter2;
    public Servo linkage;
    public Follower follower;



    // One controller per motor: each keeps its own integral and last error.
    private PIDFController pidfController;
    private final ElapsedTime timer = new ElapsedTime();

    double targetRPM = 0;

    InterpLUT lut;
    Pose topGoal;
    Pose bottomGoal;



    public Shooter(HardwareMap hm, Follower follower, boolean red) {
        this.follower = follower;

        if (red) {
            topGoal = new Pose(58, 86);
            bottomGoal = new Pose(58, 55);
        } else {
            topGoal = new Pose(82, 86);
            bottomGoal = new Pose(82, 55);
        }

        lut = new InterpLUT();

        lut.add(0, 3200);
        lut.add(10, 3200);
        lut.add(1000000000, 3200);
        //yo where tf did you pull these vlaues from????

        lut.createLUT();

        pidfController = new PIDFController(
                Constants.kP, Constants.kI, Constants.kD, Constants.kF
        );


        Shooter = hm.get(DcMotorEx.class, "Shooter1");
        Shooter2 = hm.get(DcMotorEx.class, "Shooter2");

        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
//        linkage = hm.get(Servo.class, "Linkage");
//        linkage.setPosition(0.5);
    }

    // =======================
    //  Shooter RPM Functions
    // =======================

    public double getRPM() {
        return Math.abs(Shooter.getVelocity() * 2.14);
    }

    public double getRPM2() {
        return Math.abs(Shooter2.getVelocity()*2.14);
    }

    public void spinToRpm() {
        double distance = getDitance();
        this.targetRPM = lut.get(distance);
        double currRPM = getRPM();


        // PID feedback
        double pidPower = pidfController.calculate(currRPM, targetRPM);

        double power = Range.clip(pidPower , 0, 1);

        double currRPM2 = getRPM2();

        double pidPower2 = pidfController.calculate(currRPM2, targetRPM);

        double power2 = Range.clip(pidPower2, 0, 1);
        Shooter.setPower(power);
        Shooter2.setPower(power2);


    }

    public boolean isRPMGood(){
        return (getRPM()>= targetRPM-200) && (getRPM()<= targetRPM+200) && (getRPM2()>= targetRPM-200) && (getRPM2()<= targetRPM+200);
    }
    public boolean isRPMGood(double low, double high){
        return (getRPM()>=low&&getRPM()<=high);
    }
    public void spinToRpm(double targetRPM) {
        double currRPM = getRPM();
        this.targetRPM = targetRPM;
        // PID feedback
        double pidPower = pidfController.calculate(currRPM, targetRPM);

        double power = Range.clip(pidPower , 0, 1);

        double currRPM2 = getRPM2();

        double pidPower2 = pidfController.calculate(currRPM2, targetRPM);

        double power2 = Range.clip(pidPower2, 0, 1);
        Shooter.setPower(power);

        Shooter2.setPower(power2);


    }

    public double getTargetRpm(){
        return targetRPM;
    }
    //OH MAN i sure do love getting my "    DITANCE"
    public double getDitance(){
        Pose pose = follower.pose();
        Pose goal = null;
        if (pose.y()>70){
            goal = topGoal;
        }
        else{
            goal = bottomGoal;
        }
        return  pose.distance(goal);

    }


    public void setPIDFConstants(double kp, double ki, double kd, double kf){
        pidfController.setCoefficients(kp,ki,kd,kf);
    }

    public boolean atSpeed(double low, double high) {
        double rpm = getRPM();
        return rpm >= low && rpm <= high;
    }

    public void stop() {
        Shooter.setPower(0);
        Shooter2.setPower(0);
    }



    public void setLinkage(double pos) {
        linkage.setPosition(pos);
    }


    public void setPower(double p) {
        Shooter.setPower(p);
        Shooter2.setPower(p);
    }

    public String getTelemetry() {
        double distance = getDitance();
        return "Shooter1 power: " + Shooter.getPower() + " rpm: " + getRPM() + "\n"
                + "Shooter2 power: " + Shooter2.getPower() + " rpm: " + getRPM2() + "\n"
                + "Distance: " + distance + " LUT: " + lut.get(distance) + "\n"
                + "Linkage pos: " + linkage.getPosition();
    }




}