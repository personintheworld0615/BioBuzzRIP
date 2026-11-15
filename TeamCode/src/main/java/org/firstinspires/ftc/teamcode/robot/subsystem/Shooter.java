package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.MyTelem;
import org.firstinspires.ftc.teamcode.Util.PIDFController;
import org.firstinspires.ftc.teamcode.Util.Constants;

import java.util.function.DoubleSupplier;

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
    private PIDFController pidfController2;
    private final ElapsedTime timer = new ElapsedTime();
    private double targetRPM = 0;
    //40000 rpm goal

    //0




    public Shooter(HardwareMap hm) {
        pidfController = new PIDFController(Constants.kP,Constants.kI,Constants.kD,Constants.kF);
        pidfController2 = new PIDFController(Constants.kP,Constants.kI,Constants.kD,Constants.kF);

        Shooter = hm.get(DcMotorEx.class, "ShooterMotor1");
        Shooter2 = hm.get(DcMotorEx.class, "ShooterMotor2");




        Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        Shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Linkage servo
        this.linkage = linkage;


        // we cud lie telem using specific like shooter telem methods, we should discuss
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

    public void spinToRpm(double targetRPM) {
        this.targetRPM = targetRPM;
        double currRPM = getRPM();

        // PID feedback
        double pidPower = pidfController.calculate(currRPM, targetRPM);

        double power = Range.clip(pidPower , 0, 1);

        double currRPM2 = getRPM2();

        double pidPower2 = pidfController2.calculate(currRPM2, targetRPM);

        double power2 = Range.clip(pidPower2, 0, 1);
        Shooter.setPower(power);

        Shooter2.setPower(power2);


    }
    public Command spinToRpmCommand(double targetRPM) {
        return spinToRpmCommand(() -> targetRPM);
    }

    // Reads the target every loop, so it follows a Dashboard value or a changing distance.
    public Command spinToRpmCommand(DoubleSupplier targetRPM) {
        return Command.build()
                .setStart(() -> spinToRpm(targetRPM.getAsDouble()))
                .setExecute(() -> spinToRpm(targetRPM.getAsDouble()))
                .setEnd(endCondition -> stop())
                .requiring(this);
    }

    public Command lookupCommand(DoubleSupplier distance) {
        return spinToRpmCommand(() -> distanceToRPM(distance.getAsDouble()));
    }

    public Command stopCommand() {
        return Command.build()
                .setStart(this::stop)
                .setDone(() -> true)
                .requiring(this);
    }

    //    public void setState(String state){
//        switch (state) {
//            case ACTIVE
//
//
//        }
//
//    }
    public enum ShooterStates {
        ACTIVE, STOP, PASSIVE
    }




    public void setPIDFConstants(double kp, double ki, double kd, double kf){
        pidfController.setCoefficients(kp,ki,kd,kf);
        pidfController2.setCoefficients(kp,ki,kd,kf);
    }

    public boolean atSpeed(double low, double high) {
        double rpm = getRPM();
        return rpm >= low && rpm <= high;
    }

    public void stop() {
        targetRPM = 0;
        Shooter.setPower(0);
        Shooter2.setPower(0);
    }

    public double getTargetRpm() {
        return targetRPM;
    }


    public void setLinkage(double pos) {
        linkage.setPosition(pos);
    }


    public void setPower(double p) {
        Shooter.setPower(p);
        Shooter2.setPower(p);
    }

    /**
     * Linearly interpolates the lookup table in Constants, holding the end values outside it.
     * Empty slots are skipped and points may be in any order. Returns 0 when no points are
     * recorded, so the shooter stays off.
     */
    public double distanceToRPM(double distance) {
        double[] distances = Constants.SHOOTER_DISTANCES;
        double[] rpms = Constants.SHOOTER_RPMS;
        int below = -1;
        int above = -1;
        for (int i = 0; i < Math.min(distances.length, rpms.length); i++) {
            if (rpms[i] <= 0) {
                continue;
            }
            if (distances[i] <= distance && (below < 0 || distances[i] > distances[below])) {
                below = i;
            }
            if (distances[i] >= distance && (above < 0 || distances[i] < distances[above])) {
                above = i;
            }
        }
        if (below < 0 && above < 0) {
            return 0;
        }
        if (below < 0) {
            return rpms[above];
        }
        if (above < 0 || distances[above] == distances[below]) {
            return rpms[below];
        }
        double fraction = (distance - distances[below]) / (distances[above] - distances[below]);
        return rpms[below] + fraction * (rpms[above] - rpms[below]);
    }

    /**
     * Saves a lookup table point. Re-recording within SAME_POINT_TOLERANCE_IN of an existing
     * point overwrites it, otherwise the first empty slot is used. Returns the slot written,
     * or -1 when the table is full.
     */
    public int recordPoint(double distance, double rpm) {
        int slot = slotFor(distance);
        if (slot >= 0) {
            Constants.SHOOTER_DISTANCES[slot] = distance;
            Constants.SHOOTER_RPMS[slot] = rpm;
        }
        return slot;
    }

    private int slotFor(double distance) {
        double[] distances = Constants.SHOOTER_DISTANCES;
        double[] rpms = Constants.SHOOTER_RPMS;
        int length = Math.min(distances.length, rpms.length);
        for (int i = 0; i < length; i++) {
            if (rpms[i] > 0 && Math.abs(distances[i] - distance) <= Constants.SAME_POINT_TOLERANCE_IN) {
                return i;
            }
        }
        for (int i = 0; i < length; i++) {
            if (rpms[i] <= 0) {
                return i;
            }
        }
        return -1;
    }



}