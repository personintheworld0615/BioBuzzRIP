package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.utils.PIDFController;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.calculations.RpmLookupTable;
import org.firstinspires.ftc.teamcode.utils.constants.ShooterConstants;

public class Shooter {

    public enum ShooterState {
        OFF, MANUAL, LOOKUP
    }
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public Servo linkage;

    private final PIDFController pidfController1;
    private final PIDFController pidfController2;

    private ShooterState state = ShooterState.OFF;
    private double targetRpm = 0;

    public Shooter(DcMotorEx Shooter1, DcMotorEx Shooter2, Servo linkage) {
        pidfController1 = new PIDFController(ShooterConstants.KP, ShooterConstants.KI,
                ShooterConstants.KD, ShooterConstants.KF);
        pidfController2 = new PIDFController(ShooterConstants.KP, ShooterConstants.KI,
                ShooterConstants.KD, ShooterConstants.KF);

        shooter1 = Shooter1;
        shooter2 = Shooter2;

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        this.linkage = linkage;
    }



    public void setState(ShooterState state) {
        this.state = state;
        switch (state) {
            case OFF:
                targetRpm = 0;
                resetControllers();
                stop();
                break;
            case MANUAL:
                targetRpm = ShooterConstants.TARGET_RPM;
                spinToRpm(targetRpm);
                break;
            case LOOKUP:
                targetRpm = RpmLookupTable.interpolate(ShooterConstants.DISTANCES, ShooterConstants.RPMS, Robot.getDistanceToGoal());
                spinToRpm(targetRpm);
                break;
        }
    }

    public void periodic() {
        setState(state);
    }

    public ShooterState getState() {
        return state;
    }

    public double getTargetRpm() {
        return targetRpm;
    }


    public double getRPM() {
        return Math.abs(shooter1.getVelocity() * 2.14);
    }

    public double getRPM2() {
        return Math.abs(shooter2.getVelocity()*2.14);
    }

    public void spinToRpm(double targetRPM) {
        double power = Range.clip(pidfController1.calculate(getRPM(), targetRPM), 0, 1);
        double power2 = Range.clip(pidfController2.calculate(getRPM2(), targetRPM), 0, 1);
        shooter1.setPower(power);
        shooter2.setPower(power2);
    }

    public void setPIDFConstants(double kp, double ki, double kd, double kf){
        pidfController1.setCoefficients(kp,ki,kd,kf);
        pidfController2.setCoefficients(kp,ki,kd,kf);
    }

    private void resetControllers() {
        pidfController1.reset();
        pidfController2.reset();
    }

    public boolean atSpeed(double low, double high) {
        double rpm = getRPM();
        return rpm >= low && rpm <= high;
    }

    public void stop() {
        shooter1.setPower(0);
        shooter2.setPower(0);
    }


    public void setLinkage(double pos) {
        linkage.setPosition(pos);
    }


    public void setPower(double p) {
        shooter1.setPower(p);
        shooter2.setPower(p);
    }
}
