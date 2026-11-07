package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    // Tuning coefficients (Added kF)
    private double kP, kI, kD, kF;

    // State variables
    private double errorSum = 0;
    private double lastError = 0;

    // Timer to track loop speed precisely
    private ElapsedTime timer = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF; // Initialize Feedforward
    }

    /**
     * Calculates the motor power. Call this once per loop.
     * @param current The current value (encoder position or velocity)
     * @return Calculated power output bounded between -1.0 and 1.0
     */
    public double calculate(double current,double target) {
        double dt = timer.seconds();
        timer.reset();

        double error = target - current;

        double pTerm = kP * error;

        errorSum += error * dt;
        double iTerm = kI * errorSum;

        double dTerm = 0;
        if (dt > 0) {
            dTerm = kD * ((error - lastError) / dt);
        }

        double fTerm = kF * target;

        lastError = error;

        double output = pTerm + iTerm + dTerm + fTerm;
        return Math.max(-1.0, Math.min(1.0, output));
    }


    public void setCoefficients(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

    public void reset() {
        errorSum = 0;
        lastError = 0;
        timer.reset();
    }
}
