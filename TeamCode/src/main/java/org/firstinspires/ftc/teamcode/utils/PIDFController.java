package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController {
    private static final double MAX_OUTPUT = 1.0;

    // Tuning coefficients (Added kF)
    private double kP, kI, kD, kF;

    // State variables
    private double errorSum = 0;
    private double lastError = 0;
    private boolean hasCalculated = false;

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
        // The first call has no previous loop to measure from.
        double dt = hasCalculated ? timer.seconds() : 0;
        hasCalculated = true;
        timer.reset();

        double error = target - current;

        double pTerm = kP * error;

        double dTerm = 0;
        if (dt > 0) {
            dTerm = kD * ((error - lastError) / dt);
        }

        double fTerm = kF * target;

        lastError = error;

        // Anti-windup: stop integrating while saturated, unless the error is unwinding the output.
        double candidateSum = errorSum + error * dt;
        double candidateOutput = pTerm + kI * candidateSum + dTerm + fTerm;
        if (Math.abs(candidateOutput) < MAX_OUTPUT || candidateOutput * error <= 0) {
            errorSum = candidateSum;
        }

        double output = pTerm + kI * errorSum + dTerm + fTerm;
        return Math.max(-MAX_OUTPUT, Math.min(MAX_OUTPUT, output));
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
        hasCalculated = false;
        timer.reset();
    }
}
