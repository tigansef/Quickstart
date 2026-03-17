package org.firstinspires.ftc.teamcode.writtenCode.controllers;

import com.qualcomm.robotcore.util.Range;

/**
 * Simple PIDF controller where calculate() takes ERROR (target - measurement).
 * Output is typically motor power [-1..1].
 *
 * kF is treated as a constant feedforward term (you can set it each loop).
 */
public class PIDFController {

    private double kP, kI, kD, kF;

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0;
    private boolean first = true;

    // Optional anti-windup clamp
    private double integralClamp = 1.0; // in "error-seconds" units (tune if needed)

    public PIDFController(double p, double i, double d, double f) {
        setPIDF(p, i, d, f);
        reset();
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

    public void setIntegralClamp(double clamp) {
        this.integralClamp = Math.max(0.0, clamp);
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        lastTimeNs = 0;
        first = true;
    }

    /** @param error (target - measurement) */
    public double calculate(double error) {
        long now = System.nanoTime();

        double dt;
        if (first) {
            dt = 0.0;
            first = false;
        } else {
            dt = (now - lastTimeNs) / 1e9;
            if (dt <= 0) dt = 1e-3;
        }
        lastTimeNs = now;

        // I
        integral += error * dt;
        integral = Range.clip(integral, -integralClamp, integralClamp);

        // D
        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
        lastError = error;

        return (kP * error) + (kI * integral) + (kD * derivative) + kF;
    }

    public double getP() { return kP; }
    public double getI() { return kI; }
    public double getD() { return kD; }
    public double getF() { return kF; }
}