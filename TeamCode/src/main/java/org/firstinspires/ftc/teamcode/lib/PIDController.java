package org.firstinspires.ftc.teamcode.lib;

import java.util.OptionalDouble;

public class PIDController {
    private final double kp;
    private final double ki;
    private final double kd;
    private final double f;

    private double prevError = 0;
    private double integral = 0;
    private OptionalDouble izone = OptionalDouble.empty();

    public PIDController(double p, double i, double d, double f) {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        this.f = f;
    }

    public PIDController withIZone(OptionalDouble izone) {
        this.izone = izone;
        return this;
    }

    public void resetIntegral() {
        integral = 0;
    }

    public double calculate(double error) {
        if (!izone.isPresent() || Math.abs(error) < izone.getAsDouble()) {
            integral += error;
        }
        double p = kp * error;
        double i = kp * integral;
        double d = kd * (error - prevError);
        double result = p + i + d + f;
        prevError = error;
        return result;
    }
}
