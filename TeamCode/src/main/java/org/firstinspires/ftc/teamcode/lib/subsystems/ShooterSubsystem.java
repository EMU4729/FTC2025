package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
    private static final double TICKS_PER_SHOOTER_ROTATION = 1;
    private static final double SHOOTER_PID_P = 0;
    private static final double SHOOTER_PID_I = 0;
    private static final double SHOOTER_PID_D = 0;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Servo arcServo;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "shooterL");
        rightMotor = hardwareMap.get(DcMotorEx.class, "shooterR");
        arcServo = hardwareMap.get(Servo.class, "shooterArc");
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setVelocityPIDFCoefficients(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D, 0);
        rightMotor.setVelocityPIDFCoefficients(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D, 0);
    }

    /**
     * sets left & right motors to a given velocity
     *
     * @param vel the target velocity of the shooter in ticks/second
     * @param arc The position of the arc servo, in the range [0, 1].
     */
    public void shoot(double vel, double arc) {
        leftMotor.setVelocity(vel * TICKS_PER_SHOOTER_ROTATION);
        rightMotor.setVelocity(vel * TICKS_PER_SHOOTER_ROTATION);
        arcServo.setPosition(arc);
    }
}
