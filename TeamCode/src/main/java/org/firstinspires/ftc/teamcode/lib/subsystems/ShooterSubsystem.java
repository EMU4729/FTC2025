package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
    private static final double TICKS_PER_SHOOTER_ROTATION = 28;
    private static final double SHOOTER_PID_P = 1000;
    private static final double SHOOTER_PID_I = 0;
    private static final double SHOOTER_PID_D = 0;

    private final DcMotorEx motor;
    private final Servo tiltServo;
    private final Servo popServo;
    private final Telemetry telemetry;
    private double vel = 0;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        tiltServo = hardwareMap.get(Servo.class, "shooterTilt");
        popServo = hardwareMap.get(Servo.class, "shooterPop");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D, 0);
        this.telemetry = telemetry;
    }

    /**
     * sets shooting motor to a given velocity
     *
     * @param vel the target velocity of the shooter in ticks/second
     */
    public void setSpeed(double vel) {
//         motor.setPower(vel);
        motor.setVelocity(vel * TICKS_PER_SHOOTER_ROTATION);
        this.vel = vel;
    }

    public double getMotorSpeed() {
        return motor.getVelocity() / TICKS_PER_SHOOTER_ROTATION;
    }

    public boolean atDesiredSpeed() {
        return Math.abs(vel - getMotorSpeed()) < 0.1;
    }

    /**
     * Sets the position of the tilt servo
     *
     * @param tilt The tilt servo's position, in the range [0, 1]
     */
    public void setTilt(double tilt) {
        tiltServo.setPosition(tilt);
    }

    /**
     * Pops the ball into the shooter
     */
    public void pop() {
        popServo.setPosition(1);
    }

    /**
     * Unpops the ball
     */
    public void unpop() {
        popServo.setPosition(0);
    }

    @Override
    public void periodic() {
        telemetry.addData("Shooter Speed (rps)", getMotorSpeed());
        telemetry.addData("Shooter Desired Speed (rps)", vel);
        telemetry.addData("Shooter At Target Speed", atDesiredSpeed());
    }
}
