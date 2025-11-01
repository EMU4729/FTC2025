package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
    // TODO: tune these!
    private static final double TICKS_PER_SHOOTER_ROTATION = 1;
    private static final double SHOOTER_PID_P = 1;
    private static final double SHOOTER_PID_I = 0;
    private static final double SHOOTER_PID_D = 0;

    private final DcMotorEx motor;
    private final Servo yServo; // vertical servo
    private final Servo xServo; // horizontal servo
    private final Servo popServo;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        yServo = hardwareMap.get(Servo.class, "shooterYServo");
        xServo = hardwareMap.get(Servo.class, "shooterXServo");
        popServo = hardwareMap.get(Servo.class, "shooterPop");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setVelocityPIDFCoefficients(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D, 0);
    }

    /**
     * sets shooting motor to a given velocity
     *
     * @param vel the target velocity of the shooter in ticks/second
     */
    public void setSpeed(double vel) {
//         motor.setPower(vel);
        motor.setVelocity(vel * TICKS_PER_SHOOTER_ROTATION);
    }

    /**
     * Sets the vertical position of the shooter
     *
     * @param arc The vertical arc servo's position, in the range [0, 1]
     */
    public void setY(double arc) {
        yServo.setPosition(arc);
    }

    /**
     * Sets the horizontal position of the shooter
     *
     * @param arc The horizontal arc servo's position, in the range [0, 1]
     */
    public void setX(double arc) {
        xServo.setPosition(arc);
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
}
