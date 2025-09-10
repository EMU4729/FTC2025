package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase {
    private enum State {
        LIFT,
        SHOOTER
    }

    private static final double LIFT_DISTANCE_PER_TICK = 1;
    private static final double TICKS_PER_SHOOTER_ROTATION = 1;
    private static final double LIFT_PTO_POS = 1;// TODO: change values
    private static final double SHOOTER_PTO_POS = 0;// TODO: change values
    private static final double SHOOTER_PID_P = 0;
    private static final double SHOOTER_PID_I = 0;
    private static final double SHOOTER_PID_D = 0;
    private static final double LIFT_PID_P = 0;

    private State state = State.SHOOTER;
    private int liftPosition = 0;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private final Servo ptoServo;
    private final Servo arcServo;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "shooterL");
        rightMotor = hardwareMap.get(DcMotorEx.class, "shooterR");
        ptoServo = hardwareMap.get(Servo.class, "shooterPTO");
        arcServo = hardwareMap.get(Servo.class, "shooterArc");
    }

    /**
     * sets the PTO state to a given state to switch between shooter and lift
     *
     * @param newState State.LIFT or State.SHOOTER
     */
    private void setPTOState(State newState) {
        if (newState == state) return;

        switch (newState) {
            case LIFT:
                ptoServo.setPosition(LIFT_PTO_POS);
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftMotor.setPositionPIDFCoefficients(LIFT_PID_P);
                rightMotor.setPositionPIDFCoefficients(LIFT_PID_P);
                break;

            case SHOOTER:
                liftPosition += leftMotor.getCurrentPosition();

                ptoServo.setPosition(SHOOTER_PTO_POS);
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setVelocityPIDFCoefficients(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D, 0);
                rightMotor.setVelocityPIDFCoefficients(SHOOTER_PID_P, SHOOTER_PID_I, SHOOTER_PID_D, 0);
                break;
        }

        state = newState;
    }

    /**
     * sets left right motor target position to given distance
     *
     * @param pos the distance in m
     */
    private void liftGoTo(double pos) {
        setPTOState(State.LIFT);
        int _pos = (int) (pos / LIFT_DISTANCE_PER_TICK);
        leftMotor.setTargetPosition(_pos - liftPosition);
        rightMotor.setTargetPosition(_pos - liftPosition);
    }

    /**
     * sets left & right motors to a given velocity
     *
     * @param vel the target velocity of the shooter in ticks/second
     * @param arc The position of the arc servo, in the range [0, 1].
     */
    public void shoot(double vel, double arc) {
        setPTOState(State.SHOOTER);
        leftMotor.setVelocity(vel * TICKS_PER_SHOOTER_ROTATION);
        rightMotor.setVelocity(vel * TICKS_PER_SHOOTER_ROTATION);
        arcServo.setPosition(arc);
    }
}
