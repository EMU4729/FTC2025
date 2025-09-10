package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShooterSubsystem extends SubsystemBase{

    private static enum State{
        LIFT,
        SHOOTER
    }

    private State state = State.SHOOTER;
    private int liftPosition = 0;


    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private final Servo ptoServo;
    private final Servo arcServo;

    private final static double DISTANCE_PER_ROTATION = 1;
    private final static double LIFT_PTO_POS = 1;// TODO: change values
    private final static double SHOOTER_PTO_POS = 0;// TODO: change values

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "shooterL");
        rightMotor = hardwareMap.get(DcMotorEx.class, "shooterR");
        ptoServo = hardwareMap.get(Servo.class, "shooterPTO");
        arcServo = hardwareMap.get(Servo.class, "shooterArc");
    }

    private void liftGoTo(double pos){


        int _pos = (int) (pos/DISTANCE_PER_ROTATION);
        leftMotor.setTargetPosition(_pos-liftPosition);
        rightMotor.setTargetPosition(_pos-liftPosition);
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
                leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                break;
            case SHOOTER:
                liftPosition += leftMotor.getCurrentPosition();
                leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                ptoServo.setPosition(SHOOTER_PTO_POS);

                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setVelocityPIDFCoefficients(0, 0, 0, 0);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setVelocityPIDFCoefficients(0, 0, 0, 0);
                break;
        }

        state = newState;
    }




    public void liftUp() {
        setPTOState(State.LIFT);

        // ...
    }
    public void liftDown() {
        setPTOState(State.LIFT);

        // ...
    }
    public void shoot() {
        setPTOState(State.SHOOTER);


        // ...
    }
}
