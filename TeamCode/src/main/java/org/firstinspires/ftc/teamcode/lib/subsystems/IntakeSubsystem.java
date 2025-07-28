package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * Instances of this class represent preset positions of the gripper, pivot and rotation servos.
     * <p>
     * Some preset states are also included in this class as static members. Ask Leon for the meanings of the cloud, ground and hover states.
     */
    public static class State {
        public static final State CLOUD = new State(0.5, 0.62, -0.13);
        public static final State GROUND = new State(0.05, 0.82, 0.18);
        public static final State HOVER = new State(0.05, 0.43, 0.2);

        public double grip;
        public double pivot;
        public double rotate;

        public State(double grip, double pivot, double rotate) {
            this.grip = grip;
            this.pivot = pivot;
            this.rotate = rotate;
        }
    }

    private final Servo pivot;
    private final Servo leftRotateServo;
    private final Servo rightRotateServo;
    private final Servo gripServo;
    private final Telemetry telemetry;
    private State currentState;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        pivot = hardwareMap.get(Servo.class, "iP");
        leftRotateServo = hardwareMap.get(Servo.class, "iLR");
        rightRotateServo = hardwareMap.get(Servo.class, "iRR");
        gripServo = hardwareMap.get(Servo.class, "iG");

        pivot.setDirection(Servo.Direction.REVERSE);
        rightRotateServo.setDirection(Servo.Direction.REVERSE);

        applyState(State.HOVER);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Applies the given state to the pivot, grip and rotation servos, setting their positions.
     *
     * @param state The state to apply.
     */
    public void applyState(State state) {
        currentState = state;
        pivot.setPosition(state.pivot);
        gripServo.setPosition(state.grip);
        leftRotateServo.setPosition(state.rotate);
        rightRotateServo.setPosition(state.rotate);
    }

    private void updateTelemetry() {
        telemetry.addData("Intake Grip Position", currentState.grip);
        telemetry.addData("Intake Pivot Position", currentState.pivot);
        telemetry.addData("Intake Rotate Position", currentState.rotate);
    }
}
