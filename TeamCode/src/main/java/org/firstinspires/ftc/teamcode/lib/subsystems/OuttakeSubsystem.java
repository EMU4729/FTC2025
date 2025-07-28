package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeSubsystem extends SubsystemBase {
    // TODO: Modify these using values obtained from the Outtake Calibration OpMode.
    // NOTE: You will probably need to go into the `grab` and `score` methods below and modify code there, in addition to these constants.
    private final static double grabOpen = 0.82;
    private final static double grabClose = 0.65;
    private final static double pivotGrab = 0.1;
    private final static double pivotScore = 0.8;
    private final static double outtakeRotateLeftSpecimenScore = 0.24; //  diff. 0.88 should be closer to 0.9 in my opinion this is full rotate
    private final static double outtakeRotateRightSpecimenScore = 0.88; //^ diff
    private final static double outtakeRotateSpecimenGrab = 0.9;
    private final static double outtakePivotSpecimenGrab = 0.1; // grab position
    private final static double outtakePivotSpecimenScore = 0.8;

    private final Servo grab;
    private final Servo leftPivot;
    private final Servo rightPivot;
    private final Servo leftRotateServo;
    private final Servo rightRotateServo;
    private final Telemetry telemetry;

    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        grab = hardwareMap.get(Servo.class, "ograb");
        leftRotateServo = hardwareMap.get(Servo.class, "olr");
        rightRotateServo = hardwareMap.get(Servo.class, "orr");

        leftPivot = hardwareMap.get(Servo.class, "olp");
        rightPivot = hardwareMap.get(Servo.class, "orp");

        // TODO: You might want to verify that these are correct, and that there aren't more servos that should be reversed
        leftPivot.setDirection(Servo.Direction.REVERSE);
        grab.setDirection(Servo.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    public void init() {
        leftPivot.setPosition(pivotGrab - 0.085);
        rightPivot.setPosition(pivotGrab - 0.085);
        leftRotateServo.setPosition(0.74 + 0.21);
        rightRotateServo.setPosition(0.32 + 0.21);
    }

    public void score() {
        // WARN: This method does not set the grab servo distance, which seems problematic.
        // You might want to add a call to `setGrabOpen` or `setGrabClose` here.
        leftRotateServo.setPosition(outtakeRotateLeftSpecimenScore);
        rightRotateServo.setPosition(outtakeRotateRightSpecimenScore);
        leftPivot.setPosition(outtakePivotSpecimenScore);
        rightPivot.setPosition(outtakePivotSpecimenScore);
    }

    public void grab() {
        // WARN: This method is only setting the pivot servo distances, not the grab or rotate servos.
        // This seems problematic. I'd suggest adding calls to `left/rightRotateServo.setPosition` and
        // `setGrabOpen`/`setGrabClose` here.
        leftPivot.setPosition(outtakePivotSpecimenGrab);
        rightPivot.setPosition(outtakePivotSpecimenGrab);
    }

    public void setGrabOpen() {
        grab.setPosition(grabOpen);
    }

    public void setGrabClose() {
        grab.setPosition(grabClose);
    }

    /**
     * Sets the distances of all servos in the outtake subsystem at once.
     * <p>
     * <strong>Note: This is intended for use with the outtake calibration opmode only, which is also why it's prefixed with <code>unsafe_</code>. Don't use this in competition code!</strong>
     */
    public void unsafe_setAllPositions(double grab, double pivotLeft, double pivotRight, double rotateLeft, double rotateRight) {
        this.grab.setPosition(grab);
        this.leftPivot.setPosition(pivotLeft);
        this.rightPivot.setPosition(pivotRight);
        this.leftRotateServo.setPosition(rotateLeft);
        this.rightRotateServo.setPosition(rotateRight);
    }

    private void updateTelemetry() {
        telemetry.addData("Outtake Grab Position", grab.getPosition());
        telemetry.addData("Outtake Left Pivot Position", leftPivot.getPosition());
        telemetry.addData("Outtake Right Pivot Position", rightPivot.getPosition());
        telemetry.addData("Outtake Left Rotate Position", leftRotateServo.getPosition());
        telemetry.addData("Outtake Right Rotate Position", rightRotateServo.getPosition());
    }
}
