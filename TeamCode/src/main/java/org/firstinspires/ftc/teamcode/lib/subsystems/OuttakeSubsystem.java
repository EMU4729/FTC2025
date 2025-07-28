package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OuttakeSubsystem extends SubsystemBase {
    /**
     * Preset servo values to use.
     */
    public static class Stops {
        // Pitch Servo Stops
        public static final double ROLL_SERVO_FORWARD = 0;
        public static final double ROLL_SERVO_DOWN = 0;

        // TODO: Add more
    }
    private final static double grabOpen = 0.82;
    private final static double grabClose = 0.65;
    private final static double pivotGrab = 0.1;
    private final static double pivotScore = 0.8;
    public static double outtakeRotateLeftSpecimenScore = 0.24; //  diff. 0.88 should be closer to 0.9 in my opinion this is full rotate
    public static double outtakeRotateRightSpecimenScore = 0.88; //^ diff
    public static double outtakeRotateSpecimenGrab = 0.9;
    public static double outtakePivotSpecimenGrab = 0.1; // grab position
    public static double outtakePivotSpecimenScore = 0.8;


    private final Servo grab, leftPivot, rightPivot, leftRotateServo, rightRotateServo;
    private final Telemetry telemetry;

    public OuttakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        grab = hardwareMap.get(Servo.class, "ograb");
        leftRotateServo = hardwareMap.get(Servo.class, "olr");
        rightRotateServo = hardwareMap.get(Servo.class, "orr");

        leftPivot = hardwareMap.get(Servo.class, "olp");
        rightPivot = hardwareMap.get(Servo.class, "orp");


        // = hardwareMap.get(Servo.class, "iG");
        leftPivot.setDirection(Servo.Direction.REVERSE);
        grab.setDirection(Servo.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    public void init() {
        //setPivotState(PivotState.SPECIMENGRAB180);
        leftPivot.setPosition(pivotGrab-0.085);
        rightPivot.setPosition(pivotGrab-0.085);
        //setRotateState(RotateState.SPECIMENSCORE180);
        leftRotateServo.setPosition(0.74+0.21);
        rightRotateServo.setPosition(0.32+0.21);
        //setGrabState(GrabState.CLOSED);
    }

    public void score(){

        //setRotateState()
        leftRotateServo.setPosition(outtakeRotateLeftSpecimenScore);
        rightRotateServo.setPosition(outtakeRotateRightSpecimenScore);
        setPivotPosition(outtakePivotSpecimenScore);
        setPivotPosition(pivotScore);

    }
    public void grab(){

        //setRotateState()

        leftPivot.setPosition(outtakePivotSpecimenGrab);
        rightPivot.setPosition(outtakePivotSpecimenGrab);

        setPivotPosition(outtakeRotateSpecimenGrab);

    }



    public void setRotatePosition(double position){
        leftRotateServo.setPosition(position);
        rightRotateServo.setPosition(position);
    }

    public void setPivotPosition(double position){
        leftPivot.setPosition(position);
        rightPivot.setPosition(position);
    }

    public void setGrabOpen(){
        grab.setPosition(grabOpen);
    }
    public void setGrabClose(){
        grab.setPosition(grabClose);
    }








    // TODO: Add methods to actually do stuff! (And use them inside the Robot class!)
}
