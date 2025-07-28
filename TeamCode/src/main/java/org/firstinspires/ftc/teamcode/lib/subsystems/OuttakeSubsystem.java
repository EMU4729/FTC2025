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










    // TODO: Add methods to actually do stuff! (And use them inside the Robot class!)
}
