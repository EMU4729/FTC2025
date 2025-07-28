package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * Preset servo values to use.
     */
    public static class Stops {
        // Pitch Servo Stops
        public static final double ROLL_SERVO_FORWARD = 0;
        public static final double ROLL_SERVO_DOWN = 0;

        // TODO: Add more
    }

    private final Servo pivot;
    private final Servo leftRotateServo;
    private final Servo rightRotateServo;
    private final Servo gripServo;
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        pivot = hardwareMap.get(Servo.class, "iP");
        leftRotateServo = hardwareMap.get(Servo.class, "iLR");
        rightRotateServo = hardwareMap.get(Servo.class, "iRR");
        gripServo = hardwareMap.get(Servo.class, "iG");
        pivot.setDirection(Servo.Direction.REVERSE);
        pivot.setPosition(0);

        this.telemetry = telemetry;
    }






    // TODO: Add methods to actually do stuff! (And use them inside the Robot class!)
}
