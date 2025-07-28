package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExtensionSubsystem extends SubsystemBase {
    private static final double EXTENDED_DISTANCE = 0.36;
    private static final double RETRACTED_DISTANCE = 0.54;

    private final Servo leftServo;
    private final Servo rightServo;
    private final Telemetry telemetry;

    public ExtensionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftServo = hardwareMap.get(Servo.class, "eL");
        rightServo = hardwareMap.get(Servo.class, "eR");
        rightServo.setDirection(Servo.Direction.REVERSE);
        this.telemetry = telemetry;
        set(0);
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Sets the distance of the extension.
     *
     * @param extension The desired extension percentage, in the range [0, 1].
     */
    public void set(double extension) {
        double position = extension * (EXTENDED_DISTANCE) + (1 - extension) * RETRACTED_DISTANCE;
        leftServo.setPosition(position);
        rightServo.setPosition(position);
    }

    private void updateTelemetry() {
        // The left and right servos are guaranteed to have the same commanded position, so it
        // doesn't matter which one we use here
        telemetry.addData("Extension Distance", leftServo.getPosition());
    }
}
