package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ExtensionSubsystem extends SubsystemBase {
    /**
     * The preset distances for the extension.
     */
    public static class Distances {
        public static final double EXTENDED = 1;
        public static final double RETRACTED = 0;
    }

    private final Servo leftServo;
    private final Servo rightServo;
    private final Telemetry telemetry;

    public ExtensionSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftServo = hardwareMap.get(Servo.class, "leftExtensionServo");
        rightServo = hardwareMap.get(Servo.class, "rightExtensionServo");
        this.telemetry = telemetry;
        set(Distances.RETRACTED);
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Sets the distance of the extension.
     *
     * @param distance The desired distance. It's recommended to use the distances presets in {@link Distances}.
     */
    public void set(double distance) {
        leftServo.setPosition(distance);
        rightServo.setPosition(distance);
    }

    private void updateTelemetry() {
        telemetry.addData("Extension Distance", leftServo.getPosition());
    }
}
