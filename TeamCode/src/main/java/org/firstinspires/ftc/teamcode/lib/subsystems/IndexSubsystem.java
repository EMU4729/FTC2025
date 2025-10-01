package org.firstinspires.ftc.teamcode.lib.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IndexSubsystem extends SubsystemBase {
    /**
     * The different modes of operation of the indexer
     */
    public enum Mode {
        IDLE,
        INTAKE,
        SHOOT,
        CLOCKWISE,
        ANTICLOCKWISE,
    }

    private static final float COLOR_SENSOR_GAIN = 1;
    private static final float[] PURPLE_MIN_HSV = new float[]{0, 0, 0};
    private static final float[] PURPLE_MAX_HSV = new float[]{0, 0, 0};
    private static final float[] GREEN_MIN_HSV = new float[]{0, 0, 0};
    private static final float[] GREEN_MAX_HSV = new float[]{0, 0, 0};

    private enum Ball {
        GREEN,
        PURPLE,
        EMPTY
    }

    private final Servo servo;
    private final AnalogInput encoder;
    private final NormalizedColorSensor colorSensor;
    private final Ball[] storage = new Ball[]{Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};
    private Mode mode = Mode.IDLE;

    public IndexSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(Servo.class, "indexServo");
        encoder = hardwareMap.get(AnalogInput.class, "indexEncoder");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "indexColorSensor");

        // yes, this is officially how you switch on the led on a color sensor with a switchable led
        // yes, i hate it too
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) (colorSensor)).enableLight(true);
        }
        setGain(COLOR_SENSOR_GAIN);
    }

    /**
     * Checks if the given HSV values is in a specified range.
     *
     * @param hsv The HSV color to check.
     * @param max The maximum bounds of the HSV values
     * @param min The minimum bounds of the HSV values
     * @return Whether the given value are in the range.
     */
    public static boolean hsvInRange(float[] hsv, float[] min, float[] max) {
        for (int i = 0; i < 3; i++) {
            if (hsv[i] < min[i] || hsv[i] > max[i]) return false;
        }
        return true;
    }

    /**
     * Sets the gain of the colour sensor.
     *
     * @param gain The new gain of the colour sensor.
     */
    public void setGain(float gain) {
        colorSensor.setGain(gain);
    }

    /**
     * @return the current gain setting of the colour sensor.
     */
    public float getGain() {
        return colorSensor.getGain();
    }

    /**
     * @return The current HSV reading from the color sensor as a 3-length float array.
     * Element 0 is hue in the range [0, 360), element 1 is saturation in the range [0, 1], and element 2 is value in the range [0, 1].
     */
    public float[] getHSV() {
        NormalizedRGBA rgba = colorSensor.getNormalizedColors();
        float[] hsv = new float[]{0, 0, 0};
        Color.colorToHSV(rgba.toColor(), hsv);
        return hsv;
    }

    /**
     * @return The current rotation of the indexer (rad), in the range [0, 2*Pi) (this number wraps around)
     */
    public double getCurrentRotation() {
        return encoder.getVoltage() / 3.3 * 2 * Math.PI;
    }

    /**
     * @return the current index of the storage array, such that if the intake takes a ball, the ball colour data should be stored in this index
     */
    private int getStorageIntakeIndex() {
        return (int) (getCurrentRotation() / ((2.0 / 3.0) * Math.PI));
    }

    /**
     * Sets the indexer to move to the nearest intake slot
     */
    public void setMode(Mode newMode) {
        mode = newMode;
    }

    /**
     * detects balls entering the indexer using the color sensor and updates the internal storage array
     */
    private void detectEnteringBalls() {
        float[] hsv = getHSV();
        if (hsvInRange(hsv, PURPLE_MIN_HSV, PURPLE_MAX_HSV)) {
            int index = getStorageIntakeIndex();
            storage[index] = Ball.PURPLE;
        } else if (hsvInRange(hsv, GREEN_MIN_HSV, GREEN_MAX_HSV)) {
            int index = getStorageIntakeIndex();
            storage[index] = Ball.GREEN;
        }
    }

    @Override
    public void periodic() {
        detectEnteringBalls();

        switch (mode) {
            case IDLE:
                servo.setPosition(0.5);
                break;
            case CLOCKWISE:
                servo.setPosition(1);
                break;
            case ANTICLOCKWISE:
                servo.setPosition(0);
                break;
            // TODO: add logic for the intake and shooting modes
        }
    }
}
