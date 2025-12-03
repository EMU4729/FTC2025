package org.firstinspires.ftc.teamcode.lib.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.PIDController;

import java.util.function.Function;

public class IndexSubsystem extends SubsystemBase {
    /**
     * The different modes of operation of the indexer
     */
    public enum Mode {
        IDLE,
        INTAKE,
        SHOOT_ANY,
        SHOOT_PURPLE_ONLY,
        SHOOT_GREEN_ONLY,
        CLOCKWISE,
        ANTICLOCKWISE,
        INTAKE_MANUAL,
        SHOOT_MANUAL
    }

    private static final float COLOR_SENSOR_GAIN = 1; // TODO: tune
    private static final float[] PURPLE_MIN_HSV = new float[]{220, 0.3f, 0}; // TODO: tune
    private static final float[] PURPLE_MAX_HSV = new float[]{240, 1, 0.1f}; // TODO: tune
    private static final float[] GREEN_MIN_HSV = new float[]{160, 0.5f, 0}; // TODO: tune
    private static final float[] GREEN_MAX_HSV = new float[]{180, 1, 1}; // TODO: tune

    private static final double[] INTAKE_ROTATIONS = new double[]{Math.toRadians(115), Math.toRadians(235), Math.toRadians(2)}; // TODO: tune
    private static final double[] SHOOT_ROTATIONS = new double[]{Math.toRadians(280), Math.toRadians(40), Math.toRadians(162)}; // TODO: tune
    private final PIDController controller = new PIDController(0.08, 0, 0, 0);

    public enum Ball {
        GREEN,
        PURPLE,
        EMPTY
    }

    private final Telemetry telemetry;
    private final Servo servo;
    private final AnalogInput encoder;
    private final NormalizedColorSensor colorSensor;
    private final Ball[] storage = new Ball[]{Ball.EMPTY, Ball.EMPTY, Ball.EMPTY};
    private final boolean disableColorSensor;
    private Mode mode = Mode.IDLE;
    private double rotation = 0;
    private boolean atTarget = false;
    private boolean ballRecentlyIntaken = false;
    private int manualIndex = 0;

    public IndexSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, false);
    }

    public IndexSubsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean disableColorSensor) {
        this.telemetry = telemetry;
        this.disableColorSensor = disableColorSensor;
        servo = hardwareMap.get(Servo.class, "indexServo");
        servo.setDirection(Servo.Direction.REVERSE);
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
     * Sets the slot index to rotate to in the intake and shooting manual modes
     *
     * @param index The slot index to go to
     */
    public void setManualIndex(int index) {
        controller.resetIntegral();
        manualIndex = index;
    }

    /**
     * Overwrites the internal indexer storage array. USE WITH CAUTION - THIS CAN EASILY BREAK THINGS.
     *
     * @param first  The ball in the first slot
     * @param second The ball in the second slot
     * @param third  The ball in the third slot.
     */
    public void setStorage(Ball first, Ball second, Ball third) {
        storage[0] = first;
        storage[1] = second;
        storage[2] = third;
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
     * Updates the rotation variable with the of the indexer
     */
    private void updateRotation() {
        // the range of the encoder voltage reading is from 0 - 3.3V
        rotation = encoder.getVoltage() / 3.3 * 2 * Math.PI;
    }

    /**
     * @return The current rotation of the indexer (rad), in the range [0, 2*Pi) (this number wraps around)
     */
    public double getRotation() {
        // the range of the encoder voltage reading is from 0 - 3.3V
        return rotation;
    }

    /**
     * Sets the indexer to the given mode.
     */
    public void setMode(Mode newMode) {
        mode = newMode;
        controller.resetIntegral();
    }

    /**
     * @return If the indexer is at the target position, if using a "smart" mode.
     */
    public boolean atTarget() {
        // we ignore `atTarget` if we are not in a "smart" mode
        return (mode == Mode.INTAKE || mode == Mode.SHOOT_ANY || mode == Mode.SHOOT_GREEN_ONLY || mode == Mode.SHOOT_PURPLE_ONLY || mode == Mode.INTAKE_MANUAL || mode == Mode.SHOOT_MANUAL) && atTarget;
    }

    /**
     * Resets var ballRecentlyIntaken to false
     *
     * @return true if ball is intaken recently or false if not
     */
    public boolean ballIntaken() {
        if (ballRecentlyIntaken) {
            ballRecentlyIntaken = false;
            return true;
        }
        return false;
    }

    /**
     * Returns the relative angle between two angles, accounting for continuous rotation
     *
     * @param a The first angle, in the range [0, 360]
     * @param b The second angle, in the range [0, 360]
     * @return The wrapped signed angle between a and b
     */
    private double wrappedSignedAngleBetween(double a, double b) {
        double result = a - b;
        if (result < -Math.PI) result += 2 * Math.PI;
        else if (result > Math.PI) result -= 2 * Math.PI;
        return result;
    }

    /**
     * Gets the slot index corresponding to the closest rotation matching the predicate
     *
     * @param rotations A 3-length array of the rotations corresponding to each indexer slot
     * @param predicate This function ignores any slots that cause this function to return false
     * @return The index corresponding to the closest rotation matching the predicate
     */
    private int closestSlot(double[] rotations, Function<Integer, Boolean> predicate) {
        int closestSlotIndex = -1;
        double minError = Double.POSITIVE_INFINITY;
        for (int i = 0; i < 3; i++) {
            if (!predicate.apply(i)) continue;
            double error = wrappedSignedAngleBetween(rotations[i], rotation);
            if (Math.abs(error) < Math.abs(minError)) {
                minError = error;
                closestSlotIndex = i;
            }
        }

        return closestSlotIndex;
    }

    /**
     * This function updates the internal storage array to have a green ball in the closest intake
     * position. THIS IS DANGEROUS, DO NOT USE UNLESS YOU KNOW WHAT YOU'RE DOING.
     */
    public void unsafe_manuallyMarkIntake() {
        int closestSlotIndex = closestSlot(INTAKE_ROTATIONS, (i) -> storage[i] == Ball.EMPTY);
        storage[closestSlotIndex] = Ball.GREEN;
        ballRecentlyIntaken = true;
    }

    /**
     * Detects any entering balls and sets them in the storage array with the given index
     *
     * @param slotIndex The index to store the detected ball in
     */
    private void detectEnteringBalls(int slotIndex) {
        if (disableColorSensor) return;
        float[] hsv = getHSV();
        if (hsvInRange(hsv, GREEN_MIN_HSV, GREEN_MAX_HSV)) {
            storage[slotIndex] = Ball.GREEN;
            ballRecentlyIntaken = true;
        } else if (hsvInRange(hsv, PURPLE_MIN_HSV, PURPLE_MAX_HSV)) {
            storage[slotIndex] = Ball.PURPLE;
            ballRecentlyIntaken = true;
        }
    }

    /**
     * Publishes some useful data to telemetry.
     */
    private void updateTelemetry() {
        telemetry.addData("Indexer Rotation (deg)", Math.toDegrees(rotation));
        telemetry.addData("Indexer At Target", atTarget());
        telemetry.addData("Indexer Mode", mode);
        telemetry.addData("Indexer Servo Power", servo.getPosition());
        telemetry.addData("Indexer Color Sensor Raw", colorSensor.getNormalizedColors().toColor());
        telemetry.addData("Indexer Color Sensor Conn Info", colorSensor.getConnectionInfo());
        telemetry.addData("Indexer Manual Index", manualIndex);
        telemetry.addData("Indexer Storage", String.format("(%s, %s, %s)", storage[0], storage[1], storage[2]));
        telemetry.addData("Indexer Detecting Green", hsvInRange(getHSV(), GREEN_MIN_HSV, GREEN_MAX_HSV));
        telemetry.addData("Indexer Detecting Purple", hsvInRange(getHSV(), PURPLE_MIN_HSV, PURPLE_MAX_HSV));
    }

    /**
     * Empty the current slot (from the perspective of the shooter).
     */
    public void emptyCurrentSlot() {
        int closestSlotIndex = closestSlot(SHOOT_ROTATIONS, (i) -> storage[i] != Ball.EMPTY);
        storage[closestSlotIndex] = Ball.EMPTY;
    }

    @Override
    public void periodic() {
        updateRotation();
        updateTelemetry();

        switch (mode) {
            case IDLE:
                servo.setPosition(0.5);
                return;
            case CLOCKWISE:
                servo.setPosition(0.55);
                return;
            case ANTICLOCKWISE:
                servo.setPosition(0.45);
                return;
        }

        // "smart" mode logic

        double[] rotations = new double[]{0, 0, 0};
        int closestSlotIndex = -1;

        switch (mode) {
            case INTAKE:
                rotations = INTAKE_ROTATIONS;
                closestSlotIndex = closestSlot(INTAKE_ROTATIONS, (i) -> storage[i] == Ball.EMPTY);
                detectEnteringBalls(closestSlotIndex);
                break;
            case SHOOT_ANY:
                rotations = SHOOT_ROTATIONS;
                closestSlotIndex = closestSlot(SHOOT_ROTATIONS, (i) -> storage[i] != Ball.EMPTY);
                break;
            case SHOOT_GREEN_ONLY:
                rotations = SHOOT_ROTATIONS;
                closestSlotIndex = closestSlot(SHOOT_ROTATIONS, (i) -> storage[i] == Ball.GREEN);
                break;
            case SHOOT_PURPLE_ONLY:
                rotations = SHOOT_ROTATIONS;
                closestSlotIndex = closestSlot(SHOOT_ROTATIONS, (i) -> storage[i] == Ball.PURPLE);
                break;
            case SHOOT_MANUAL:
                rotations = SHOOT_ROTATIONS;
                closestSlotIndex = manualIndex;
                break;
            case INTAKE_MANUAL:
                rotations = INTAKE_ROTATIONS;
                closestSlotIndex = manualIndex;
                break;
        }

        if (closestSlotIndex == -1) {
            // no available closest slot, idle
            atTarget = false;
            servo.setPosition(0.5);
            return;
        }

        double error = -wrappedSignedAngleBetween(rotation, rotations[closestSlotIndex]);
        telemetry.addData("Indexer Error", error);
        servo.setPosition(0.5 + controller.calculate(error));
        atTarget = Math.abs(error) < 0.05;
    }
}
