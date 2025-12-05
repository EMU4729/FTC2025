package org.firstinspires.ftc.teamcode.lib.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.w8wjb.ftc.AdafruitNeoDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LEDSubsystem extends SubsystemBase {
    private static final int STRING_LENGTH = 50; // todo: change
    private static final boolean DISABLE = true;

    public enum Mode {
        RAINBOW,
        SOLID,
        OFF
    }

    private final AdafruitNeoDriver leds;
    private int hue = 0;
    private Mode mode = Mode.OFF;
    private int solidColor = Color.BLACK;

    public LEDSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leds = hardwareMap.get(AdafruitNeoDriver.class, "leds");
        if (DISABLE) return;
        leds.setNumberOfPixels(STRING_LENGTH);
    }

    /**
     * Sets the mode of the LEDs. If SOLID is selected, use the {@link LEDSubsystem#setSolidColor} method to set the desired color.
     *
     * @param mode The desired LED mode - either OFF, SOLID or RAINBOW.
     */
    public void setMode(Mode mode) {
        this.mode = mode;
    }

    /**
     * Sets the LEDs to a solid color - also modifies the subsystem's mode.
     *
     * @param color The desired color.
     */
    public void setSolidColor(int color) {
        solidColor = color;
        mode = Mode.SOLID;
    }

    @Override
    public void periodic() {
        if (DISABLE) return;
        switch (mode) {
            case SOLID:
                leds.fill(solidColor);
                break;
            case OFF:
                leds.fill(Color.BLACK);
                break;
            case RAINBOW:
                for (int i = 0; i < STRING_LENGTH; i++) {
                    int color = Color.HSVToColor(new float[]{(float) (hue + i) % 360, 1f, 1f});
                    leds.setPixelColor(i, color);
                }
                hue = (hue + 1) % 360;

        }
    }
}
