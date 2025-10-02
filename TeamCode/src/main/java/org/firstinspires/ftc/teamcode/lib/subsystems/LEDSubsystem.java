package org.firstinspires.ftc.teamcode.lib.subsystems;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.w8wjb.ftc.AdafruitNeoDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LEDSubsystem extends SubsystemBase {
    private static final int STRING_LENGTH = 50; // todo: change

    private final AdafruitNeoDriver leds;
    private int hue = 0;

    public LEDSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leds = hardwareMap.get(AdafruitNeoDriver.class, "leds");
        leds.setNumberOfPixels(STRING_LENGTH);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < STRING_LENGTH; i++) {
            int color = Color.HSVToColor(new float[]{(float) hue, 1f, 1f});
            leds.setPixelColor(i, color);
        }
        hue = (hue + 1) % 360;
    }
}
