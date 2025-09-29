package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;

@TeleOp(name = "Tune Index Color Sensor HSV Bounds")
public class IndexHSVBoundsTuningOpMode extends OpMode {
    private static final float HUE_STEP = 1f;
    private static final float OTHER_STEP = 0.05f;
    private static final String[] VALUE_LABELS = {
            "Upper Hue",
            "Upper Saturation",
            "Upper Value",
            "Lower Hue",
            "Lower Saturation",
            "Lower Value"
    };
    private IndexSubsystem index;
    private int currentValue = 0;
    private final float[] values = {0, 0, 0, 359, 1, 1};

    @Override
    public void init() {
        index = new IndexSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            if (currentValue == 0 || currentValue == 3) {
                values[currentValue] += HUE_STEP;
            } else {
                values[currentValue] += OTHER_STEP;
            }
        } else if (gamepad1.dpadDownWasPressed()) {
            if (currentValue == 0 || currentValue == 3) {
                values[currentValue] -= HUE_STEP;
            } else {
                values[currentValue] -= OTHER_STEP;
            }
        } else if (gamepad1.dpadLeftWasPressed()) {
            currentValue = (currentValue - 1) % 6;
        } else if (gamepad1.dpadRightWasPressed()) {
            currentValue = (currentValue + 1) % 6;
        }

        float[] lower = new float[]{values[0], values[1], values[2]};
        float[] upper = new float[]{values[3], values[4], values[5]};

        telemetry.addData("Currently Modifying", VALUE_LABELS[currentValue]);
        telemetry.addData("HSV Upper Bound", upper);
        telemetry.addData("HSV Lower Bound", lower);
        telemetry.addData("Current HSV", index.getHSV());
        telemetry.addData("In Bounds?", IndexSubsystem.hsvInRange(index.getHSV(), lower, upper));
        telemetry.update();
    }
}
