package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;

@TeleOp(name = "Index Tuning")
public class IndexTuningOpMode extends OpMode {
    private static final float HUE_STEP = 1f;
    private static final float OTHER_STEP = 0.05f;
    private static final float GAIN_STEP = 0.05f;
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
    private double indexPosition = 0;

    @Override
    public void init() {
        index = new IndexSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // editing bounds value -> up and down dpad
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
        }

        // changing bounds value being edited -> left and right dpad
        if (gamepad1.dpadLeftWasPressed()) {
            currentValue = (currentValue - 1) % 6;
        } else if (gamepad1.dpadRightWasPressed()) {
            currentValue = (currentValue + 1) % 6;
        }

        // indexer rotation -> left stick x
        if (gamepad1.left_stick_x > 0.1) {
            indexPosition += 0.01;
        } else if (gamepad1.left_stick_x < -0.1) {
            indexPosition -= 0.01;
        }
        index.unsafe_setPosition(indexPosition);

        // gain adjustment -> left and right bumper
        if (gamepad1.rightBumperWasPressed()) {
            index.setGain(index.getGain() + GAIN_STEP);
        } else if (gamepad1.leftBumperWasPressed()) {
            index.setGain(index.getGain() - GAIN_STEP);
        }

        float[] lower = new float[]{values[0], values[1], values[2]};
        float[] upper = new float[]{values[3], values[4], values[5]};
        float[] hsv = index.getHSV();

        telemetry.addData("Currently Modifying", VALUE_LABELS[currentValue]);
        telemetry.addData("HSV Upper Bound", String.format("(%f, %f, %f)", upper[0], upper[1], upper[2]));
        telemetry.addData("HSV Lower Bound", String.format("(%f, %f, %f)", lower[0], lower[1], lower[2]));
        telemetry.addData("Current HSV", String.format("(%f, %f, %f)", hsv[0], hsv[1], hsv[2]));
        telemetry.addData("Current Gain", index.getGain());
        telemetry.addData("In Bounds?", IndexSubsystem.hsvInRange(index.getHSV(), lower, upper));
        index.periodic();
        telemetry.update();
    }
}
