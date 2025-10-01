package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;

@TeleOp(name = "Tune Index Color Sensor", group = "Tuning")
public class IndexColorGainTuningOpMode extends OpMode {
    private static final float GAIN_STEP = 0.05f;

    private IndexSubsystem index;

    @Override
    public void init() {
        index = new IndexSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) {
            index.setGain(index.getGain() + GAIN_STEP);
        } else if (gamepad1.dpadDownWasPressed()) {
            index.setGain(index.getGain() - GAIN_STEP);
        }

        telemetry.addData("Gain", index.getGain());
        telemetry.addData("HSV", index.getHSV());
        telemetry.update();
    }
}
