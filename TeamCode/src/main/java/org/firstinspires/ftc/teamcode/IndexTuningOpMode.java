package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;

@TeleOp(name = "Index Tuning")
public class IndexTuningOpMode extends OpMode {
    private IndexSubsystem index;
    private double indexPosition = 0;

    @Override
    public void init() {
        index = new IndexSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // indexer rotation -> left stick x
        if (gamepad1.left_stick_x > 0.1) {
            indexPosition = Math.min(indexPosition + 0.01, 1);
        } else if (gamepad1.left_stick_x < -0.1) {
            indexPosition = Math.max(indexPosition - 0.01, 0);
        }
        index.unsafe_setPosition(indexPosition);

        // gain adjustment -> left and right bumper
        if (gamepad1.rightBumperWasPressed()) {
            index.setGain(index.getGain() + 0.01f);
        } else if (gamepad1.leftBumperWasPressed()) {
            index.setGain(index.getGain() - 0.01f);
        }

        index.periodic();
        telemetry.update();
    }
}
