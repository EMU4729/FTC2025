package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Pop Servo Tuning")
public class PopTuningOpMode extends OpMode {
    private Servo popServo;
    private double value = 0;

    @Override
    public void init() {
        popServo = hardwareMap.get(Servo.class, "shooterPop");
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up) {
            value = Math.min(value + 0.001, 1);
        } else if (gamepad1.dpad_down) {
            value = Math.max(value - 0.001, 0);
        }
        popServo.setPosition(value);
        telemetry.addData("Servo Position", value);
        telemetry.update();
    }


}
