package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;

@TeleOp(name = "Intake Test")
public class IntakeTestOpMode extends OpMode {
    private IntakeSubsystem intake;

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.5) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
        intake.periodic();
        telemetry.update();
    }
}
