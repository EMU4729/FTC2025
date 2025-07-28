package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;

@TeleOp(name = "Intake Calibration")
public class IntakeCalibrationTeleOp extends OpMode {
    IntakeSubsystem intake;
    IntakeSubsystem.State intakeState;

    @Override
    public void init() {
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        intakeState = IntakeSubsystem.State.HOVER;
    }

    @Override
    public void loop() {
        if (gamepad2.dpad_down) {
            intakeState.pivot -= 0.01;
        } else if (gamepad2.dpad_up) {
            intakeState.pivot += 0.01;
        }

        if (gamepad2.dpad_left) {
            intakeState.rotate -= 0.01;
        } else if (gamepad2.dpad_right) {
            intakeState.rotate += 0.01;
        }

        if (gamepad2.left_bumper) {
            intakeState.grip -= 0.01;
        } else if (gamepad2.right_bumper) {
            intakeState.grip += 0.01;
        }

        intake.applyState(intakeState);

        telemetry.update();
    }
}