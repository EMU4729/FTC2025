package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;

@TeleOp(name = "Lift Test")
public class LiftTestOpMode extends OpMode {
    private LiftSubsystem lift;
    private double lockPosition = 1;

    public void init() {
        lift = new LiftSubsystem(hardwareMap, telemetry);
    }

    public void loop() {
        if (gamepad1.dpad_left) {
            lockPosition = Math.max(lockPosition - 0.01, 0);
        } else if (gamepad1.dpad_right) {
            lockPosition = Math.min(lockPosition + 0.01, 1);
        }
        lift.setLockPosition(lockPosition);

        if (gamepad1.left_bumper) {
            lift.setLeftPower(1);
        } else {
            lift.setLeftPower(0);
        }

        if (gamepad1.right_bumper) {
            lift.setRightPower(1);
        } else {
            lift.setRightPower(0);
        }
    }
}
