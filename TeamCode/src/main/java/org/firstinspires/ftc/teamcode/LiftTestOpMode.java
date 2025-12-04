package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;

@TeleOp(name = "Lift Test")
public class LiftTestOpMode extends OpMode {
    private LiftSubsystem lift;
    private double leftLockPosition = 1;
    private double rightLockPosition = 1;

    public void init() {
        lift = new LiftSubsystem(hardwareMap, telemetry);
    }

    public void loop() {
        if (gamepad1.dpad_left) {
            leftLockPosition = Math.max(leftLockPosition - 0.001, 0);
        } else if (gamepad1.dpad_right) {
            leftLockPosition = Math.min(leftLockPosition + 0.001, 1);
        }
        if (gamepad1.dpad_up) {
            rightLockPosition = Math.max(rightLockPosition - 0.001, 0);
        } else if (gamepad1.dpad_down) {
            rightLockPosition = Math.min(rightLockPosition + 0.001, 1);
        }
        lift.setLockPosition(leftLockPosition, rightLockPosition);

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

        telemetry.addData("Lift Left Lock Position", leftLockPosition);
        telemetry.addData("Lift Right Lock Position", rightLockPosition);

        lift.periodic();
        telemetry.update();
    }
}
