package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.OuttakeSubsystem;

@TeleOp(name = "Outtake Calibration")
public class OuttakeCalibrationOpMode extends OpMode {
    OuttakeSubsystem outtake;
    double grab = 0;
    double leftPivot = 0;
    double rightPivot = 0;
    double leftRotate = 0;
    double rightRotate = 0;
    boolean controllingRotate = false;

    @Override
    public void init() {
        outtake = new OuttakeSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad2.startWasPressed()) {
            controllingRotate = !controllingRotate;
            telemetry.addData("Sticks are currently controlling", controllingRotate ? "Rotation Servos" : "Pivot Servos");
        }

        double leftDelta = 0;
        if (gamepad2.left_stick_y < -0.5) {
            leftDelta = 0.001;
        } else if (gamepad2.left_stick_y > 0.5) {
            leftDelta = -0.001;
        }

        double rightDelta = 0;
        if (gamepad2.right_stick_y < -0.5) {
            rightDelta = 0.001;
        } else if (gamepad2.right_stick_y > 0.5) {
            rightDelta = -0.001;
        }

        if (controllingRotate) {
            leftRotate += leftDelta;
            rightRotate += rightDelta;
        } else {
            leftPivot += leftDelta;
            rightPivot += rightDelta;
        }

        if (gamepad2.right_bumper) {
            grab += 0.001;
        } else if (gamepad2.left_bumper) {
            grab -= 0.001;
        }

        outtake.unsafe_setAllPositions(grab, leftPivot, rightPivot, leftRotate, rightRotate);

        telemetry.update();
    }
}
