package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;

@TeleOp(name = "TeleOp")
public class TeleopOpMode extends OpMode {
    DriveSubsystem drive;
    LiftSubsystem lift;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        drive.driveRobotRelative(-gamepad2.left_stick_y, gamepad2.left_stick_x, -gamepad2.right_stick_x);
        if (gamepad1.dpad_up) {
            lift.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            lift.setPower(-0.5);
        } else {
            lift.setPower(0);
        }
        drive.periodic();
        lift.periodic();
        telemetry.update();
    }
}
