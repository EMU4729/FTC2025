package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;

@TeleOp(name = "TeleOp")
public class TeleopOpMode extends OpMode {
    DriveSubsystem drive;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        drive.driveRobotRelative(-gamepad2.left_stick_y, gamepad2.left_stick_x, -gamepad2.right_stick_x);
        drive.periodic();
        telemetry.update();
    }
}
