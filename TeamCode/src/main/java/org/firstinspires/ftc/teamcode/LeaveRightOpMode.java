package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;

@Autonomous(name = "Leave Right")
public class LeaveRightOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, telemetry, DcMotor.ZeroPowerBehavior.BRAKE);
        ElapsedTime timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        do {
            drive.driveRobotRelative(0, -0.5, 0);
        } while (timer.time() <= 2 && opModeIsActive());
        do {
            drive.stop();
        } while (opModeIsActive());
    }
}
