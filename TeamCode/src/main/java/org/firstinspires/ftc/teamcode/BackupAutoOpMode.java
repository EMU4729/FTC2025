package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;


@Autonomous(name = "Backup Auto")
public class BackupAutoOpMode extends OpMode {
    private DriveSubsystem drive;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        timer.reset();
        while (timer.time() < 0.5)
            drive.driveRobotRelative(0.5, 0, 0);
        drive.stop();
    }

    @Override
    public void loop() {
    }
}
