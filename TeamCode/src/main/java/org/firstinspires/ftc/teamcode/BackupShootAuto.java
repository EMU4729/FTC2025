package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@Autonomous(name = "Backup Shoot Auto")
public class BackupShootAuto extends OpMode {
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IndexSubsystem index;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        timer.reset();

        // drive back for 0.2s
        double driveTime = timer.time();
        while (timer.time() - driveTime < 0.2) {
            drive.driveRobotRelative(0.5, 0, 0);
        }
        drive.stop();

        // fire all balls
        index.setMode(IndexSubsystem.Mode.SHOOT_MANUAL);
        for (int i = 0; i < 3; i++) {
            do {
                index.setManualIndex(i);
                shooter.setSpeed(100);
            } while (!index.atTarget() || shooter.getMotorSpeed() < 60);

            double shootTime = timer.time();
            do {
                shooter.pop();
            } while (timer.time() - shootTime < 0.3);

            double unpopTime = timer.time();
            do {
                shooter.unpop();
            } while (timer.time() - unpopTime < 0.2);
        }

        driveTime = timer.time();
        while (timer.time() - driveTime < 0.2){
            drive.driveRobotRelative(0, 0.5, 0);
        }
        drive.stop();
    }

    @Override
    public void loop() {}
}
