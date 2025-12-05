package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.GyroStraight;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OTOSLocalisationSubsystem;

@Autonomous(name = "Test Gyro Straight")
public class GyroStraightTestOpMode extends LinearOpMode {
    private OTOSLocalisationSubsystem localisation;
    private DriveSubsystem drive;
    private final ElapsedTime timer = new ElapsedTime();
    private GyroStraight gyroStraightCommand;
    private double stepStartTime = timer.time();

    @Override
    public void runOpMode() {
        this.localisation = new OTOSLocalisationSubsystem(hardwareMap, telemetry);
        this.drive = new DriveSubsystem(hardwareMap, telemetry, DcMotor.ZeroPowerBehavior.BRAKE);
        gyroStraightCommand = new GyroStraight(1, 0, drive, localisation, telemetry);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            runDirection(1, 0);
            runDirection(-1, 0);
//        runDirection(1, 0);
//        runDirection(0, -1);
//        runDirection(-1, 0);
//        runDirection(1, 1);
//        runDirection(1, -1);
//        runDirection(-1, 1);
//        runDirection(-1, -1);
        }


    }

    private void runDirection(double x, double y) {
        telemetry.addData("Current Direction", String.format("(%f, %f)", x, y));
        telemetry.update();
        stepStartTime = timer.time();
        gyroStraightCommand.setDirection(x, y);
        gyroStraightCommand.start();
        do {
            gyroStraightCommand.execute();
        } while (timer.time() - stepStartTime < 1 && opModeIsActive());
        stepStartTime = timer.time();
        do {
            gyroStraightCommand.end();
        } while (timer.time() - stepStartTime < 1 && opModeIsActive());
    }
}
