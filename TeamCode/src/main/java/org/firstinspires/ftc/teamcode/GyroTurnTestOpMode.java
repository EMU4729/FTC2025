package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.GyroTurn;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OTOSLocalisationSubsystem;

@Autonomous(name = "Test Gyro Turn")
public class GyroTurnTestOpMode extends LinearOpMode {
    private OTOSLocalisationSubsystem localisation;
    private DriveSubsystem drive;
    private final ElapsedTime timer = new ElapsedTime();
    private GyroTurn gyroTurnCommand;
    private double stepStartTime = timer.time();

    @Override
    public void runOpMode() {
        localisation = new OTOSLocalisationSubsystem(hardwareMap, telemetry);
        drive = new DriveSubsystem(hardwareMap, telemetry, DcMotor.ZeroPowerBehavior.BRAKE);
        gyroTurnCommand = new GyroTurn(0, drive, localisation, telemetry);

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            runTurn(Math.PI / 2);
            // runDirection(1, 0);
            // runDirection(0, -1);
            // runDirection(-1, 0);
            // runDirection(1, 1);
            // runDirection(1, -1);
            // runDirection(-1, 1);
            // runDirection(-1, -1);
        }
    }

    private void runTurn(double targetYawDelta) {
        telemetry.addData("Current Target Yaw Delta", targetYawDelta);
        telemetry.update();
        stepStartTime = timer.time();
        gyroTurnCommand.setTarget(targetYawDelta);
        gyroTurnCommand.start();
        do {
            gyroTurnCommand.execute();
        } while (timer.time() - stepStartTime < 2 && opModeIsActive());
        stepStartTime = timer.time();
        do {
            gyroTurnCommand.end();
        } while (timer.time() - stepStartTime < 2 && opModeIsActive());
    }
}
