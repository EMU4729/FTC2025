package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.GyroStraight;
import org.firstinspires.ftc.teamcode.lib.GyroTurn;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OTOSLocalisationSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@Autonomous(name = "Blue Three Ball Auto")
public class BlueThreeBallAutoOpMode extends LinearOpMode {
    private DriveSubsystem drive;
    private OTOSLocalisationSubsystem localisation;
    private IndexSubsystem index;
    private ShooterSubsystem shooter;
    private LEDSubsystem led;

    @Override
    public void runOpMode() {
        drive = new DriveSubsystem(hardwareMap, telemetry, DcMotor.ZeroPowerBehavior.BRAKE);
        localisation = new OTOSLocalisationSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        led = new LEDSubsystem(hardwareMap, telemetry);

        GyroStraight gyroStraightCommand = new GyroStraight(0, 1, drive, localisation, telemetry);
        GyroTurn gyroTurnCommand = new GyroTurn(Math.PI / 2, drive, localisation, telemetry);
        ElapsedTime timer = new ElapsedTime();

        index.setStorage(IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.PURPLE, IndexSubsystem.Ball.PURPLE);

        waitForStart();
        index.setMode(IndexSubsystem.Mode.SHOOT_ANY);

        // go right to see obelisk
        timer.reset();
        gyroStraightCommand.start();
        do {
            gyroStraightCommand.execute();
            periodic();
        } while (timer.time() <= 1 && opModeIsActive());
        gyroStraightCommand.end();

        // wait for motif detection
        timer.reset();
        do {
            drive.stop();
            periodic();
        } while (!localisation.getMotif().isPresent() && timer.time() <= 0.5 && opModeIsActive());

        // prime the indexer and shooter
        if (!localisation.getMotif().isPresent()) {
            led.setSolidColor(Color.RED);
            led.setMode(LEDSubsystem.Mode.SOLID);
        }

        IndexSubsystem.Ball[] motif = localisation.getMotif().orElse(new IndexSubsystem.Ball[]{IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.PURPLE, IndexSubsystem.Ball.PURPLE});
        int motifIndex = 0;
        IndexSubsystem.Mode shootMode;
        switch (motif[motifIndex]) {
            case PURPLE:
                shootMode = IndexSubsystem.Mode.SHOOT_PURPLE_ONLY;
                break;
            case GREEN:
                shootMode = IndexSubsystem.Mode.SHOOT_GREEN_ONLY;
                break;
            default:
                shootMode = IndexSubsystem.Mode.SHOOT_ANY;
                break;
        }
        shooter.setSpeed(67);
        index.setMode(shootMode);

        // turn to face goal
        gyroTurnCommand.start();
        timer.reset();
        do {
            gyroTurnCommand.execute();
            periodic();
        } while (timer.time() < 2 && opModeIsActive());
        gyroTurnCommand.end();

        // shoot balls in motif pattern
        do {
            switch (motif[motifIndex]) {
                case PURPLE:
                    shootMode = IndexSubsystem.Mode.SHOOT_PURPLE_ONLY;
                    break;
                case GREEN:
                    shootMode = IndexSubsystem.Mode.SHOOT_GREEN_ONLY;
                    break;
                default:
                    shootMode = IndexSubsystem.Mode.SHOOT_ANY;
                    break;
            }

            timer.reset();
            do {
                shooter.setSpeed(67);
                index.setMode(shootMode);
                periodic();
            } while ((Math.abs(shooter.getMotorSpeed() - 65) >= 0.5 || timer.time() <= 1) && opModeIsActive());

            timer.reset();
            do {
                shooter.pop();
                periodic();
            } while (timer.time() <= 1.5 && opModeIsActive());

            timer.reset();
            do {
                shooter.unpop();
                periodic();
            } while (timer.time() <= 0.5 && opModeIsActive());

            motifIndex++;
            index.emptyCurrentSlot();
            periodic();
        } while (motifIndex < 3 && opModeIsActive());
        shooter.setSpeed(0);

        // leave
        timer.reset();
        do {
            drive.driveRobotRelative(-0.3, 1, 0);
            periodic();
        } while (timer.time() < 1 && opModeIsActive());

        do {
            drive.stop();
            periodic();
        } while (opModeIsActive());
    }

    private void periodic() {
        drive.periodic();
        index.periodic();
        localisation.periodic();
        shooter.periodic();
        led.periodic();
        telemetry.update();
    }
}
