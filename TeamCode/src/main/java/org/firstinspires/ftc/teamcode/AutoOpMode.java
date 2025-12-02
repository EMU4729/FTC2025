package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.DriveGoTo;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LocalisationSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@Autonomous(name = "Auto")
public class AutoOpMode extends OpMode {
    private DriveSubsystem drive;
    private IndexSubsystem index;
    private ShooterSubsystem shooter;
    private LocalisationSubsystem localisation;

    private enum State {
        MOVING_BACK,
        INITIALISING_OTOS,
        MOVING_TO_SHOOT_POSITION,
        WAITING_TO_SHOOT,
        SHOOTING,
    }

    private State state = State.MOVING_BACK;
    private DriveGoTo goToShootPositionCommand;
    private final ElapsedTime timer = new ElapsedTime();
    private double shootTime = 0;
    private double movingBackTime = 0;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        localisation = new LocalisationSubsystem(hardwareMap, telemetry);
        goToShootPositionCommand = new DriveGoTo(drive, localisation, new SparkFunOTOS.Pose2D());
        index.setStorage(IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.GREEN);
    }

    @Override
    public void start() {
        movingBackTime = timer.time();
    }

    @Override
    public void loop() {
        index.setMode(IndexSubsystem.Mode.SHOOT_ANY);
        shooter.setSpeed(1);
        switch (state) {
            case MOVING_BACK:
                drive.driveRobotRelative(0, -0.1, 0);
                if (timer.time() - movingBackTime >= 200) {
                    drive.stop();
                    state = State.INITIALISING_OTOS;
                }
                break;
            case INITIALISING_OTOS:
                if (localisation.isInitialised()) state = State.MOVING_TO_SHOOT_POSITION;
                break;
            case MOVING_TO_SHOOT_POSITION:
                goToShootPositionCommand.execute();
                if (goToShootPositionCommand.atTarget()) state = State.WAITING_TO_SHOOT;
                break;
            case WAITING_TO_SHOOT:
                if (index.atTarget() && shooter.atDesiredSpeed()) {
                    shootTime = timer.time();
                    state = State.SHOOTING;
                }
                break;
            case SHOOTING:
                shooter.pop();
                if (timer.time() - shootTime > 0.5) {
                    index.emptyCurrentSlot();
                    shooter.unpop();
                    state = State.WAITING_TO_SHOOT;
                }
                break;
        }

        drive.periodic();
        index.periodic();
        shooter.periodic();
        localisation.periodic();
        telemetry.update();
    }
}
