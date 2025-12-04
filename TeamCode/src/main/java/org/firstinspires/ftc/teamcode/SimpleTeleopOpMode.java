package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.LiftRaise;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OTOSLocalisationSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@TeleOp(name = "Simple Teleop")
public class SimpleTeleopOpMode extends OpMode {
    private static final boolean DISABLE_COLOR_SENSOR = true;

    DriveSubsystem drive;
    LiftSubsystem lift;
    IntakeSubsystem intake;
    IndexSubsystem index;
    ShooterSubsystem shooter;
    //    LEDSubsystem led;
    OTOSLocalisationSubsystem localisation;
    private double shootTime = 0;
    private double shooterTilt = 1;
    private LiftRaise liftRaiseCommand;


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        POPPING,
        UNPOPPING,
    }

    private LaunchState launchState;

    private final ElapsedTime timer = new ElapsedTime();
    private int manualIndexerIndex = 0;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry, DISABLE_COLOR_SENSOR);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        localisation = new OTOSLocalisationSubsystem(hardwareMap, telemetry);
        launchState = LaunchState.IDLE;
        liftRaiseCommand = new LiftRaise(localisation, lift);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        drive.driveRobotRelative(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);

        // for human player intake
        if (gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5) {
            index.setMode(IndexSubsystem.Mode.INTAKE_MANUAL); // switch to indexer slot
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        // Rotate indexer
        if (gamepad1.dpadLeftWasPressed() || gamepad2.dpadLeftWasPressed()) {
            if (manualIndexerIndex == 0) {
                manualIndexerIndex = 2;
            } else {
                manualIndexerIndex--;
            }
        }
        if (gamepad1.dpadRightWasPressed() || gamepad2.dpadRightWasPressed()) {
            if (manualIndexerIndex == 2) {
                manualIndexerIndex = 0;
            } else {
                manualIndexerIndex++;
            }
        }
        index.setManualIndex(manualIndexerIndex);

        if ((gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) && launchState == LaunchState.IDLE) {
            launchState = LaunchState.SPIN_UP;
            shootTime = timer.time();
        } else if (gamepad1.right_trigger <= 0.5 && gamepad2.right_trigger <= 0.5) {
            launchState = LaunchState.IDLE;
        }

        switch (launchState) {
            case IDLE:
                shooter.setSpeed(0);
                shooter.unpop();
                break;

            case SPIN_UP:
                shooter.setSpeed(100);
                index.setMode(IndexSubsystem.Mode.SHOOT_MANUAL);
                if (shooter.getMotorSpeed() >= 67 && timer.time() - shootTime > 1 && (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5)) {
                    launchState = LaunchState.POPPING;
                    shootTime = timer.time();
                }
                break;

            case POPPING:
                shooter.pop();
                if (timer.time() - shootTime > 0.5) {
                    shootTime = timer.time();
                    launchState = LaunchState.UNPOPPING;
                }
                break;

            case UNPOPPING:
                shooter.unpop();
                if (timer.time() - shootTime > 0.5) {
                    manualIndexerIndex = (manualIndexerIndex + 1) % 3;
                    index.setManualIndex(manualIndexerIndex);
                    launchState = LaunchState.SPIN_UP;
                }
                break;
        }

        // Raises or lowers lift
        // lift control
        if (gamepad1.rightBumperWasPressed()) {
            liftRaiseCommand.start();
        } else if (gamepad1.right_bumper) {
            liftRaiseCommand.execute();
        } else if (gamepad1.rightBumperWasReleased()) {
            liftRaiseCommand.end();
        }

        // Shooter arc control
        if (gamepad2.dpad_up || gamepad1.dpad_up) {
            shooterTilt = Math.min(shooterTilt + 0.05, 1);
        } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
            shooterTilt = Math.max(shooterTilt - 0.05, 0);
        }
        shooter.setTilt(shooterTilt);

        drive.periodic();
        lift.periodic();
        intake.periodic();
        index.periodic();
        shooter.periodic();
        telemetry.update();
    }
}
