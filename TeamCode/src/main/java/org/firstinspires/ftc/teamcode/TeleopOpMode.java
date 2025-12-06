package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.LiftRaise;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OTOSLocalisationSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@TeleOp(name = "Teleop")
public class TeleopOpMode extends OpMode {
    private DriveSubsystem drive;
    private LiftSubsystem lift;
    private IntakeSubsystem intake;
    private IndexSubsystem index;
    private ShooterSubsystem shooter;
    private LEDSubsystem led;
    private OTOSLocalisationSubsystem localisation;
    private double shootTime = 0;
    private double shooterTilt = 1;
    private LiftRaise liftRaiseCommand;


    private enum LaunchState {
        IDLE,
        REVERSE_INTAKE,
        INTAKING,
        CONFIRM_INTAKE,
        SPIN_UP,
        POPPING,
        UNPOPPING,
    }

    private LaunchState launchState;
    private IndexSubsystem.Mode shootMode = IndexSubsystem.Mode.SHOOT_ANY;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime intakeTimer = new ElapsedTime();

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        localisation = new OTOSLocalisationSubsystem(hardwareMap, telemetry);
        led = new LEDSubsystem(hardwareMap, telemetry);
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

        // colour selection
        if (gamepad1.a || gamepad2.a) {
            shootMode = IndexSubsystem.Mode.SHOOT_GREEN_ONLY;
            led.setSolidColor(Color.GREEN);
        } else if (gamepad1.x || gamepad2.x) {
            shootMode = IndexSubsystem.Mode.SHOOT_PURPLE_ONLY;
            led.setSolidColor(Color.BLUE);
        } else if (gamepad1.b || gamepad2.b) {
            shootMode = IndexSubsystem.Mode.SHOOT_ANY;
            led.setSolidColor(Color.RED);
        }
        led.setMode(LEDSubsystem.Mode.SOLID);
        telemetry.addData("Shooting Mode", shootMode);

        // for human player intake
        boolean intakeInput = gamepad1.left_trigger > 0.5 || gamepad2.left_trigger > 0.5;
        boolean reverseIntakeInput = gamepad1.left_bumper || gamepad2.left_bumper;
        boolean revInput = gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5;
        boolean shootInput = gamepad1.y || gamepad2.y;

        switch (launchState) {
            case IDLE:
                intake.setPower(0);
                shooter.setSpeed(0);
                shooter.unpop();

                // TRANSITIONS
                if (intakeInput) {
                    // LT Pressed
                    launchState = LaunchState.INTAKING;
                    intakeTimer.reset();
                } else if (revInput) {
                    // RT Pressed
                    launchState = LaunchState.SPIN_UP;
                    shootTime = timer.time();
                } else if (reverseIntakeInput) {
                    // LB Pressed
                    launchState = LaunchState.REVERSE_INTAKE;
                }
                break;

            case REVERSE_INTAKE:
                intake.setPower(-1);

                // TRANSITIONS
                if (!reverseIntakeInput) {
                    // LB released
                    launchState = LaunchState.IDLE;
                }
                break;

            case INTAKING:
                intake.setPower(1); // Run intake motor
                index.setMode(IndexSubsystem.Mode.INTAKE); // Set indexer to intake mode

                // TRANSITIONS
                if (!intakeInput) {
                    launchState = LaunchState.IDLE; // LT Released
                } else if (index.getDetectedBallColour() != IndexSubsystem.Ball.EMPTY && intakeTimer.time() > 0.5) {
                    // Color sensor detects ball -> Move to confirmation
                    launchState = LaunchState.CONFIRM_INTAKE;
                    intakeTimer.reset();
                }
                break;

            case CONFIRM_INTAKE:
                intake.setPower(1); // Same as Intaking
                index.setMode(IndexSubsystem.Mode.INTAKE);

                // TRANSITIONS
                if (index.getDetectedBallColour() == IndexSubsystem.Ball.EMPTY) {
                    // Ball lost (sensor stops detecting) -> go back to Intaking
                    launchState = LaunchState.INTAKING;
                } else if (intakeTimer.time() > 0.5) {
                    // 0.5 seconds passed with ball detected -> Set indexer storage slot
                    IndexSubsystem.Ball ballColor = index.getDetectedBallColour();
                    index.setBallInIntakeSlot(ballColor);

                    // Loop back to Intaking to get the next ball
                    launchState = LaunchState.INTAKING;
                }
                break;

            case SPIN_UP:
                intake.setPower(0);
                shooter.setSpeed(67);
                index.setMode(shootMode);
                if (Math.abs(shooter.getMotorSpeed() - 65) < 0.5 && timer.time() - shootTime > 1 && revInput && shootInput) {
                    launchState = LaunchState.POPPING;
                    shootTime = timer.time();
                } else if (!revInput) {
                    launchState = LaunchState.IDLE;
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
                    index.emptyCurrentSlot();
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

        //  FAILSAFE IN CASE MY FSM SUCK
        if (gamepad1.start && gamepad1.dpad_right) {
            index.setManualIndex(-1);
        }

        // Shooter arc control
        if (gamepad2.dpad_up || gamepad1.dpad_up) {
            shooterTilt = Math.min(shooterTilt + 0.01, 1);
        } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
            shooterTilt = Math.max(shooterTilt - 0.01, 0);
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
