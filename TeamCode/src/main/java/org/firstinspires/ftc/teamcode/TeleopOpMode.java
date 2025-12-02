package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.LiftRaise;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LocalisationSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@TeleOp(name = "TeleOp")
public class TeleopOpMode extends OpMode {
    private static final boolean DISABLE_COLOR_SENSOR = false;

    private DriveSubsystem drive;
    private LiftSubsystem lift;
    private IntakeSubsystem intake;
    private IndexSubsystem index;
    private ShooterSubsystem shooter;
    private LocalisationSubsystem localisation;
    private LiftRaise liftRaiseCommand;

    private enum ShootState {
        IDLE,
        PREPARING,
        SHOOTING
    }

    private enum IntakeState {
        IDLE,
        PREPARING,
        INTAKING,
    }

    private ShootState shootState = ShootState.IDLE;
    private IntakeState intakeState = IntakeState.IDLE;
    private IndexSubsystem.Mode shootMode = IndexSubsystem.Mode.SHOOT_ANY;
    private final ElapsedTime timer = new ElapsedTime();
    private double shootTime = 0;
    private double intakeTime = 0;
    private boolean intakenRecently = false;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry, DISABLE_COLOR_SENSOR);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
//        led = new LEDSubsystem(hardwareMap, telemetry);
        localisation = new LocalisationSubsystem(hardwareMap, telemetry);
        liftRaiseCommand = new LiftRaise(localisation, lift);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        drive.driveRobotRelative(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);

//         select colour conditional statements
        if (gamepad2.a) {
            shootMode = IndexSubsystem.Mode.SHOOT_GREEN_ONLY;
//            led.setSolidColor(Color.GREEN);
            telemetry.addData("Current Ball", "GREEN");
        } else if (gamepad2.x) {
            shootMode = IndexSubsystem.Mode.SHOOT_PURPLE_ONLY;
//            led.setSolidColor(Color.rgb(128, 0, 128)); // purple
            telemetry.addData("Current Ball", "PURPLE");
        } else if (gamepad2.b) {
            shootMode = IndexSubsystem.Mode.SHOOT_ANY;
//            led.setMode(LEDSubsystem.Mode.RAINBOW);
            telemetry.addData("Current Ball", "ANY");
        }

        // intake FSM
        if (gamepad2.left_trigger > 0.5 && intakeState == IntakeState.IDLE) {
            intakeState = IntakeState.PREPARING;
        } else if (gamepad2.left_trigger <= 0.5) {
            intakeState = IntakeState.IDLE;
        }

        switch (intakeState) {
            case IDLE:
                // Reset everything
                intake.setPower(0);
                break;

            case PREPARING:
                // primes the indexer
                index.setMode(IndexSubsystem.Mode.INTAKE);
                intake.setPower(0);
                if (index.atTarget()) {
                    intakeState = IntakeState.INTAKING;
                }
                break;

            case INTAKING:
                // Pulls the ball in
                intake.setPower(1);
                if (index.ballIntaken()) {
                    intakeState = IntakeState.PREPARING;
                }
                break;
        }

        // Shooter arc control
        if (gamepad2.dpad_up) {
            shooter.setTilt(shooter.getTilt() + 0.05);
        } else if (gamepad2.dpad_down) {
            shooter.setTilt(shooter.getTilt() - 0.05);
        }

        // Shoot state FSM
        if (gamepad2.right_trigger > 0.5 && shootState == ShootState.IDLE) {
            shootState = ShootState.PREPARING;
        } else if (gamepad2.right_trigger <= 0.5) {
            shootState = ShootState.IDLE;
        }

        if (gamepad1.right_trigger > 0.5) {
            shooter.setSpeed(gamepad2.y ? 0.5 : 2);
        } else {
            shooter.setSpeed(0);
        }

        switch (shootState) {
            case IDLE:
                if (gamepad1.right_trigger <= 0.5) {
                    shooter.setSpeed(0);
                }
                shooter.unpop();
                break;

            case PREPARING:
                index.setMode(shootMode);
                shooter.setSpeed(gamepad2.y ? 0.5 : 2);
                if (index.atTarget() && shooter.atDesiredSpeed() && gamepad2.right_trigger > 0.5) {
                    shootState = ShootState.SHOOTING;
                    shootTime = timer.time();
                }
                break;

            case SHOOTING:
                shooter.pop();
                shootMode = IndexSubsystem.Mode.SHOOT_ANY;

                if (timer.time() - shootTime > 0.5) {
                    shooter.unpop();
                    index.emptyCurrentSlot();
                    shootState = ShootState.PREPARING;
                }
                break;
        }

        // Raises or lowers lift
        if (gamepad2.right_bumper) {
            liftRaiseCommand.execute();
        } else if (gamepad2.rightBumperWasReleased()) {
            liftRaiseCommand.end();
        }

        // alternative ball detection implementation based on detecting current spike in the intake
        // motor (used if the colour sensor does not work)
        if (DISABLE_COLOR_SENSOR) {
            if (intake.ballIntaken()) {
                intakeTime = timer.time();
                intakenRecently = true;
            }
            if (intakenRecently && timer.time() - intakeTime > 0.3) {
                index.unsafe_manuallyMarkIntake();
                intakenRecently = false;
            }
        }

        drive.periodic();
        lift.periodic();
        intake.periodic();
        index.periodic();
        shooter.periodic();
        localisation.periodic();
//        led.periodic();
        telemetry.update();
    }
}
