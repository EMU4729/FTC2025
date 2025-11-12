package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@TeleOp(name = "TeleOp")
public class TeleopOpMode extends OpMode {
    DriveSubsystem drive;
    LiftSubsystem lift;
    IntakeSubsystem intake;
    IndexSubsystem index;
    ShooterSubsystem shooter;
    LEDSubsystem led;

    private enum ShootState {
        IDLE,
        PREPARING,
        SHOOTING
    }

    // put the comment for method again (I forgot)
    private enum IntakeState {
        IDLE,
        READY,
        PULL,
    }

    private ShootState shootState = ShootState.IDLE;
    private IntakeState intakeState = IntakeState.IDLE;
    private IndexSubsystem.Mode shootMode = IndexSubsystem.Mode.SHOOT_ANY;
    private ElapsedTime timer;
    private double shootTime = 0;
    private double currentArc = 0;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        led = new LEDSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        timer.reset();
    }

    /**
     * Get the speed the shooter should be at to fire based off the speed setting modes.
     *
     * @return A double of the motor speed that the motor should be at to fire.
     */
    private double getCorrectShooterSpeed(boolean slow) {
        if (!slow) {
            return 2;
        } else {
            return 0.5;
        }
    }

    @Override
    public void loop() {
        drive.driveRobotRelative(-gamepad2.left_stick_y, gamepad2.left_stick_x, -gamepad2.right_stick_x);

        // select colour conditional statements
        if (gamepad2.a) {
            shootMode = IndexSubsystem.Mode.SHOOT_GREEN_ONLY;
            telemetry.addData("Current Ball", "GREEN");
        } else if (gamepad2.x) {
            shootMode = IndexSubsystem.Mode.SHOOT_PURPLE_ONLY;
            telemetry.addData("Current Ball", "PURPLE");
        } else if (gamepad2.y) {
            shootMode = IndexSubsystem.Mode.SHOOT_ANY;
            telemetry.addData("Current Ball", "ANY");
        }

        // intake FSM
        if (gamepad2.left_trigger > 0.5 && intakeState == IntakeState.IDLE) {
            intakeState = IntakeState.READY;
        } else if (gamepad2.left_trigger <= 0.5) {
            intakeState = IntakeState.IDLE;
        }

        switch (intakeState) {
            case IDLE:
                // Reset everything
                intake.setPower(0);
                break;

            case READY:
                // primes the indexer
                index.setMode(IndexSubsystem.Mode.INTAKE);
                intake.setPower(0);
                if (index.atTarget()) {
                    intakeState = IntakeState.PULL;
                }
                break;

            case PULL:
                // Pulls the ball in
                intake.setPower(1);
                if (index.ballIntaken()) {
                    intakeState = IntakeState.READY;
                }
                break;
        }

        // Shooter arc control
        if (gamepad2.dpad_up) {
            currentArc += 5;
        } else if (gamepad2.dpad_down) {
            currentArc -= 5;
        }
        shooter.setArc(currentArc);

        // Shoot state FSM
        if (gamepad2.right_trigger > 0.5 && shootState == ShootState.IDLE) {
            shootState = ShootState.PREPARING;
        } else if (gamepad2.right_trigger <= 0.5) {
            shootState = ShootState.IDLE;
        }

        switch (shootState) {
            case IDLE:
                shooter.setSpeed(0);
                shooter.unpop();
                break;

            case PREPARING:
                index.setMode(shootMode);
                shooter.setSpeed(getCorrectShooterSpeed(gamepad2.y));
                if (index.atTarget() && shooter.atDesiredSpeed()) {
                    shootState = ShootState.SHOOTING;
                    shootTime = timer.time();
                }
                break;

            case SHOOTING:
                shooter.pop();
                index.emptyCurrentSlot();

                shootMode = IndexSubsystem.Mode.SHOOT_ANY;

                if (timer.time() - shootTime > 0.5) {
                    shooter.unpop();
                    shootState = ShootState.PREPARING;
                }
                break;
        }

        drive.periodic();
        lift.periodic();
        intake.periodic();
        index.periodic();
        shooter.periodic();
        led.periodic();
        telemetry.update();
    }
}
