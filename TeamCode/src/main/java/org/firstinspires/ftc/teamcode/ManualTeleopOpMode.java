package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LocalisationSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@TeleOp(name = "Manual Teleop")
public class ManualTeleopOpMode extends OpMode {
    private static final boolean DISABLE_COLOR_SENSOR = true;

    DriveSubsystem drive;
    LiftSubsystem lift;
    IntakeSubsystem intake;
    IndexSubsystem index;
    ShooterSubsystem shooter;
//    LEDSubsystem led;
    LocalisationSubsystem localisation;

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
    private double shooterTilt = 1;
    private double intakeTime = 0;
    private boolean intakenRecently = false;
    private int manualIndexerIndex = 0;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry, DISABLE_COLOR_SENSOR);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
//        led = new LEDSubsystem(hardwareMap, telemetry);
        localisation = new LocalisationSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        drive.driveRobotRelative(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);

        if (gamepad1.left_trigger > 0.5) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.dpad_left) {
            manualIndexerIndex = 0;
        } else if (gamepad1.dpad_up) {
            manualIndexerIndex = 1;
        } else if (gamepad1.dpad_right) {
            manualIndexerIndex = 2;
        }
        index.setManualIndex(manualIndexerIndex);

        if (gamepad1.left_trigger > 0.5) {
            index.setMode(IndexSubsystem.Mode.INTAKE_MANUAL);
            intake.setPower(1);
            shooter.setSpeed(0);
        } else if (gamepad1.right_trigger > 0.5) {
            index.setMode(IndexSubsystem.Mode.SHOOT_MANUAL);
            shooter.setSpeed(100);
            intake.setPower(0);
        } else {
            index.setMode(IndexSubsystem.Mode.INTAKE_MANUAL);
            shooter.setSpeed(0);
            intake.setPower(0);
        }

        if (gamepad1.a) {
            shooter.pop();
        } else {
            shooter.unpop();
        }

        // Shooter arc control
        if (gamepad2.dpad_up) {
            shooterTilt += 0.05;
        } else if (gamepad2.dpad_down) {
            shooterTilt -= 0.05;
        }
        shooter.setTilt(shooterTilt);

        // Raises or lowers lift
        if (gamepad1.left_bumper) {
            lift.setRightPower(-1);
        } else {
            lift.setRightPower(0);
        }

        if (gamepad1.right_bumper) {
            lift.setLeftPower(-1);
        } else {
            lift.setLeftPower(0);
        }

        drive.periodic();
        lift.periodic();
        intake.periodic();
        index.periodic();
        shooter.periodic();
//        led.periodic();
        telemetry.update();
    }
}
