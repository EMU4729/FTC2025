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

@TeleOp(name = "Simple Teleop")
public class SimpleTeleopOpMode extends OpMode {
    private static final boolean DISABLE_COLOR_SENSOR = true;

    DriveSubsystem drive;
    LiftSubsystem lift;
    IntakeSubsystem intake;
    IndexSubsystem index;
    ShooterSubsystem shooter;
//    LEDSubsystem led;
    LocalisationSubsystem localisation;
    ElapsedTime triggerCooldown = new ElapsedTime();
    private double shootTime = 0;
    private double shooterTilt = 1;


    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCHING,
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
        localisation = new LocalisationSubsystem(hardwareMap, telemetry);
        launchState = LaunchState.IDLE;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void loop() {
        drive.driveRobotRelative(-gamepad2.left_stick_y, -gamepad2.left_stick_x, -gamepad2.right_stick_x);


        // for human player intake
        if (gamepad2.left_trigger > 0.5) {
            index.setManualIndex(manualIndexerIndex);
            index.setMode(IndexSubsystem.Mode.SHOOT_MANUAL); // switch to indexer slot
            shooter.setSpeed(-1);
        } else if (gamepad2.right_trigger <= 0.5){
            shooter.setSpeed(0);
        }



        //Rotate indexer
        if (gamepad1.dpad_left || gamepad2.dpad_left){
            if (manualIndexerIndex == 0){
                manualIndexerIndex = 2;
            } else {
                manualIndexerIndex--;
            }
            index.setManualIndex(manualIndexerIndex);
            index.setMode(IndexSubsystem.Mode.SHOOT_MANUAL); // switch to indexer slot
        }

        if (gamepad1.dpad_right || gamepad2.dpad_right){
            if (manualIndexerIndex == 2){
                manualIndexerIndex = 0;
            } else {
                manualIndexerIndex++;
            }
            index.setManualIndex(manualIndexerIndex);
            index.setMode(IndexSubsystem.Mode.SHOOT_MANUAL); // switch to indexer slot
        }


        // Pre-assigned index rotations
//        if (gamepad1.dpad_left) {
//            manualIndexerIndex = 0;
//        } else if (gamepad1.dpad_up) {
//            manualIndexerIndex = 1;
//        } else if (gamepad1.dpad_right) {
//            manualIndexerIndex = 2;
//        }
//        index.setManualIndex(manualIndexerIndex);

//        if (gamepad1.left_trigger > 0.5) {
//            index.setMode(IndexSubsystem.Mode.INTAKE_MANUAL);
//            intake.setPower(1);
//            shooter.setSpeed(0);
//        } else if (gamepad1.right_trigger > 0.5) {
//            index.setMode(IndexSubsystem.Mode.SHOOT_MANUAL);
//            shooter.setSpeed(100);
//            intake.setPower(0);
//        } else {
//            index.setMode(IndexSubsystem.Mode.INTAKE_MANUAL);
//            shooter.setSpeed(0);
//            intake.setPower(0);
//        }



        if (gamepad2.right_trigger > 0.5) {
            triggerCooldown.reset();
                launchState = LaunchState.SPIN_UP;
        } else if (gamepad2.right_trigger <= 0.5) {
                launchState = LaunchState.IDLE;
        }

            switch (launchState) {
                case IDLE:
                    shooter.unpop();
                    break;

                case SPIN_UP:
                    shooter.setSpeed(1);
                    if (shooter.atDesiredSpeed() && gamepad2.right_trigger > 0.5) {
                        launchState = LaunchState.LAUNCHING;
                        shootTime = timer.time();
                    }
                    break;

                case LAUNCHING:
                    shooter.pop();
                    if (manualIndexerIndex == 2){
                        manualIndexerIndex = 0;
                    } else {
                        manualIndexerIndex++;
                    }
                    index.setManualIndex(manualIndexerIndex);
                    index.setMode(IndexSubsystem.Mode.SHOOT_MANUAL); // switch to indexer slot
                    if (timer.time() - shootTime > 0.5) {
                        shooter.unpop();
                        index.emptyCurrentSlot();
                        launchState = LaunchState.SPIN_UP;
                    }
                    break;

            }
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

        // Shooter arc control
        if (gamepad2.dpad_up || gamepad1.dpad_up) {
            shooterTilt += 0.05;
        } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
            shooterTilt -= 0.05;
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
