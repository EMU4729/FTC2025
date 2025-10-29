package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    double currentArc = 0.8;

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
    public void loop() {
        drive.driveRobotRelative(-gamepad2.left_stick_y, gamepad2.left_stick_x, -gamepad2.right_stick_x);

        if (gamepad1.dpad_up) {
            lift.setPower(0.5);
        } else if (gamepad1.dpad_down) {
            lift.setPower(-0.5);
        } else {
            lift.setPower(0);
        }

        if (gamepad1.left_trigger > 0.5) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.dpad_right) {
            index.setMode(IndexSubsystem.Mode.CLOCKWISE);
        } else if (gamepad1.dpad_left) {
            index.setMode(IndexSubsystem.Mode.ANTICLOCKWISE);
        } else {
            index.setMode(IndexSubsystem.Mode.IDLE);
        }

        if (gamepad1.right_bumper) {
            currentArc = Math.min(currentArc + 0.05, 1);
        } else if (gamepad1.left_bumper) {
            currentArc = Math.max(currentArc - 0.05, 0);
        }

        if (gamepad1.right_trigger > 0.5) {
            // NOTE: if the shooter is acting weird, the issue is probably in these numbers
            shooter.pop();
            shooter.setSpeed(1);
        } else {
            shooter.setSpeed(0);
            shooter.unpop();
        }
        shooter.setYArc(currentArc);

        drive.periodic();
        lift.periodic();
        intake.periodic();
        index.periodic();
        shooter.periodic();
        led.periodic();
        telemetry.update();
    }
}
