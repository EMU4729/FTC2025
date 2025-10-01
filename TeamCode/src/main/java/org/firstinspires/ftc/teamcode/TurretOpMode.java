package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@TeleOp(name = "Turret (IAC Demo)")
public class TurretOpMode extends OpMode {
    IndexSubsystem index;
    ShooterSubsystem shooter;

    @Override
    public void init() {
        index = new IndexSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_right) {
            index.setMode(IndexSubsystem.Mode.CLOCKWISE);
        } else if (gamepad1.dpad_left) {
            index.setMode(IndexSubsystem.Mode.ANTICLOCKWISE);
        } else {
            index.setMode(IndexSubsystem.Mode.IDLE);
        }

        if (gamepad1.right_trigger > 0.5) {
            // NOTE: if the shooter is acting weird, the issue is probably in these numbers
            shooter.shoot(0.8, 0.5);
        } else {
            shooter.shoot(0, 0.5);
        }

        index.periodic();
        shooter.periodic();
        telemetry.update();
    }
}
