package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter Test")
public class ShooterTestOpMode extends OpMode {
    private ShooterSubsystem shooter;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > 0.5) {
            shooter.setSpeed(100);
        } else {
            shooter.setSpeed(0);
        }

        if (gamepad1.a) {
            shooter.pop();
        } else {
            shooter.unpop();
        }

        shooter.periodic();
    }
}
