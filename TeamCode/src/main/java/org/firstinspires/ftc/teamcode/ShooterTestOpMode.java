package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter Test")
public class ShooterTestOpMode extends OpMode {
    private ShooterSubsystem shooter;
    private double tilt = 0;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        // shoot control
        if (gamepad1.right_trigger > 0.5) {
            shooter.setSpeed(100);
        } else {
            shooter.setSpeed(0);
        }

        // pop control
        if (gamepad1.a) {
            shooter.pop();
        } else {
            shooter.unpop();
        }

        // tilt control
        if (gamepad1.dpad_up) {
            tilt = Math.min(tilt + 0.01, 1);
        } else if (gamepad1.dpad_down) {
            tilt = Math.max(tilt - 0.01, 0);
        }
        shooter.setTilt(tilt);

        shooter.periodic();
    }
}
