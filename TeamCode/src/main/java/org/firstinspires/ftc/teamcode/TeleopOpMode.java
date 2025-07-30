package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Robot;

@TeleOp(name = "TeleOp")
public class TeleopOpMode extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void start() {
        robot.teleopInit();
    }

    @Override
    public void loop() {
        robot.teleopPeriodic();
        robot.periodic();
        telemetry.update();
    }
}
