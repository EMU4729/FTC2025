package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.Robot;

@TeleOp(name = "Test TeleOp")
public class TestOpMode extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad2);
    }

    @Override
    public void start() {
        robot.teleopInit();
    }

    @Override
    public void loop() {
        robot.teleopTestPeriodic();
        robot.periodic();
    }
}
