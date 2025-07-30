package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Robot;

@Autonomous(name = "Left Mobility")
public class LeftMobilityAuto extends OpMode {
    private Robot robot;
    private ElapsedTime elapsedTime;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        if (elapsedTime.time() < 3) {
            robot.drive.driveRobotRelative(0, 0.5, 0);
        } else {
            robot.drive.stop();
        }

        robot.periodic();
        telemetry.update();
    }
}
