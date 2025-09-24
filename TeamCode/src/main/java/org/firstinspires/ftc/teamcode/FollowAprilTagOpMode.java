package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;

@TeleOp(name = "FollowAprilTag")
public class FollowAprilTagOpMode extends OpMode {
    Robot robot;

    private AprilTagProcessor aprilTag;
    private double turnVal;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    @Override
    public void start() {
        turnVal = 0;
    }

    @Override
    public void loop() {
        robot.periodic();

        GetAprilTagCenterX().ifPresent((val) -> turnVal = val);

        double error = turnVal - 0.5;
        robot.drive.driveRobotRelative(0, 0, error);


        // rotate code here - todo

        telemetry.update();
    }

    private Optional<Double> GetAprilTagCenterX() {
        // currently just tells how many april tags are seen
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        if (currentDetections.isEmpty()) {
            telemetry.addData("April Tag Detection Error", "No April Tags Seen");
            return Optional.empty();
        } else {
            telemetry.addData("April Tag Detection Error", "None");
            AprilTagDetection detected_tag = currentDetections.get(0);
            return Optional.of(detected_tag.center.x);
        }
        //AprilTagDetection detection = currentDetections.get(0);



    }
}