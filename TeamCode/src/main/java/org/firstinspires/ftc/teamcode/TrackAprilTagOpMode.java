package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;

@TeleOp(name = "Track AprilTag")
public class TrackAprilTagOpMode extends OpMode {
    DriveSubsystem drive;

    private AprilTagProcessor aprilTag;
    private double turnVal;

    @Override
    public void init() {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam"), aprilTag);
    }

    @Override
    public void start() {
        turnVal = 0;
    }

    @Override
    public void loop() {
        drive.periodic();

        getAprilTagCenterX().ifPresent((val) -> turnVal = val);

        double error = turnVal - 0.5;
        drive.driveRobotRelative(0, 0, error);

        telemetry.update();
    }

    /**
     * Function gets the x coordinates of an april tag
     * @return returns the centre x coordinates of april tag if it exists
     */
    private Optional<Double> getAprilTagCenterX() {
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
    }
}