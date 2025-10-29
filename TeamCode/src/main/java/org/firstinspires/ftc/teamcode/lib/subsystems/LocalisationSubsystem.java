package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class LocalisationSubsystem {
    private static final boolean USE_WEBCAM = true;
    private final SparkFunOTOS OTOSSensor;
    private final Telemetry telemetry;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    HardwareMap hardwareMap;


    // CAMERA Position / YPR
    private Position cameraPosition = new Position(DistanceUnit.METER,
            0, 0, 0, 0);

    // pitch was init'd as -90 in demo, but set to 0 here for now (-90 made no sense)
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);


    // ROBOT X, Y and HDG
    public double robotX;
    public double robotY;
    public double robotH;



    public LocalisationSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        configureAprilTag();

        OTOSSensor = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOTOS();

        telemetry.addLine("Localisation Subsystem Configured");
        telemetry.update();
    }

    public void periodic() {
        List<AprilTagDetection> freshDetections = aprilTag.getFreshDetections();

        if (!freshDetections.isEmpty()) {
            // we cannot assume that there is only one detection, but we do know the last one is the latest. therefore, by going through all of them, we can know we are up to date.
            for (AprilTagDetection detection : freshDetections) {
                // as shown by demo code - no "Obelisk" tags can be used for positioning
                if (!detection.metadata.name.contains("Obelisk")) {
                    robotX = detection.robotPose.getPosition().x;
                    robotY = detection.robotPose.getPosition().y;
                    robotH = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                }
            }

            // update the OTOS's position data
            updateOTOS();
            telemetry.addData("AprilUpdate", true);
        } else {
            // navigate off OTOS
            positionFromOTOS();
            telemetry.addData("AprilUpdate", false);
        }

        telemetry.addData("RobotX", robotX);
        telemetry.addData("RobotY", robotY);
        telemetry.addData("RobotHDG", robotH);
        telemetry.update();
    }

    private void updateOTOS() {
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(
                robotX,
                robotY,
                robotH
        );
        OTOSSensor.setPosition(currentPosition);
    }

    private void positionFromOTOS() {
        SparkFunOTOS.Pose2D currentPosition = OTOSSensor.getPosition();
        robotX = currentPosition.x;
        robotY = currentPosition.y;
        robotH = currentPosition.h;
    }

    private void configureOTOS() {
        // UNIT CALIBRATION
        OTOSSensor.setLinearUnit(DistanceUnit.METER);
        OTOSSensor.setAngularUnit(AngleUnit.DEGREES);

        // OFFSET ON ROBOT -- todo: find exact values, need robot physically
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0); // h is for heading, probably
        OTOSSensor.setOffset(offset);

        // SCALAR CALIBRATION -- todo: find exact values, need robot physically
        OTOSSensor.setLinearScalar(1.0);
        OTOSSensor.setAngularScalar(1.0);

        // CALIBRATE IMU
        OTOSSensor.calibrateImu();
        OTOSSensor.resetTracking();

        // starting position, will be updated by april tag positioning.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        OTOSSensor.setPosition(currentPosition);

        telemetry.addLine("OTOS Configured");
        telemetry.update();
    }


    private void configureAprilTag() {
        // april tag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // vision portal
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // NOTE: USE_WEBCAM is set at top of class. idk what to set it, so it is currently true as the example code used that.
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.FRONT); // used to be BACK in e.g. code
        }

        // enable the processor
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
        telemetry.addLine("April Tag Process Configured");
        telemetry.update();
    }
}
