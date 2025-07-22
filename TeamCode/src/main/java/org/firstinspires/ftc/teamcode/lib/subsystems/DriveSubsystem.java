package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveSubsystem extends SubsystemBase {
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor rearLeft;
    private final DcMotor rearRight;
    private final Telemetry telemetry;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        frontLeft = hardwareMap.get(DcMotor.class, "leftFront");
        frontRight = hardwareMap.get(DcMotor.class, "rightFront");
        rearLeft = hardwareMap.get(DcMotor.class, "leftRear");
        rearRight = hardwareMap.get(DcMotor.class, "rightRear");
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Drives the robot with robot-relative speeds.
     * @param x Robot-relative x speed in the range [-1, 1].
     * @param y Robot-relative y speed in the range [-1, 1].
     * @param r Rotational speed in the range [-1, 1].
     */
    public void driveRobotRelative(double x, double y, double r) {
        double frontLeftPower = x - y - r;
        double frontRightPower = x + y + r;
        double backLeftPower = x + y - r;
        double backRightPower = x - y + r;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        rearLeft.setPower(backLeftPower);
        rearRight.setPower(backRightPower);
    }

    /**
     * Drives the robot with field-relative speeds.
     * @param x Field-relative x speed in the range [-1, 1].
     * @param y Field-relative y speed in the range [-1, 1].
     * @param r Rotational speed in the range [-1, 1].
     * @param yawRadians The current yaw of the robot, in radians.
     */
    public void driveFieldRelative(double x, double y, double r, double yawRadians) {
        // https://en.wikipedia.org/wiki/Rotation_matrix
        double robotRelativeY = x * Math.cos(yawRadians) + y * Math.sin(yawRadians);
        double robotRelativeX = -x * Math.sin(yawRadians) + y * Math.cos(yawRadians);
        driveRobotRelative(robotRelativeX, robotRelativeY, r);
    }

    private void updateTelemetry() {
        telemetry.addData("FL Drive Power", frontLeft.getPower());
        telemetry.addData("FR Drive Power", frontRight.getPower());
        telemetry.addData("BL Drive Power", rearLeft.getPower());
        telemetry.addData("BR Drive Power", rearRight.getPower());
    }
}
