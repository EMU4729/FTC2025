package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class LiftSubsystem extends SubsystemBase {
    private static final double LIFT_DISTANCE_PER_TICK = 1 / 537.7;
    private static final double LIFT_PID_P = 1;
    private static final double SYNC_P_GAIN = 0.005;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;
private IMU imu;
    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftL");
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftR");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(imu.getClass(), "imu_lift");
        this.telemetry = telemetry;

        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        new Orientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.DEGREES,
                                90,
                                0,
                                -45,
                                0  // acquisitionTime, not used
                        )
                )
        );

// Initialize IMU using Parameters

        imu.initialize(myIMUparameters);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Check your physical wiring to confirm which one needs reverse
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders so 0 is the bottom
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void periodic() {
        telemetry.addData("Lift Left Conn", leftMotor.getConnectionInfo());
        telemetry.addData("Lift Right Conn", leftMotor.getConnectionInfo());


        // Create Orientation variable
        Orientation myRobotOrientation;

// Get Robot Orientation
        myRobotOrientation = imu.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.DEGREES
        );

// Then read or display the desired values (Java type float):
        float X_axis = myRobotOrientation.firstAngle;
        float Y_axis = myRobotOrientation.secondAngle;
        float Z_axis = myRobotOrientation.thirdAngle;

        AngularVelocity myRobotAngularVelocity;

// Read Angular Velocities
        myRobotAngularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

// Then read or display these values (Java type float)
// from the object you just created:
        float zRotationRate = myRobotAngularVelocity.zRotationRate;
        float xRotationRate = myRobotAngularVelocity.xRotationRate;
        float yRotationRate = myRobotAngularVelocity.yRotationRate;



    }
    private double clamp(double val) {
        return Math.max(-1.0, Math.min(1.0, val));
    }

    /**
     * Directly sets the power of the lift motors.
     *
     * @param targetPower The power in the range [-1, 1]
     */
    public void setPower(double targetPower) {
        // Get positions
        int leftPos = leftMotor.getCurrentPosition();
        int rightPos = rightMotor.getCurrentPosition();

        // Calculate difference (Error)
        // If left is at 1000 and right is at 800, error is 200.
        int error = leftPos - rightPos;

        // Calculate correction
        // If left is higher (positive error), we subtract power from left (or add to right).
        double correction = error * SYNC_P_GAIN;

        // Apply power with correction
        // We restrain the values to ensure they don't exceed +/- 1.0
        double leftPower = targetPower - correction;
        double rightPower = targetPower + correction;

        // Safety clamp
        leftMotor.setPower(clamp(leftPower));
        rightMotor.setPower(clamp(rightPower));
    }


    public void setLeftPower(double power) {
        leftMotor.setPower(power);
    }

    public void setRightPower(double power) {
        rightMotor.setPower(power);
    }
}


