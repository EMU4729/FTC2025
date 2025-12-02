package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class LiftSubsystem extends SubsystemBase {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Servo leftLock;
    private final Servo rightLock;
    private final Telemetry telemetry;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftL");
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftR");
        leftLock = hardwareMap.get(Servo.class, "liftLLock");
        rightLock = hardwareMap.get(Servo.class, "liftRLock");
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLock.setPosition(1);
        rightLock.setPosition(1);

        this.telemetry = telemetry;
    }

    /**
     * Directly sets the power of the lift motors.
     *
     * @param power The power in the range [-1, 1]
     */
    public void setPower(double power) {
        setLeftPower(power);
        setRightPower(power);
    }


    /**
     * Sets the power of the left lift motor.
     *
     * @param power The power in the range [-1, 1]
     */
    public void setLeftPower(double power) {
        leftLock.setPosition(0);
        leftMotor.setPower(power);
    }

    /**
     * Sets the power of the right lift motor.
     *
     * @param power The power in the range [-1, 1]
     */
    public void setRightPower(double power) {
        leftLock.setPosition(0);
        rightMotor.setPower(power);
    }
}


