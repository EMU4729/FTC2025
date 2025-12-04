package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.PIDController;

public class LiftSubsystem extends SubsystemBase {
    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Servo leftLock;
    private final Servo rightLock;
    private final Telemetry telemetry;
    private final IMU imu;
    private final PIDController PIDcontrol;
    public boolean lift_activate = false;

    public static double kP = 0.03;  // Start small! Power per degree of error
    public static double kD = 0.005; // Power per degree/sec of velocity
    public static double kI = 0.0;   // Keep 0 for now
    public static double kStatic = 0.15; // The "Deadband" / Minimum power to move

    // --- State Variables ---
    private double targetAngle = 0.0; // 0 is usually level
    private double errorSum = 0.0;
    private long lastTimeNano = 0;


    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftL");
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftR");
        leftLock = hardwareMap.get(Servo.class, "liftLLock");
        rightLock = hardwareMap.get(Servo.class, "liftRLock");
        imu = hardwareMap.get(IMU.class, "IMU");
        PIDcontrol = hardwareMap.get(PIDController.class, "pidcontrol");

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLock.setDirection(Servo.Direction.REVERSE);

        setLockPosition(1);

        this.telemetry = telemetry;
        lastTimeNano = System.nanoTime();
    }



    public void updateBalance() {
        // 1. Calculate DT (Time delta in Seconds)
        long currentTimeNano = System.nanoTime();
        double dt = (currentTimeNano - lastTimeNano) / 1.0E9; // Convert Nano to Seconds
        lastTimeNano = currentTimeNano;

        // Safety check for weird dt (first loop)
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        // 2. Get Inputs (Position AND Velocity)
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        double currentAngle = orientation.getRoll(AngleUnit.DEGREES);
        // CRITICAL: Get the rotation rate specifically for the Roll axis (X or Y depending on mounting)
        // Usually Roll corresponds to X-axis rotation on the Control Hub, but check your mounting!
        double gyroRate = velocity.xRotationRate;

        // 3. Calculate Error
        double error = targetAngle - currentAngle;

        // 4. PID Math
        double pTerm = kP * error;
        double dTerm = kD * (-gyroRate); // D on Measurement (Resists motion)

        // Integral (with anti-windup clamping)
        errorSum += error * dt;
        if (kI != 0) {
            errorSum = Range.clip(errorSum, -10, 10);
        }
        double iTerm = kI * errorSum;

        // 5. Total Output
        double rawOutput = pTerm + iTerm + dTerm;

        // 6. Motor Mapping (Deadband / Feedforward)
        double motorPower = applyMotorDeadband(rawOutput);

        // 7. Safety Clamp
        motorPower = Range.clip(motorPower, -1.0, 1.0);


        // For pure balancing (no driver input):
        setLeftPower(motorPower);
        setRightPower(-motorPower); // Inverse direction to tilt the robot

        // Telemetry for Tuning
        telemetry.addData("Lift/Roll", currentAngle);
        telemetry.addData("Lift/Error", error);
        telemetry.addData("Lift/Power", motorPower);
    }
    private double applyMotorDeadband(double input) {
        // If input is tiny (noise), do nothing
        if (Math.abs(input) < 0.02) return 0.0;

        // Add the static friction (kStatic) to overcome the gearbox
        if (input > 0) return input + kStatic;
        else return input - kStatic;
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
     * Sets the position of the locking servos.
     *
     * @param position The position to set the servos to.
     */
    public void setLockPosition(double position) {
        leftLock.setPosition(position);
        rightLock.setPosition(position);
    }

    /**
     * Sets the power of the left lift motor.
     *
     * @param power The power in the range [-1, 1]
     */
    public void setLeftPower(double power) {
        leftMotor.setPower(power);
    }

    /**
     * Sets the power of the right lift motor.
     *
     * @param power The power in the range [-1, 1]
     */
    public void setRightPower(double power) {
        rightMotor.setPower(power);
    }

    public void loop(){
        getIMUReading();

    }

}


