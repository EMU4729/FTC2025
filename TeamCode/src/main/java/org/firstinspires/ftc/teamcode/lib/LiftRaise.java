package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OTOSLocalisationSubsystem;

public class LiftRaise {
    private static final double SYNC_P = 1;
    private final OTOSLocalisationSubsystem localisation;
    private final LiftSubsystem lift;
    private final ElapsedTime timer = new ElapsedTime();

    public LiftRaise(OTOSLocalisationSubsystem localisation, LiftSubsystem lift) {
        this.localisation = localisation;
        this.lift = lift;
    }

    public void start() {
        timer.reset();
    }

    public void execute() {
        // unlock lift for the first 0.5s
        if (timer.time() < 0.5) {
            lift.unlock();
            return;
        }

        // Get positions
        double roll = localisation.getIMUAngles().getRoll(AngleUnit.RADIANS);

        double correction = roll * SYNC_P;

        // Apply power with correction We restrain the values to ensure they don't exceed +/- 1.0
        double leftPower = correction - 1;
        double rightPower = -correction - 1;

        leftPower = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower * 1.3));

        lift.setLeftPower(leftPower);
        lift.setRightPower(rightPower);
    }

    public void end() {
        lift.setLeftPower(0);
        lift.setRightPower(0);
    }
}
