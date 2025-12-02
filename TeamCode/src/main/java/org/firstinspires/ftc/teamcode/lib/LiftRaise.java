package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LocalisationSubsystem;

public class LiftRaise {
    private static final double SYNC_P = 1;
    private final LocalisationSubsystem localisation;
    private final LiftSubsystem lift;

    public LiftRaise(LocalisationSubsystem localisation, LiftSubsystem lift) {
        this.localisation = localisation;
        this.lift = lift;
    }

    public void execute() {
        // Get positions
        double roll = localisation.getIMUAngles().getRoll(AngleUnit.RADIANS);

        double correction = roll * SYNC_P;

        // Apply power with correction We restrain the values to ensure they don't exceed +/- 1.0
        double leftPower = 1 - correction;
        double rightPower = 1 + correction;

        leftPower = Math.max(-1, Math.min(1, leftPower));
        rightPower = Math.max(-1, Math.min(1, rightPower));

        lift.setLeftPower(leftPower);
        lift.setRightPower(rightPower);
    }

    public void end() {
        lift.setLeftPower(0);
        lift.setLeftPower(0);
    }
}
