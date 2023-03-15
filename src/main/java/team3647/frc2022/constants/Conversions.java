// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Conversions {

    public static double metersToSteps(double meters) {
        return meters / (Units.inchesToMeters(Constants.WHEEL_CIRCUMFERENCE_INCHES) * Constants.GEARING)
                * Constants.ENCODER_COUNT;
    }

    public static double stepsToMeters(double steps) {
        return steps / Constants.ENCODER_COUNT * Constants.GEARING
                * Units.inchesToMeters(Constants.WHEEL_CIRCUMFERENCE_INCHES);
    }

    public static double stepsPerDecisecondToMetersPerSecond(double steps) {
        return stepsToMeters(steps) * 10;
    }

}