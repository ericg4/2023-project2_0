// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

/** Add your docs here. */
public class LEDConstants {
    // LED Counts
    public static final int candleLEDS = 8;
    public static final int stripLEDS = 144;
    public static final int LEDCOUNT = candleLEDS + stripLEDS;

    // Animations List
    public static final Animation RAINBOW = new RainbowAnimation(1, 0.1, LEDCOUNT);
    public static final Animation GREEN_STROBE = new StrobeAnimation(0, 255, 0, 0, 50.0 / 256.0, LEDCOUNT);
    public static final Animation LARSON = new LarsonAnimation(0, 255, 46, 0, 1, LEDCOUNT, BounceMode.Front, 3);
    public static final Animation COLOR_FLOW = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LEDCOUNT, Direction.Forward);
    public static final Animation SOLID_YELLOW =
            new StrobeAnimation(
                    256, 256, 0, 128, 1, LEDCOUNT);
    public static final Animation SOLID_PURPLE =
            new StrobeAnimation(
                    162, 25, 255, 128, 1, LEDCOUNT);

}
