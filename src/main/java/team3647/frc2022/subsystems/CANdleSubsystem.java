// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.frc2022.constants.Constants;
import team3647.frc2022.constants.LEDConstants;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(Constants.CANdleID);

    private static final CANdleSubsystem candleSub = new CANdleSubsystem();

    public static CANdleSubsystem getInstance() {
        return candleSub;
    }

    private final int LedCount = LEDConstants.LEDCOUNT;

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        NONE
    }
    public static enum LEDModes {
        // LED Modes
        CUBE("Cube"), 
        CONE("Cone"), 
        RAINBOW("Rainbow"), 
        SCORING("Scoring"),
        MANUALTEST("ManualTest"),
        IDLE("Idle"),
        NONE("None");

        private String ledMode;

        // Constructor
        private LEDModes(String ledMode) {
                this.ledMode = ledMode;
        }

        private String returnLEDState() {
            return ledMode;
        }  

    }

    private LEDModes currentLEDMode = LEDModes.CONE;

    public CANdleSubsystem() {
        // this.joystick = joy;
        // changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.v5Enabled = false;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);

        SmartDashboard.putNumber("R", 255);
        SmartDashboard.putNumber("G", 0);
        SmartDashboard.putNumber("B", 0);
        SmartDashboard.putNumber("start", 0);
        SmartDashboard.putNumber("end", 8);

        SmartDashboard.putString("LED State", currentLEDMode.returnLEDState());

        resetColor(0, LedCount);

    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() {
        return m_candle.getBusVoltage();
    }

    public double get5V() {
        return m_candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return m_candle.getCurrent();
    }

    public double getTemperature() {
        return m_candle.getTemperature();
    }

    public void configBrightness(double percent) {
        m_candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        m_candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        m_candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        m_candle.configStatusLedState(offWhenActive, 0);
    }

    public void changeAnimation(AnimationTypes toChange) {
        switch (toChange) {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, LedCount);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(0, 255, 0, 0, 50.0 / 256.0, LedCount);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
                break;
            case NONE:
                m_toAnimate = null;
                break;
        }
        m_candle.animate(m_toAnimate);
    }

    public void setRangeToColor(int[] RGB, int start, int count) {
        int r = RGB[0];
        int g = RGB[1];
        int b = RGB[2];
        m_candle.setLEDs(r, g, b, 255, start, count);
    }

    public void setToColor(int[] RGB, int pos) {
        int r = RGB[0];
        int g = RGB[1];
        int b = RGB[2];
        m_candle.setLEDs(r, g, b, 255, pos, 1);
    }

    public void resetColor(int startPos, int count) {
        m_candle.setLEDs(0, 0, 0, 255, startPos, count);
    }

    public void speedColorAdjuster(double speed) {
        double maxSpeed = 10;

        int numLEDs = (int) (speed / maxSpeed * LedCount);

        for (double i = 0; i < numLEDs; i++) {
            int[] ledColor = {(255 - (int) (i / LedCount * 255)), 0, ((int) (i / LedCount * 255))};
            setToColor(ledColor, (int) i);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("LED State", currentLEDMode.returnLEDState());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    // Set Animation Mode Functions
}
