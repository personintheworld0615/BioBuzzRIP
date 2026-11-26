package org.firstinspires.ftc.teamcode.robot.subsystem;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

public class ColorSensor{



    private final NormalizedColorSensor colorSensor;
    private final float[] hsvValues = new float[3];

    // Tunable thresholds
    private static final float MIN_VALUE = 0.15f;
    private static final float MIN_SATURATION = 0.45f;
    private static final float WHITE_VALUE = 0.60f;


    public ColorSensor(HardwareMap hardwareMap) {
        this(hardwareMap,  2.0F);
    }


    public ColorSensor(HardwareMap hardwareMap, float gain) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");
        colorSensor.setGain(gain);
        setLightEnabled(true);
    }

    public void setLightEnabled(boolean enabled) {
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(enabled);
        }
    }

    public DetectedColor getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        if (value < MIN_VALUE) {
            return DetectedColor.NONE;
        }

        if (saturation < MIN_SATURATION) {
            return (value > WHITE_VALUE) ? DetectedColor.WHITE : DetectedColor.NONE;
        }

        if (hue < 20f || hue > 340f) {
            return DetectedColor.RED;
        } else if (hue >= 20f && hue < 40f) {
            return DetectedColor.ORANGE;
        } else if (hue >= 40f && hue <= 70f) {
            return DetectedColor.YELLOW;
        } else if (hue >= 80f && hue <= 150f) {
            return DetectedColor.GREEN;
        } else if (hue >= 190f && hue <= 250f) {
            return DetectedColor.BLUE;
        } else if (hue >= 260f && hue <= 310f) {
            return DetectedColor.PURPLE;
        }

        return DetectedColor.NONE;
    }

    public boolean isColor(DetectedColor target) {
        return getDetectedColor() == target;
    }

    public float getHue() {
        return hsvValues[0];
    }

    public float getSaturation() {
        return hsvValues[1];
    }

    public float getValue() {
        return hsvValues[2];
    }

    public float[] getHSV(){
        return hsvValues;
    }
    public enum DetectedColor {
        RED,
        ORANGE,
        YELLOW,
        GREEN,
        BLUE,
        PURPLE,
        WHITE,
        NONE
    }
}
