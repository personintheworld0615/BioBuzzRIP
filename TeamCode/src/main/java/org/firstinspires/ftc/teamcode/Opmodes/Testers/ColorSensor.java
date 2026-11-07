package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensor {

    public NormalizedColorSensor colorSensor;

    public enum DetectColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }
 
    // Constructor or initialization method
    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");
    }

    public DetectColor getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;

        // Threshold evaluation (adjust limits based on sensor distance and ambient light)
        if (red > blue && red > green && red > 0.05f) {
            return DetectColor.RED;
        } else if (blue > red && blue > green && blue > 0.05f) {
            return DetectColor.BLUE;
        } else if (red > 0.05f && green > 0.05f && blue < 0.05f) {
            return DetectColor.YELLOW;
        }

        return DetectColor.UNKNOWN;
    }
}