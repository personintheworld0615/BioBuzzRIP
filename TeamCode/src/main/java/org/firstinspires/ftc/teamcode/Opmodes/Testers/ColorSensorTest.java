package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@TeleOp(name = "Color Sensor HSV Detection")
public class ColorSensorTest extends LinearOpMode {

    private NormalizedColorSensor colorSensor;

    private final float[] hsvValues = new float[3];

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        colorSensor.setGain(2.0f);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        telemetry.addLine("Ready. Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            String detectedColor = detectColor();

            telemetry.addData("Detected Color", detectedColor);
            telemetry.addData("Hue (0-360)", "%.1f", hsvValues[0]);
            telemetry.addData("Saturation (0-1)", "%.3f", hsvValues[1]);
            telemetry.addData("Value (0-1)", "%.3f", hsvValues[2]);
            telemetry.update();
        }
    }

    /**
     * Reads the color sensor, converts NormalizedRGBA to HSV, and classifies the color.
     */
    private String detectColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        // Filter out black/too dark readings
        if (value < 0.15f) {
            return "NONE / TOO DARK";
        }

        // Filter out white, gray, or unpainted surfaces (low saturation)
        if (saturation < 0.45f) {
            return (value > 0.6f) ? "WHITE" : "GRAY / UNKNOWN";
        }

        // Classify dominant color by Hue angle
        if (hue < 20f || hue > 340f) {
            return "RED";
        } else if (hue >= 20f && hue < 40f) {
            return "ORANGE";
        } else if (hue >= 40f && hue <= 70f) {
            return "YELLOW";
        } else if (hue >= 80f && hue <= 150f) {
            return "GREEN";
        } else if (hue >= 190f && hue <= 250f) {
            return "BLUE";
        } else if (hue >= 260f && hue <= 310f) {
            return "PURPLE";
        }

        return "UNKNOWN";
    }
}
