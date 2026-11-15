package org.firstinspires.ftc.teamcode.Util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MyTelem {
    private static Telemetry telemetry;


    public static void init(Telemetry opModeTelemetry) {
        telemetry = new MultipleTelemetry(opModeTelemetry,FtcDashboard.getInstance().getTelemetry());
    }

    public static void addData(String caption, Object value) {
        if (telemetry != null) {
            telemetry.addData(caption, value);
        }
    }

    public static void addLine(String line) {
        if (telemetry != null) {
            telemetry.addLine(line);
        }
    }

    public static void update() {
        if (telemetry != null) {
            telemetry.update();
        }
    }
}

