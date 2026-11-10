
package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "Hive Motion Graph Tester", group = "Vision")
public class HiveMotionGraphTester extends LinearOpMode {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    // ---------------- CONFIGURATION ----------------

    // Odd-sized median filter is recommended
    public static int FILTER_SIZE = 5;

    // Time window used for rolling linear regression
    public static double REGRESSION_WINDOW_SECONDS = 0.25;

    // Placeholder for future tipping logic
    public static double changemewhenhivetips = 0.0;

    // ---------------- HISTORIES ----------------

    private final ArrayDeque<Double> pitchHistory =
            new ArrayDeque<>();

    private final ArrayDeque<Double> rangeHistory =
            new ArrayDeque<>();

    private static class MotionSample {
        double time;
        double pitch;

        MotionSample(double time, double pitch) {
            this.time = time;
            this.pitch = pitch;
        }
    }

    // Stores unwrapped pitch over time
    private final ArrayDeque<MotionSample> regressionHistory =
            new ArrayDeque<>();

    // ---------------- PREVIOUS VALUES ----------------

    private double previousPitch = 0;
    private double previousRange = 0;

    private double previousTime = 0;

    private double cumulativePitch = 0;

    private boolean initialized = false;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(
                telemetry,
                dashboard.getTelemetry()
        );

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();

        dashboard.startCameraStream(visionPortal, 0);

        telemetry.addLine("Hive Motion Graph Tester Ready");
        telemetry.update();

        waitForStart();

        try {

            while (opModeIsActive()) {

                List<AprilTagDetection> detections =
                        aprilTagProcessor.getDetections();

                AprilTagClusterDetection cluster = null;

                // Find the first detected cluster
                for (AprilTagDetection detection : detections) {
                    if (detection instanceof AprilTagClusterDetection) {
                        cluster = (AprilTagClusterDetection) detection;
                        break;
                    }
                }

                if (cluster == null) {

                    resetHistory();

                    telemetry.addLine("NO CLUSTER DETECTED");
                    telemetry.update();

                    sleep(20);
                    continue;
                }

                // ---------------- RAW MEASUREMENTS ----------------

                double rawPitch = cluster.ftcPose.pitch;
                double rawRange = cluster.ftcPose.range;

                double currentTime =
                        System.nanoTime() / 1_000_000_000.0;

                // ---------------- MEDIAN FILTER ----------------

                int filterSize = Math.max(1, FILTER_SIZE);

                pitchHistory.addLast(rawPitch);
                rangeHistory.addLast(rawRange);

                trimHistory(pitchHistory, filterSize);
                trimHistory(rangeHistory, filterSize);

                double filteredPitch = getMedian(pitchHistory);
                double filteredRange = getMedian(rangeHistory);

                // ---------------- INITIALIZATION ----------------

                if (!initialized) {

                    previousPitch = filteredPitch;
                    previousRange = filteredRange;
                    previousTime = currentTime;

                    cumulativePitch = 0;

                    regressionHistory.clear();

                    regressionHistory.addLast(
                            new MotionSample(currentTime, cumulativePitch)
                    );

                    initialized = true;

                    telemetry.addData("Raw Pitch", rawPitch);
                    telemetry.addData("Filtered Pitch", filteredPitch);

                    telemetry.addData("Raw Range", rawRange);
                    telemetry.addData("Filtered Range", filteredRange);

                    telemetry.addLine("Initializing");

                    telemetry.update();

                    sleep(20);
                    continue;
                }

                // ---------------- TIME ----------------

                double deltaTime = currentTime - previousTime;

                if (deltaTime <= 0) {
                    sleep(20);
                    continue;
                }

                // ---------------- INSTANTANEOUS VELOCITY ----------------

                // Account for angular wrapping
                double pitchChange =
                        wrapDegrees(filteredPitch - previousPitch);

                // Unwrap pitch over time
                cumulativePitch += pitchChange;

                // Do NOT median-filter this derivative.
                // We want to preserve short movement bursts.
                double instantaneousPitchSlope =
                        pitchChange / deltaTime;

                // Range velocity
                double rangeSlope =
                        (filteredRange - previousRange) / deltaTime;

                // ---------------- ROLLING REGRESSION ----------------

                regressionHistory.addLast(
                        new MotionSample(currentTime, cumulativePitch)
                );

                double windowSeconds =
                        Math.max(0.05, REGRESSION_WINDOW_SECONDS);

                double cutoffTime = currentTime - windowSeconds;

                // Keep at least two samples
                while (regressionHistory.size() > 2
                        && regressionHistory.peekFirst().time < cutoffTime) {

                    regressionHistory.removeFirst();
                }

                // Calculate overall pitch velocity using regression
                double regressionSlope =
                        calculateRegressionSlope(regressionHistory);

                // ---------------- ACCELERATION ----------------

                // Acceleration from the regression velocity would require
                // storing its previous value. This is intentionally omitted
                // for now because differentiating noisy velocity amplifies noise.

                // ---------------- TELEMETRY ----------------

                telemetry.addData("Raw Pitch", rawPitch);
                telemetry.addData("Filtered Pitch", filteredPitch);

                telemetry.addData("Raw Range", rawRange);
                telemetry.addData("Filtered Range", filteredRange);

                telemetry.addData(
                        "Instantaneous Pitch Slope (deg/s)",
                        instantaneousPitchSlope
                );

                telemetry.addData(
                        "Regression Pitch Slope (deg/s)",
                        regressionSlope
                );

                telemetry.addData(
                        "Range Slope",
                        rangeSlope
                );

                telemetry.addData(
                        "Cumulative Pitch",
                        cumulativePitch
                );

                telemetry.addData(
                        "Regression Samples",
                        regressionHistory.size()
                );

                telemetry.addData("Delta Time", deltaTime);
                telemetry.addData("Filter Size", filterSize);

                telemetry.addData(
                        "Regression Window",
                        windowSeconds
                );

                telemetry.addData(
                        "Changeme",
                        changemewhenhivetips
                );

                telemetry.addData(
                        "Cluster Name",
                        cluster.metadata.name
                );

                telemetry.addData(
                        "Camera FPS",
                        visionPortal.getFps()
                );

                telemetry.update();

                // ---------------- UPDATE PREVIOUS VALUES ----------------

                previousPitch = filteredPitch;
                previousRange = filteredRange;
                previousTime = currentTime;

                sleep(20);
            }

        } finally {

            dashboard.stopCameraStream();
            visionPortal.close();

        }
    }

    // ---------------- REGRESSION ----------------

    // Returns the least-squares slope in degrees per second
    private double calculateRegressionSlope(
            ArrayDeque<MotionSample> history
    ) {

        if (history.size() < 2) {
            return 0;
        }

        double meanTime = 0;
        double meanPitch = 0;

        for (MotionSample sample : history) {
            meanTime += sample.time;
            meanPitch += sample.pitch;
        }

        meanTime /= history.size();
        meanPitch /= history.size();

        double numerator = 0;
        double denominator = 0;

        for (MotionSample sample : history) {

            double timeDifference = sample.time - meanTime;
            double pitchDifference = sample.pitch - meanPitch;

            numerator += timeDifference * pitchDifference;
            denominator += timeDifference * timeDifference;
        }

        if (denominator == 0) {
            return 0;
        }

        return numerator / denominator;
    }

    // ---------------- MEDIAN FILTER ----------------

    private double getMedian(ArrayDeque<Double> history) {

        if (history.isEmpty()) {
            return 0;
        }

        double[] values = new double[history.size()];

        int index = 0;

        for (double value : history) {
            values[index++] = value;
        }

        Arrays.sort(values);

        int middle = values.length / 2;

        if (values.length % 2 == 0) {
            return (values[middle - 1] + values[middle]) / 2.0;
        }

        return values[middle];
    }

    private void trimHistory(
            ArrayDeque<Double> history,
            int maxSize
    ) {

        while (history.size() > maxSize) {
            history.removeFirst();
        }
    }

    // ---------------- RESET ----------------

    private void resetHistory() {

        initialized = false;

        pitchHistory.clear();
        rangeHistory.clear();
        regressionHistory.clear();

        cumulativePitch = 0;
    }

    // ---------------- ANGLE WRAPPING ----------------

    private double wrapDegrees(double angle) {

        while (angle > 180) {
            angle -= 360;
        }

        while (angle < -180) {
            angle += 360;
        }

        return angle;
    }
}
