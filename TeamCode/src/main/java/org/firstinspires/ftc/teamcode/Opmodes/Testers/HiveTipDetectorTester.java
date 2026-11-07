
package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@TeleOp(name = "Hive Tip Detector", group = "Vision")
public class HiveTipDetectorTester extends LinearOpMode {

    // ==========================================
    // FTC DASHBOARD CONFIGURATION
    // ==========================================

    public static double PITCH_THRESHOLD = 28.0;
    public static double ARAV_IS_AN_IDIOT= 28.0;

    public static double ROLL_THRESHOLD = 15.0;
    public static double TEST_VALUE;

    // ==========================================
    // VISION VARIABLES
    // ==========================================

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        // Send telemetry to Driver Station AND Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();

        telemetry = new MultipleTelemetry(
                telemetry,
                dashboard.getTelemetry()
        );

        TEST_VALUE = 55.0;
        // Initialize AprilTag processor
        aprilTagProcessor =
                AprilTagProcessor.easyCreateWithDefaults();

        // Initialize webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(
                        WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(320,240))
                .build();

        // Display webcam on FTC Dashboard
        dashboard.startCameraStream(visionPortal, 0);

        telemetry.addLine("HIVE TIP DETECTOR READY");
        telemetry.addData("Pitch Threshold", PITCH_THRESHOLD);
        telemetry.addData("Roll Threshold", ROLL_THRESHOLD);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Get all current detections
            List<AprilTagDetection> detections =
                    aprilTagProcessor.getDetections();

            boolean hiveIsTipped = false;

            telemetry.addLine("===== HIVE DETECTOR =====");

            telemetry.addData(
                    "Total Detections", detections.size());

            telemetry.addData(
                    "Pitch Threshold", "%.2f degrees",
                    PITCH_THRESHOLD);

            telemetry.addData(
                    "Roll Threshold", "%.2f degrees",
                    ROLL_THRESHOLD);
            telemetry.addData("",ARAV_IS_AN_IDIOT);

            // ==========================================
            // PROCESS EVERY DETECTION
            // ==========================================

            for (int i = 0; i < detections.size(); i++) {

                AprilTagDetection detection = detections.get(i);

                telemetry.addLine("--------------------------");

                if (detection instanceof AprilTagClusterDetection) {

                    AprilTagClusterDetection cluster =
                            (AprilTagClusterDetection) detection;

                    telemetry.addData(
                            "Detection " + i, "APRILTAG CLUSTER");

                    telemetry.addData(
                            "Cluster Name",
                            cluster.metadata != null
                                    ? cluster.metadata.name
                                    : "Unknown");


                    if (cluster.ftcPose != null) {

                        double pitch = cluster.ftcPose.pitch;
                        double roll = cluster.ftcPose.roll;

                        telemetry.addData(
                                "Cluster Pitch",
                                "%.2f degrees", pitch);

                        telemetry.addData(
                                "Cluster Roll",
                                "%.2f degrees", roll);

                        telemetry.addData(
                                "Cluster Yaw",
                                "%.2f degrees",
                                cluster.ftcPose.yaw);

                        telemetry.addData(
                                "Cluster Range",
                                "%.2f inches",
                                cluster.ftcPose.range);
                        telemetry.addData(
                                "Test",
                                TEST_VALUE);

                        telemetry.addData(
                                "Cluster X",
                                "%.2f inches",
                                cluster.ftcPose.x);

                        telemetry.addData(
                                "Cluster Y",
                                "%.2f inches",
                                cluster.ftcPose.y);

                        telemetry.addData(
                                "Cluster Z",
                                "%.2f inches",
                                cluster.ftcPose.z);

                        telemetry.addData(
                                "Cluster Pitch Status",
                                Math.abs(pitch) > PITCH_THRESHOLD
                                        ? "EXCEEDED"
                                        : "NORMAL");

                        telemetry.addData(
                                "Cluster Roll Status",
                                Math.abs(roll) > ROLL_THRESHOLD
                                        ? "EXCEEDED"
                                        : "NORMAL");

                        if (Math.abs(pitch) > PITCH_THRESHOLD) {
                            hiveIsTipped = true;
                        }
                    }

                    continue;
                }

                // ======================================
                // INDIVIDUAL APRILTAG DETECTION
                // ======================================

                if (detection instanceof AprilTagSingleDetection) {

                    AprilTagSingleDetection tag =
                            (AprilTagSingleDetection) detection;

                    if (tag.ftcPose == null) {
                        continue;
                    }

                    double pitch = tag.ftcPose.pitch;
                    double roll = tag.ftcPose.roll;

                    telemetry.addData(
                            "Detection " + i, "SINGLE APRILTAG");

                    telemetry.addData(
                            "Tag ID", tag.id);

                    telemetry.addData(
                            "Pitch", "%.2f degrees", pitch);

                    telemetry.addData(
                            "Roll", "%.2f degrees", roll);

                    telemetry.addData(
                            "Yaw", "%.2f degrees",
                            tag.ftcPose.yaw);

                    telemetry.addData(
                            "Range", "%.2f inches",
                            tag.ftcPose.range);

                    telemetry.addData(
                            "X", "%.2f inches",
                            tag.ftcPose.x);

                    telemetry.addData(
                            "Y", "%.2f inches",
                            tag.ftcPose.y);

                    telemetry.addData(
                            "Z", "%.2f inches",
                            tag.ftcPose.z);

                    boolean pitchExceeded =
                            Math.abs(pitch) > PITCH_THRESHOLD;

                    boolean rollExceeded =
                            Math.abs(roll) > ROLL_THRESHOLD;

                    telemetry.addData(
                            "Pitch Status",
                            pitchExceeded ? "EXCEEDED" : "NORMAL");

                    telemetry.addData(
                            "Roll Status",
                            rollExceeded ? "EXCEEDED" : "NORMAL");

                    if (pitchExceeded || rollExceeded) {
                        hiveIsTipped = true;
                    }
                }
            }

            // ==========================================
            // OVERALL HIVE STATUS
            // ==========================================

            telemetry.addLine("==========================");

            if (hiveIsTipped) {
                telemetry.addLine("!!! HIVE HAS TIPPED !!!");
            } else {
                telemetry.addLine("HIVE STATUS: NORMAL");
            }

            telemetry.addData(
                    "Camera FPS", visionPortal.getFps());

            telemetry.update();

            sleep(20);
        }

        dashboard.stopCameraStream();
        visionPortal.close();
    }
}
