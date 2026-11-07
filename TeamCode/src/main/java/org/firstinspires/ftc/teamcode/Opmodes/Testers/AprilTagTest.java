
package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@TeleOp(name = "AprilTag Cluster Test")
public class AprilTagTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize webcam
        WebcamName webcam = hardwareMap.get(
                WebcamName.class, "Webcam 1");

        // Initialize AprilTag processor
        AprilTagProcessor aprilTagProcessor =
                AprilTagProcessor.easyCreateWithDefaults();

        // Initialize VisionPortal
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addLine("AprilTag Cluster Detector initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections =
                    aprilTagProcessor.getDetections();

            telemetry.addData(
                    "Clusters Detected", detections.size());

            for (AprilTagDetection detection : detections) {

                // Only process AprilTag clusters
                if (detection instanceof AprilTagClusterDetection) {

                    AprilTagClusterDetection cluster =
                            (AprilTagClusterDetection) detection;

                    telemetry.addLine("======================");

                    // Cluster identification
                    telemetry.addData(
                            "Cluster Name",
                            cluster.metadata.name);

                    telemetry.addData(
                            "Percent Cluster Found",
                            "%.1f%%",
                            cluster.percentClusterFound * 100.0);

                    // Cluster position and orientation
                    if (cluster.ftcPose != null) {

                        telemetry.addData(
                                "Distance (in)",
                                "%.2f",
                                cluster.ftcPose.range);

                        telemetry.addData(
                                "X (in)",
                                "%.2f",
                                cluster.ftcPose.x);

                        telemetry.addData(
                                "Y (in)",
                                "%.2f",
                                cluster.ftcPose.y);

                        telemetry.addData(
                                "Z (in)",
                                "%.2f",
                                cluster.ftcPose.z);

                        telemetry.addData(
                                "Pitch (deg)",
                                "%.2f",
                                cluster.ftcPose.pitch);

                        telemetry.addData(
                                "Roll (deg)",
                                "%.2f",
                                cluster.ftcPose.roll);

                        telemetry.addData(
                                "Yaw (deg)",
                                "%.2f",
                                cluster.ftcPose.yaw);
                    }

                } else {
                    telemetry.addLine(
                            "Non-cluster AprilTag ignored");
                }
            }

            telemetry.addData("FPS", portal.getFps());
            telemetry.update();
        }

        portal.close();
    }
}
