
package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@TeleOp(name = "AprilTag Test")
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

        telemetry.addLine("AprilTag detector initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections =
                    aprilTagProcessor.getDetections();

            telemetry.addData(
                    "Tag Detected", !detections.isEmpty());

            for (AprilTagDetection detection : detections) {

                if (detection instanceof AprilTagSingleDetection) {

                    AprilTagSingleDetection tag =
                            (AprilTagSingleDetection) detection;

                    telemetry.addData("Tag ID", tag.id);

                    if (tag.ftcPose != null) {
                        telemetry.addData(
                                "Distance (in)",
                                "%.2f", tag.ftcPose.range);

                        telemetry.addData(
                                "X (in)",
                                "%.2f", tag.ftcPose.x);

                        telemetry.addData(
                                "Y (in)",
                                "%.2f", tag.ftcPose.y);

                        telemetry.addData(
                                "Z (in)",
                                "%.2f", tag.ftcPose.z);
                    }

                } else if (
                        detection instanceof AprilTagClusterDetection) {

                    telemetry.addLine(
                            "AprilTag cluster detected");
                }
            }

            telemetry.addData("FPS", portal.getFps());
            telemetry.update();
        }

        portal.close();
    }
}
