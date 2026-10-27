package org.firstinspires.ftc.teamcode.opmodes.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Hive Tipped Detector", group = "Vision")
public class Yogurt extends LinearOpMode {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;


    private static final double TIPPED_THRESHOLD = 15.0;

    @Override
    public void runOpMode() {
        // initialize april tag processor
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        //build vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();



        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
            boolean hiveIsTipped = false;

            for (AprilTagDetection detection : currentDetections) {

                if (detection.ftcPose != null) {
                    int tagId = 34;
                    double pitch = detection.ftcPose.pitch;
                    double roll = detection.ftcPose.roll;


                    telemetry.addData("Tag ID", tagId);
                    telemetry.addData("Pitch", "%.2f degrees", pitch);
                    telemetry.addData("Roll", "%.2f degrees", roll);

                    //if the values are greater than the threshold then the hive is considered tipped
                    if (Math.abs(pitch) > TIPPED_THRESHOLD || Math.abs(roll) > TIPPED_THRESHOLD) {
                        hiveIsTipped = true;
                    }
                }
            }



            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }
}