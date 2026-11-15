package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
public class Camera {

    public enum TipSide {
        STARTING, SECONDARY
    }
    //Starting is the normal pos at the beginning of a match and secondary is the other one

    public static final String WEBCAM_NAME = "Webcam 1";

    // Cluster names from the SDK's BioBuzz tag library. At the start the CELL that points at a
    // FLOWER is down, which leaves red's audience CELL (tags 34-37) and blue's far "SCORING" CELL
    // (tags 42-45) up (manual Figure 9-17). Those are the starting CELLs; a tip brings the
    // secondary ones up.
    private static final String RED_STARTING_CLUSTER = "RED AUDIENCE";
    private static final String RED_SECONDARY_CLUSTER = "RED SCORING";
    private static final String BLUE_STARTING_CLUSTER = "BLUE SCORING";
    private static final String BLUE_SECONDARY_CLUSTER = "BLUE AUDIENCE";

    // Tune these with ShooterTest, then copy them in here.
    // Drop the hive target after this long without seeing it.
    public static double TARGET_TIMEOUT_MS = 250;
    // The tracked CELL tilting past this pitch means the hive tipped (from HiveTipDetectorTester).
    public static double TIP_PITCH_THRESHOLD_DEG = 28;
    // The pitch must stay past the threshold this long, so one noisy frame can't flip the side.
    public static double TIP_CONFIRM_MS = 300;

    private boolean red;
    private TipSide tipSide = TipSide.STARTING;
    private final ElapsedTime tipSuspectedFor = new ElapsedTime();
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;
    private final ElapsedTime sinceHiveSeen = new ElapsedTime();

    private AprilTagClusterDetection hive = null;
    private boolean freshHiveTarget = false;

    public Camera(HardwareMap hardwareMap, boolean red) {
        this.red = red;
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    public void periodic() {
        freshHiveTarget = false;
        List<AprilTagDetection> detections = aprilTagProcessor.getFreshDetections();
        AprilTagClusterDetection found = detections == null ? null : findAllianceHive(detections);
        if (found != null) {
            hive = found;
            freshHiveTarget = true;
            sinceHiveSeen.reset();
        }
        if (sinceHiveSeen.milliseconds() > TARGET_TIMEOUT_MS) {
            hive = null;
        }
    }

    public void setRedAlliance(boolean red) {
        this.red = red;
    }

    // Watches the tracked CELL's pitch. When it stays tipped, the other CELL is now up, so the
    // tip side swaps and the camera starts tracking the other cluster.
    public void detectTip() {

        //ARAV REDO THIS WITH WHATEVER CONDITION YOU WANT, ONLY CONDITION IS TIPSIDE MUST EQUAL Tipside.Starting or Tipside.secondary.
        // look at the example code:
        // basically just if the hive isn't known or the  condition (here it the the hive ptich degree) is NOT met, then you should reset the timer
        // this was how I implement your idea of having the timer to make sure ocnsistency or was it mr levy idk
        //anwyays jsut replace the second part with your condition, and tip confirm ms is the time threshold u want
//        if (hive == null || Math.abs(getHivePitchDeg()) <= TIP_PITCH_THRESHOLD_DEG) {
//            tipSuspectedFor.reset(); //this is just a timer
//            return;
//        }
//        if (tipSuspectedFor.milliseconds() >= TIP_CONFIRM_MS) {
//            tipSide = tipSide == TipSide.STARTING ? TipSide.SECONDARY : TipSide.STARTING;
//            hive = null;
//            tipSuspectedFor.reset();
//        }
    }

    // Schedule after updateCommand() so it sees this loop's frame.
    public Command tipDetectionCommand() {
        return Command.build()
                .setStart(tipSuspectedFor::reset)
                .setExecute(this::detectTip);
    }

    // Ivy has no subsystem periodic, so schedule this before anything that reads the camera.
    public Command updateCommand() {
        return Command.build()
                .setExecute(this::periodic)
                .requiring(this);
    }

    private AprilTagClusterDetection findAllianceHive(List<AprilTagDetection> detections) {
        String hiveName = tipSide == TipSide.STARTING
                ? (red ? RED_STARTING_CLUSTER : BLUE_STARTING_CLUSTER)
                : (red ? RED_SECONDARY_CLUSTER : BLUE_SECONDARY_CLUSTER);
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata != null && cluster.ftcPose != null
                        && hiveName.equals(cluster.metadata.name)) {
                    return cluster;
                }
            }
        }
        return null;
    }

    public TipSide getTipSide() {
        return tipSide;
    }

    public boolean hasHiveTarget() {
        return hive != null;
    }

    // True only on the loop a new camera frame saw the hive, so each frame is used once.
    public boolean hasFreshHiveTarget() {
        return freshHiveTarget;
    }

    // Cluster pose is solved from whichever member tags are visible and refers to the cluster
    // origin, so it stays centered even when part of the cluster is out of frame.
    // Only call these while hasHiveTarget() is true.

    public double getHiveRightIn() {
        return hive.ftcPose.x;
    }

    public double getHiveForwardIn() {
        return hive.ftcPose.y;
    }

    public double getHiveRangeIn() {
        return hive.ftcPose.range;
    }

    public double getHiveBearingDeg() {
        return hive.ftcPose.bearing;
    }

    public double getHivePitchDeg() {
        return hive.ftcPose.pitch;
    }

    public int getHivePercentFound() {
        return hive.percentClusterFound;
    }

    public void close() {
        visionPortal.close();
    }
}
