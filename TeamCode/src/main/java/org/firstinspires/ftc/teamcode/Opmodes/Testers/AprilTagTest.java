package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.vision.HiveTagProcessor;
import org.firstinspires.ftc.teamcode.vision.HiveTip;
import org.firstinspires.ftc.teamcode.vision.HiveTip.CameraPose;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Hive;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Snapshot;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Tag;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Reports the height above the tiles of every hive AprilTag (30-45) a C270 can see, marks
 * tags above {@link #HEIGHT_THRESHOLD_IN} in green on the stream, and decides whether each
 * hive is RIGHT side up (starting configuration) or LEFT side up.
 *
 * Without a robot, type the camera's field position into ROBOT_X/Y/HEADING on the
 * Dashboard. Coordinates are FTC field coordinates (inches, origin at field centre,
 * +X toward the audience, +Y toward blue, heading CCW from +X) - not Pedro's 0-144 frame.
 */
@Config
@TeleOp(name = "AprilTag Test", group = "Testers")
public class AprilTagTest extends LinearOpMode {

    public enum HeadlineHive { NEAREST, RED, BLUE }

    // Camera mount (Dashboard-tunable)
    public static double CAMERA_HEIGHT_IN = 12.0;
    public static double CAMERA_PITCH_DEG = 35.0;
    public static double CAMERA_FORWARD_OFFSET_IN = 0.0;
    public static double CAMERA_LEFT_OFFSET_IN = 0.0;

    // Robot pose in FTC field coordinates; enter by hand when testing without a robot.
    public static double ROBOT_X_IN = 0.0;
    public static double ROBOT_Y_IN = -48.0;
    public static double ROBOT_HEADING_DEG = 90.0;

    public static double HEIGHT_THRESHOLD_IN = 45.0;
    /** Which hive drives the headline isRightTipped value. NEAREST uses ROBOT_Y_IN. */
    public static HeadlineHive HEADLINE_HIVE = HeadlineHive.NEAREST;
    public static double DECIMATION = 2.0;

    private static final double STREAM_MAX_FPS = 15;
    /** Telemetry refresh period; keeps the loop from starving the vision thread. */
    private static final long LOOP_PERIOD_MS = 50;

    private double appliedDecimation = Double.NaN;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        HiveTagProcessor hiveTags = new HiveTagProcessor(readPose(), HEIGHT_THRESHOLD_IN);
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(HiveTagProcessor.CAMERA_WIDTH_PX, HiveTagProcessor.CAMERA_HEIGHT_PX))
                .addProcessor(hiveTags)
                .build();
        FtcDashboard.getInstance().startCameraStream(hiveTags, STREAM_MAX_FPS);
        CameraStreamServer.getInstance().setSource(hiveTags);

        try {
            // Works during INIT too, so the camera can be tested without pressing start.
            while (opModeInInit()) {
                loopOnce(hiveTags, portal);
            }
            while (opModeIsActive()) {
                loopOnce(hiveTags, portal);
            }
        } finally {
            FtcDashboard.getInstance().stopCameraStream();
            portal.close();
        }
    }

    private void loopOnce(HiveTagProcessor hiveTags, VisionPortal portal) {
        CameraPose pose = readPose();
        Hive headline = headlineHive();
        hiveTags.updateSettings(pose, HEIGHT_THRESHOLD_IN);
        if (DECIMATION != appliedDecimation) {
            hiveTags.setDecimation((float) DECIMATION);
            appliedDecimation = DECIMATION;
        }

        Snapshot snapshot = hiveTags.getLatest();
        long now = System.nanoTime();

        telemetry.addData("isRightTipped (" + headline + ")", describe(snapshot.isRightTipped(headline)));
        telemetry.addData("  source", snapshot.sourceLabel(headline, now));
        for (Hive hive : Hive.values()) {
            telemetry.addData(hive + " hive", "isRightTipped=%s (%s)",
                    describe(snapshot.isRightTipped(hive)), snapshot.sourceLabel(hive, now));
        }

        telemetry.addData("Pose (field in/deg)", "x=%.1f y=%.1f h=%.1f",
                pose.robotXIn, pose.robotYIn, pose.robotHeadingDeg);
        telemetry.addData("Camera", "height=%.1fin pitch=%.1fdeg", pose.camHeightIn, pose.camPitchDeg);
        telemetry.addData("Threshold", "%.1f in", HEIGHT_THRESHOLD_IN);
        telemetry.addData("Tags visible", snapshot.tags.size());
        for (Tag tag : snapshot.tags) {
            telemetry.addData("Tag " + tag.id + " " + HiveTip.cellName(tag.id),
                    "h=%.1fin %s | field (%.1f, %.1f) | dist %.1fin%s",
                    tag.heightIn, tag.isAboveThreshold ? "ABOVE" : "below",
                    tag.fieldXIn, tag.fieldYIn, tag.horizontalDistanceIn,
                    tag.isPositionConsistent ? "" : " | CHECK POSE: tag not where its cell is");
        }
        telemetry.addData("FPS", "%.1f", portal.getFps());
        telemetry.update();
        sleep(LOOP_PERIOD_MS);
    }

    private static CameraPose readPose() {
        return new CameraPose(ROBOT_X_IN, ROBOT_Y_IN, ROBOT_HEADING_DEG,
                CAMERA_FORWARD_OFFSET_IN, CAMERA_LEFT_OFFSET_IN, CAMERA_HEIGHT_IN, CAMERA_PITCH_DEG);
    }

    private static Hive headlineHive() {
        switch (HEADLINE_HIVE) {
            case RED:
                return Hive.RED;
            case BLUE:
                return Hive.BLUE;
            default:
                return Hive.nearestTo(ROBOT_Y_IN);
        }
    }

    private static String describe(Boolean isRightTipped) {
        return isRightTipped == null ? "unknown (no tags seen yet)" : isRightTipped.toString();
    }
}
