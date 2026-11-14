package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Each run creates /sdcard/FIRST/TestN/ with:
 *   arctan.csv, arctanfiltered.csv, range.csv, rangefiltered.csv
 *
 * Flip TIPPED on Dashboard to mark tip=1 in every CSV, then logging continues.
 */
@Config
@TeleOp(name = "Hive Tip Data Logger", group = "Vision")
public class HiveTipDataLogger extends LinearOpMode {

    /** Flip this on Dashboard to mark tip start. */
    public static double TIPPED = 0.0;
    public static int FILTER_SIZE = 5;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private final ArrayDeque<Double> angleHistory = new ArrayDeque<>();
    private final ArrayDeque<Double> rangeHistory = new ArrayDeque<>();

    private double previousTipped = TIPPED;
    public static boolean tipMarked = false;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .build();
        dashboard.startCameraStream(visionPortal, 0);

        File testFolder = nextTestFolder();
        previousTipped = TIPPED;

        telemetry.addLine("Hive Tip Data Logger Ready");
        telemetry.addData("Folder", testFolder.getAbsolutePath());
        telemetry.addLine("Flip TIPPED on Dashboard to mark tip");
        telemetry.update();

        waitForStart();

        FileWriter arctan = null;
        FileWriter arctanFiltered = null;
        FileWriter rangeOut = null;
        FileWriter rangeFiltered = null;

        try {
            arctan = openCsv(new File(testFolder, "arctan.csv"));
            arctanFiltered = openCsv(new File(testFolder, "arctanfiltered.csv"));
            rangeOut = openCsv(new File(testFolder, "range.csv"));
            rangeFiltered = openCsv(new File(testFolder, "rangefiltered.csv"));

            ElapsedTime runtime = new ElapsedTime();

            while (opModeIsActive()) {
                if (!tipMarked && TIPPED != previousTipped) {
                    tipMarked = true;
                }
                previousTipped = TIPPED;

                AprilTagClusterDetection cluster = findCluster();
                int tip = tipMarked ? 1 : 0;
                double t = runtime.seconds();

                if (cluster == null || cluster.ftcPose == null) {
                    angleHistory.clear();
                    rangeHistory.clear();

                    writeRow(arctan, t, Double.NaN, tip);
                    writeRow(arctanFiltered, t, Double.NaN, tip);
                    writeRow(rangeOut, t, Double.NaN, tip);
                    writeRow(rangeFiltered, t, Double.NaN, tip);

                    telemetry.addLine("NO CLUSTER");
                    telemetry.addData("Tip Marked", tipMarked);
                    telemetry.addData("Folder", testFolder.getName());
                    telemetry.update();
                    sleep(20);
                    continue;
                }

                double rawAngle = Math.toDegrees(
                        Math.atan2(cluster.ftcPose.z, cluster.ftcPose.y));
                double rawRange = cluster.ftcPose.range;

                int filterSize = Math.max(1, FILTER_SIZE);
                pushBounded(angleHistory, rawAngle, filterSize);
                pushBounded(rangeHistory, rawRange, filterSize);

                double filteredAngle = getMedian(angleHistory);
                double filteredRange = getMedian(rangeHistory);

                writeRow(arctan, t, rawAngle, tip);
                writeRow(arctanFiltered, t, filteredAngle, tip);
                writeRow(rangeOut, t, rawRange, tip);
                writeRow(rangeFiltered, t, filteredRange, tip);

                telemetry.addData("Folder", testFolder.getName());
                telemetry.addData("Tip Marked", tipMarked);
                telemetry.addData("Angle", "%.2f / %.2f", rawAngle, filteredAngle);
                telemetry.addData("Range", "%.2f / %.2f", rawRange, filteredRange);
                telemetry.update();

                sleep(20);
            }
        } catch (IOException e) {
            telemetry.addData("File Error", e.getMessage());
            telemetry.update();
        } finally {
            closeQuietly(arctan);
            closeQuietly(arctanFiltered);
            closeQuietly(rangeOut);
            closeQuietly(rangeFiltered);
            dashboard.stopCameraStream();
            visionPortal.close();
        }
    }

    private AprilTagClusterDetection findCluster() {
        for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
            if (detection instanceof AprilTagClusterDetection) {
                return (AprilTagClusterDetection) detection;
            }
        }
        return null;
    }

    /** Creates /sdcard/FIRST/Test1, Test2, ... for each run. */
    private File nextTestFolder() {
        File root = new File("/sdcard/FIRST");
        int n = 1;
        File folder;
        do {
            folder = new File(root, "Test" + n);
            n++;
        } while (folder.exists());
        //noinspection ResultOfMethodCallIgnored
        folder.mkdirs();
        return folder;
    }

    private FileWriter openCsv(File file) throws IOException {
        FileWriter writer = new FileWriter(file, false);
        writer.write("time_s,value,tip\n");
        writer.flush();
        return writer;
    }

    private void writeRow(FileWriter writer, double timeSec, double value, int tip)
            throws IOException {
        writer.write(String.format(Locale.US, "%.4f,%.4f,%d\n", timeSec, value, tip));
        writer.flush();
    }

    private void pushBounded(ArrayDeque<Double> history, double value, int maxSize) {
        history.addLast(value);
        while (history.size() > maxSize) {
            history.removeFirst();
        }
    }

    private double getMedian(ArrayDeque<Double> history) {
        if (history.isEmpty()) {
            return 0;
        }
        double[] values = new double[history.size()];
        int i = 0;
        for (double v : history) {
            values[i++] = v;
        }
        Arrays.sort(values);
        int mid = values.length / 2;
        if (values.length % 2 == 0) {
            return (values[mid - 1] + values[mid]) / 2.0;
        }
        return values[mid];
    }

    private void closeQuietly(FileWriter writer) {
        if (writer == null) {
            return;
        }
        try {
            writer.close();
        } catch (IOException ignored) {
        }
    }
}
