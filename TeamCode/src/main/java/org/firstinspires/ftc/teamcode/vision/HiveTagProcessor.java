package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.vision.HiveTip.CameraPose;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Hive;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Snapshot;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Tag;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Runs AprilTag detection for the hive tags, estimates their heights and the hive tip, and
 * draws the result for LiveView, the Driver Station stream and the FTC Dashboard stream.
 *
 * Why a custom tag library: in SDK 12 the default BIOBUZZ library only defines tags 30-45
 * as four clusters, so the processor never returns single detections for them and the
 * cluster pose points at the CELL opening rather than at a tag. Registering 30-45 as
 * individual 3.25 in tags gives one pose per tag, which is what height needs.
 */
public class HiveTagProcessor implements VisionProcessor, CameraStreamSource {

    // Logitech C270 at 640x480, from the SDK's built-in calibration (teamwebcamcalibrations.xml).
    public static final int CAMERA_WIDTH_PX = 640;
    public static final int CAMERA_HEIGHT_PX = 480;
    private static final double C270_FX = 822.317;
    private static final double C270_FY = 822.317;
    private static final double C270_CX = 319.495;
    private static final double C270_CY = 242.502;

    static final int ABOVE_THRESHOLD_COLOR = Color.rgb(0, 230, 118);   // green
    static final int BELOW_THRESHOLD_COLOR = Color.rgb(255, 160, 0);   // amber
    private static final int TEXT_BACKGROUND = Color.argb(170, 0, 0, 0);
    private static final float OUTLINE_WIDTH_PX = 4f;
    private static final float TAG_TEXT_PX = 16f;
    private static final float BANNER_TEXT_PX = 15f;
    private static final float PAD_PX = 4f;

    private final AprilTagProcessor aprilTag = buildAprilTagProcessor();

    /** Set from the OpMode thread; takes effect on the next frame. */
    private volatile CameraPose pose;
    private volatile double thresholdIn;
    private final AtomicReference<Snapshot> latest = new AtomicReference<>(Snapshot.EMPTY);
    /**
     * Last frame rendered for the DS/Dashboard streams. Never null: Dashboard blocks without a
     * timeout until getFrameBitmap answers. A published bitmap is never drawn on again.
     */
    private final AtomicReference<Bitmap> streamBitmap = new AtomicReference<>(
            Bitmap.createBitmap(CAMERA_WIDTH_PX, CAMERA_HEIGHT_PX, Bitmap.Config.RGB_565));
    /** Render a stream frame only when a viewer has asked for one since the last render. */
    private final AtomicBoolean isStreamFrameRequested = new AtomicBoolean(true);

    public HiveTagProcessor(CameraPose pose, double thresholdIn) {
        updateSettings(pose, thresholdIn);
    }

    private static AprilTagProcessor buildAprilTagProcessor() {
        AprilTagLibrary.Builder library = new AprilTagLibrary.Builder();
        for (int id = HiveTip.FIRST_TAG_ID; id <= HiveTip.LAST_TAG_ID; id++) {
            library.addTag(id, HiveTip.cellName(id) + " " + id, HiveTip.TAG_SIZE_IN, DistanceUnit.INCH);
        }
        return new AprilTagProcessor.Builder()
                .setTagLibrary(library.build())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(C270_FX, C270_FY, C270_CX, C270_CY)
                .setDrawTagOutline(false)   // we draw our own colour-coded outline
                .setDrawTagID(false)
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .build();
    }

    public void updateSettings(CameraPose pose, double thresholdIn) {
        this.pose = pose;
        this.thresholdIn = thresholdIn;
    }

    public void setDecimation(float decimation) {
        aprilTag.setDecimation(decimation);
    }

    public Snapshot getLatest() {
        return latest.get();
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        aprilTag.init(width, height, calibration);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        aprilTag.processFrame(frame, captureTimeNanos);
        CameraPose currentPose = pose;
        double currentThreshold = thresholdIn;

        List<Tag> tags = new ArrayList<>();
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            if (!(detection instanceof AprilTagSingleDetection) || detection.ftcPose == null) {
                continue;   // unknown ids have no metadata and therefore no pose
            }
            AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
            if (HiveTip.isHiveTag(tag.id)) {
                tags.add(HiveTip.estimateTag(tag.id, tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z,
                        toFloats(tag.corners), currentPose, currentThreshold));
            }
        }

        Snapshot snapshot = latest.get().next(tags, System.nanoTime());
        latest.set(snapshot);
        if (isStreamFrameRequested.getAndSet(false)) {
            renderStreamFrame(frame, snapshot);
        }
        return snapshot;
    }

    /** Copies the camera frame into a bitmap with the overlay burned in, for DS/Dashboard. */
    private void renderStreamFrame(Mat frame, Snapshot snapshot) {
        Bitmap bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, bitmap);
        drawOverlay(new Canvas(bitmap), 1f, 1f, snapshot);
        streamBitmap.set(bitmap);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        if (userContext instanceof Snapshot) {
            drawOverlay(canvas, scaleBmpPxToCanvasPx, scaleCanvasDensity, (Snapshot) userContext);
        }
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        final Bitmap bitmap = streamBitmap.get();
        isStreamFrameRequested.set(true);
        continuation.dispatch(consumer -> consumer.accept(bitmap));
    }

    /**
     * Draws tag outlines, height labels and the tip banner. Called from both the vision and
     * UI threads, so paints are created per call rather than shared.
     */
    private static void drawOverlay(Canvas canvas, float scale, float density, Snapshot snapshot) {
        Paint outline = new Paint(Paint.ANTI_ALIAS_FLAG);
        outline.setStyle(Paint.Style.STROKE);
        outline.setStrokeWidth(OUTLINE_WIDTH_PX * density);
        Paint text = new Paint(Paint.ANTI_ALIAS_FLAG);
        Paint background = new Paint();
        background.setColor(TEXT_BACKGROUND);

        for (Tag tag : snapshot.tags) {
            float[] c = tag.cornersPx;
            outline.setColor(tag.isAboveThreshold ? ABOVE_THRESHOLD_COLOR : BELOW_THRESHOLD_COLOR);
            Path path = new Path();
            path.moveTo(c[0] * scale, c[1] * scale);
            for (int i = 2; i < c.length; i += 2) {
                path.lineTo(c[i] * scale, c[i + 1] * scale);
            }
            path.close();
            canvas.drawPath(path, outline);

            String label = String.format(Locale.US, "%d: %.1f\"%s", tag.id, tag.heightIn,
                    tag.isPositionConsistent ? "" : " ?");
            float left = Math.min(Math.min(c[0], c[2]), Math.min(c[4], c[6])) * scale;
            float top = Math.min(Math.min(c[1], c[3]), Math.min(c[5], c[7])) * scale;
            drawLabel(canvas, text, background, label, left, top - PAD_PX * density,
                    TAG_TEXT_PX * density, outline.getColor());
        }

        float size = BANNER_TEXT_PX * density;
        float y = size + PAD_PX * density;
        for (Hive hive : Hive.values()) {
            Boolean isRightTipped = snapshot.isRightTipped(hive);
            String line = String.format(Locale.US, "%s isRightTipped=%s (%s)", hive,
                    isRightTipped == null ? "?" : isRightTipped,
                    snapshot.sourceLabel(hive, snapshot.timeNanos));
            drawLabel(canvas, text, background, line, PAD_PX * density, y, size, Color.WHITE);
            y += size + 2 * PAD_PX * density;
        }
    }

    private static void drawLabel(Canvas canvas, Paint text, Paint background, String label,
                                  float x, float baselineY, float size, int color) {
        text.setTextSize(size);
        text.setColor(color);
        float pad = size / 4f;
        canvas.drawRect(x - pad, baselineY - size - pad, x + text.measureText(label) + pad,
                baselineY + pad, background);
        canvas.drawText(label, x, baselineY, text);
    }

    private static float[] toFloats(Point[] corners) {
        float[] out = new float[corners.length * 2];
        for (int i = 0; i < corners.length; i++) {
            out[2 * i] = (float) corners[i].x;
            out[2 * i + 1] = (float) corners[i].y;
        }
        return out;
    }
}
