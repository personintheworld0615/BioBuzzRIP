package org.firstinspires.ftc.teamcode.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * BIOBUZZ hive tip detection logic: tag layout, tag height geometry and the tip vote.
 * Plain Java (no Android classes) so it can be unit tested on a PC.
 *
 * <p>Tag layout (competition manual TU06, section 9.6.1). Each hive CELL carries four
 * 3.25 in 36h11 tags on its bottom face, facing the tiles:
 * <pre>
 *   30-33  RED  scoring  cell (far from audience, -X)   left  cell from red DS
 *   34-37  RED  audience cell (+X)                      right cell from red DS
 *   38-41  BLUE audience cell (+X)                      left  cell from blue DS
 *   42-45  BLUE scoring  cell (far from audience, -X)   right cell from blue DS
 * </pre>
 * In the starting configuration (Figure 10-2) the red audience cell and the blue scoring
 * cell are raised, i.e. both hives start "right side tipped up" from their own alliance's
 * driver station.
 *
 * <p>Geometry: ftcPose is in the camera frame (x right, y forward, z up). With the camera
 * pitched up by p, a point at (y, z) is
 * <pre>
 *   height     = camHeight + y*sin(p) + z*cos(p)
 *   horizontal =             y*cos(p) - z*sin(p)
 * </pre>
 * and x stays horizontal (no camera roll). The field position then follows from the
 * camera's field position and heading.
 */
public final class HiveTip {

    public static final int FIRST_TAG_ID = 30;
    public static final int LAST_TAG_ID = 45;
    public static final double TAG_SIZE_IN = 3.25;

    private static final int RED_AUDIENCE_FIRST = 34;
    private static final int BLUE_AUDIENCE_FIRST = 38;
    private static final int BLUE_SCORING_FIRST = 42;

    /** Tags must be within this distance (in Y) of their hive's pivot to be consistent. */
    static final double MAX_HIVE_Y_ERROR_IN = 12.0;
    /**
     * Below-threshold tags only vote when at least this far below it. Raised-cell tags sit
     * only a few inches above 45 in, so a small pitch/height error can push one just under.
     */
    static final double LOW_TAG_MARGIN_IN = 5.0;

    private HiveTip() {
    }

    /**
     * The two HIVEs. Positions are FTC field coordinates: origin at field centre, +X toward
     * the audience, +Y from the red wall toward the blue wall, inches. The hives sit 25.5 in
     * apart centre-to-centre, so each pivot is 12.75 in off the field centre line.
     */
    public enum Hive {
        RED(-12.75),
        BLUE(12.75);

        /** Field Y of this hive's pivot axis, inches. */
        public final double fieldYIn;

        Hive(double fieldYIn) {
            this.fieldYIn = fieldYIn;
        }

        /** The hive on the robot's half of the field. */
        public static Hive nearestTo(double robotFieldYIn) {
            return robotFieldYIn < 0 ? RED : BLUE;
        }
    }

    /**
     * Where the camera is on the field. Robot pose is in FTC field coordinates (heading CCW
     * from +X), NOT Pedro's 0-144 frame. The camera points along the robot's heading,
     * pitched up by {@link #camPitchDeg}.
     */
    public static final class CameraPose {
        public final double robotXIn;
        public final double robotYIn;
        public final double robotHeadingDeg;
        /** Lens position relative to robot centre: + forward. */
        public final double camForwardIn;
        /** Lens position relative to robot centre: + left. */
        public final double camLeftIn;
        /** Lens height above the floor tiles. */
        public final double camHeightIn;
        /** Upward tilt of the optical axis from horizontal. */
        public final double camPitchDeg;

        public CameraPose(double robotXIn, double robotYIn, double robotHeadingDeg,
                          double camForwardIn, double camLeftIn,
                          double camHeightIn, double camPitchDeg) {
            requireFinite(robotXIn, "robotX");
            requireFinite(robotYIn, "robotY");
            requireFinite(robotHeadingDeg, "robotHeading");
            requireFinite(camForwardIn, "camForward");
            requireFinite(camLeftIn, "camLeft");
            requireFinite(camHeightIn, "camHeight");
            requireFinite(camPitchDeg, "camPitch");
            this.robotXIn = robotXIn;
            this.robotYIn = robotYIn;
            this.robotHeadingDeg = robotHeadingDeg;
            this.camForwardIn = camForwardIn;
            this.camLeftIn = camLeftIn;
            this.camHeightIn = camHeightIn;
            this.camPitchDeg = camPitchDeg;
        }

        public double cameraFieldXIn() {
            double h = Math.toRadians(robotHeadingDeg);
            return robotXIn + camForwardIn * Math.cos(h) - camLeftIn * Math.sin(h);
        }

        public double cameraFieldYIn() {
            double h = Math.toRadians(robotHeadingDeg);
            return robotYIn + camForwardIn * Math.sin(h) + camLeftIn * Math.cos(h);
        }

        private static void requireFinite(double value, String name) {
            if (Double.isNaN(value) || Double.isInfinite(value)) {
                throw new IllegalArgumentException(name + " must be finite, was " + value);
            }
        }
    }

    /** One hive tag located in the field frame. */
    public static final class Tag {
        public final int id;
        /** Tag centre height above the floor tiles. */
        public final double heightIn;
        /** Distance from the lens to the tag, projected onto the floor. */
        public final double horizontalDistanceIn;
        public final double fieldXIn;
        public final double fieldYIn;
        public final boolean isAboveThreshold;
        /** Below the threshold by a safe margin, so a lowered cell and not a mis-measured raised one. */
        public final boolean isClearlyBelowThreshold;
        /** False when the field position can't be this tag's cell; usually a wrong robot pose. */
        public final boolean isPositionConsistent;
        /** Image corners x0,y0..x3,y3 in px for drawing; null when not from a camera. */
        final float[] cornersPx;

        private Tag(int id, double heightIn, double horizontalDistanceIn, double fieldXIn, double fieldYIn,
                    boolean isAboveThreshold, boolean isClearlyBelowThreshold,
                    boolean isPositionConsistent, float[] cornersPx) {
            this.id = id;
            this.heightIn = heightIn;
            this.horizontalDistanceIn = horizontalDistanceIn;
            this.fieldXIn = fieldXIn;
            this.fieldYIn = fieldYIn;
            this.isAboveThreshold = isAboveThreshold;
            this.isClearlyBelowThreshold = isClearlyBelowThreshold;
            this.isPositionConsistent = isPositionConsistent;
            this.cornersPx = cornersPx == null ? null : cornersPx.clone();
        }
    }

    /**
     * Immutable vision state: the latest frame's tags plus the last decided tip of each hive,
     * so the answer survives frames where no tags are visible.
     */
    public static final class Snapshot {
        private static final long NEVER = Long.MIN_VALUE;
        public static final Snapshot EMPTY = new Snapshot(
                Collections.<Tag>emptyList(), 0, null, NEVER, null, NEVER);

        /** Hive tags seen in the latest frame (unmodifiable). */
        public final List<Tag> tags;
        public final long timeNanos;
        private final Boolean redRightTipped;
        private final long redSeenNanos;
        private final Boolean blueRightTipped;
        private final long blueSeenNanos;

        private Snapshot(List<Tag> tags, long timeNanos, Boolean redRightTipped, long redSeenNanos,
                         Boolean blueRightTipped, long blueSeenNanos) {
            this.tags = Collections.unmodifiableList(new ArrayList<>(tags));
            this.timeNanos = timeNanos;
            this.redRightTipped = redRightTipped;
            this.redSeenNanos = redSeenNanos;
            this.blueRightTipped = blueRightTipped;
            this.blueSeenNanos = blueSeenNanos;
        }

        /** Returns a new snapshot for this frame; a hive the frame can't decide keeps its old value. */
        public Snapshot next(List<Tag> frameTags, long nowNanos) {
            Boolean red = voteRightTipped(frameTags, Hive.RED);
            Boolean blue = voteRightTipped(frameTags, Hive.BLUE);
            return new Snapshot(frameTags, nowNanos,
                    red != null ? red : redRightTipped, red != null ? nowNanos : redSeenNanos,
                    blue != null ? blue : blueRightTipped, blue != null ? nowNanos : blueSeenNanos);
        }

        /** Last decided tip: true = right side up (start config), false = left, null = never seen. */
        public Boolean isRightTipped(Hive hive) {
            return hive == Hive.RED ? redRightTipped : blueRightTipped;
        }

        /** True if this hive's tip was decided from the latest frame. */
        public boolean isLive(Hive hive) {
            return isRightTipped(hive) != null && seenNanos(hive) == timeNanos;
        }

        /** Seconds since this hive's tip was last decided; NaN if never. */
        public double ageSeconds(Hive hive, long nowNanos) {
            long seen = seenNanos(hive);
            return seen == NEVER ? Double.NaN : (nowNanos - seen) / 1e9;
        }

        /** "live", "last, 3.2s ago" or "never seen". */
        public String sourceLabel(Hive hive, long nowNanos) {
            if (isLive(hive)) {
                return "live";
            }
            if (isRightTipped(hive) == null) {
                return "never seen";
            }
            return String.format(Locale.US, "last, %.1fs ago", ageSeconds(hive, nowNanos));
        }

        private long seenNanos(Hive hive) {
            return hive == Hive.RED ? redSeenNanos : blueSeenNanos;
        }
    }

    public static boolean isHiveTag(int id) {
        return id >= FIRST_TAG_ID && id <= LAST_TAG_ID;
    }

    public static Hive hiveOf(int id) {
        requireHiveTag(id);
        return id < BLUE_AUDIENCE_FIRST ? Hive.RED : Hive.BLUE;
    }

    /** True if the tag is on the cell that is raised at the start of a match. */
    public static boolean isOnRightCell(int id) {
        requireHiveTag(id);
        boolean isRedAudience = id >= RED_AUDIENCE_FIRST && id < BLUE_AUDIENCE_FIRST;
        boolean isBlueScoring = id >= BLUE_SCORING_FIRST;
        return isRedAudience || isBlueScoring;
    }

    /** +1 if the tag's cell is on the audience (+X) side of its hive, -1 otherwise. */
    public static int cellFieldXSign(int id) {
        requireHiveTag(id);
        boolean isAudienceCell = id >= RED_AUDIENCE_FIRST && id < BLUE_SCORING_FIRST;
        return isAudienceCell ? 1 : -1;
    }

    public static String cellName(int id) {
        String side = cellFieldXSign(id) > 0 ? "AUD" : "SCR";
        String hand = isOnRightCell(id) ? "R" : "L";
        return hiveOf(id) + "-" + side + "/" + hand;
    }

    /**
     * Locates one hive tag from its ftcPose (inches, camera frame).
     *
     * @param cornersPx image corners for drawing, or null
     */
    public static Tag estimateTag(int id, double xIn, double yIn, double zIn, float[] cornersPx,
                                  CameraPose pose, double thresholdIn) {
        double pitch = Math.toRadians(pose.camPitchDeg);
        double height = pose.camHeightIn + yIn * Math.sin(pitch) + zIn * Math.cos(pitch);
        double forward = yIn * Math.cos(pitch) - zIn * Math.sin(pitch);

        double heading = Math.toRadians(pose.robotHeadingDeg);
        double fieldX = pose.cameraFieldXIn() + forward * Math.cos(heading) + xIn * Math.sin(heading);
        double fieldY = pose.cameraFieldYIn() + forward * Math.sin(heading) - xIn * Math.cos(heading);

        boolean isOnExpectedXSide = fieldX * cellFieldXSign(id) > 0;
        boolean isNearHive = Math.abs(fieldY - hiveOf(id).fieldYIn) <= MAX_HIVE_Y_ERROR_IN;

        return new Tag(id, height, forward, fieldX, fieldY,
                height > thresholdIn, height < thresholdIn - LOW_TAG_MARGIN_IN,
                isOnExpectedXSide && isNearHive, cornersPx);
    }

    /**
     * Tags above the threshold decide. If none of this hive's tags are above it, a tag well
     * below it (a lowered cell) still tells us the opposite cell is raised. Ties are null.
     */
    static Boolean voteRightTipped(List<Tag> tags, Hive hive) {
        int highRight = 0;
        int highLeft = 0;
        int lowRight = 0;
        int lowLeft = 0;
        for (Tag tag : tags) {
            if (hiveOf(tag.id) != hive) {
                continue;
            }
            boolean isRight = isOnRightCell(tag.id);
            if (tag.isAboveThreshold) {
                if (isRight) highRight++; else highLeft++;
            } else if (tag.isClearlyBelowThreshold) {
                if (isRight) lowRight++; else lowLeft++;
            }
        }
        if (highRight + highLeft > 0) {
            return majority(highRight, highLeft);
        }
        return majority(lowLeft, lowRight);
    }

    private static Boolean majority(int rightUpVotes, int leftUpVotes) {
        if (rightUpVotes > leftUpVotes) return Boolean.TRUE;
        if (leftUpVotes > rightUpVotes) return Boolean.FALSE;
        return null;
    }

    private static void requireHiveTag(int id) {
        if (!isHiveTag(id)) {
            throw new IllegalArgumentException("Not a hive tag id: " + id);
        }
    }
}
