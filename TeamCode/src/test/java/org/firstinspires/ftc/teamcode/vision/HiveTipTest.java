package org.firstinspires.ftc.teamcode.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.vision.HiveTip.CameraPose;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Hive;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Snapshot;
import org.firstinspires.ftc.teamcode.vision.HiveTip.Tag;
import org.junit.Test;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class HiveTipTest {

    private static final double EPS = 1e-6;
    private static final double THRESHOLD = 45.0;
    private static final long ONE_SECOND_NS = 1_000_000_000L;

    /** Robot on the red side of the field facing +Y (toward the red hive), camera level. */
    private static final CameraPose LEVEL_FACING_RED_HIVE =
            new CameraPose(0, -48, 90, 0, 0, 12, 0);

    private static Tag tag(int id, double x, double y, double z) {
        return tag(id, x, y, z, LEVEL_FACING_RED_HIVE);
    }

    private static Tag tag(int id, double x, double y, double z, CameraPose pose) {
        return HiveTip.estimateTag(id, x, y, z, null, pose, THRESHOLD);
    }

    private static Snapshot frame(Tag... tags) {
        return Snapshot.EMPTY.next(Arrays.asList(tags), 0);
    }

    // ---- Tag layout ----

    @Test
    public void mapsIdsToHivesPerCompetitionManual() {
        for (int id = 30; id <= 37; id++) {
            assertEquals(Hive.RED, HiveTip.hiveOf(id));
        }
        for (int id = 38; id <= 45; id++) {
            assertEquals(Hive.BLUE, HiveTip.hiveOf(id));
        }
    }

    @Test
    public void rightCellsAreTheOnesRaisedInTheStartingConfiguration() {
        // Red audience cell (34-37) and blue scoring cell (42-45) start raised.
        assertTrue(HiveTip.isOnRightCell(34));
        assertTrue(HiveTip.isOnRightCell(37));
        assertTrue(HiveTip.isOnRightCell(42));
        assertTrue(HiveTip.isOnRightCell(45));
        assertFalse(HiveTip.isOnRightCell(30));
        assertFalse(HiveTip.isOnRightCell(33));
        assertFalse(HiveTip.isOnRightCell(38));
        assertFalse(HiveTip.isOnRightCell(41));
    }

    @Test
    public void audienceCellsAreOnPositiveFieldX() {
        assertEquals(1, HiveTip.cellFieldXSign(34));
        assertEquals(1, HiveTip.cellFieldXSign(41));
        assertEquals(-1, HiveTip.cellFieldXSign(30));
        assertEquals(-1, HiveTip.cellFieldXSign(45));
    }

    @Test
    public void recognisesOnlyHiveTagIds() {
        assertTrue(HiveTip.isHiveTag(30));
        assertTrue(HiveTip.isHiveTag(45));
        assertFalse(HiveTip.isHiveTag(29));
        assertFalse(HiveTip.isHiveTag(46));
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsUnknownIds() {
        HiveTip.hiveOf(20);
    }

    @Test(expected = IllegalArgumentException.class)
    public void estimateRejectsNonHiveTags() {
        tag(20, 0, 35, 37);
    }

    @Test
    public void nearestHiveFollowsRobotFieldY() {
        assertEquals(Hive.RED, Hive.nearestTo(-48));
        assertEquals(Hive.BLUE, Hive.nearestTo(48));
    }

    // ---- Geometry ----

    @Test
    public void heightIsCameraHeightPlusVerticalOffsetWhenCameraIsLevel() {
        Tag tag = tag(35, 0, 30, 10);

        assertEquals(22.0, tag.heightIn, EPS);
        assertEquals(30.0, tag.horizontalDistanceIn, EPS);
    }

    @Test
    public void heightAccountsForCameraPitch() {
        CameraPose pitchedUp30 = new CameraPose(0, -48, 90, 0, 0, 12, 30);

        // Tag sits exactly on the optical axis 40 in away: rises 40*sin(30) = 20 in.
        Tag onAxis = tag(35, 0, 40, 0, pitchedUp30);
        // Tag 10 in "up" in the image adds 10*cos(30) and pulls it 10*sin(30) closer.
        Tag aboveAxis = tag(35, 0, 40, 10, pitchedUp30);

        assertEquals(32.0, onAxis.heightIn, EPS);
        assertEquals(40 * Math.cos(Math.toRadians(30)), onAxis.horizontalDistanceIn, EPS);
        assertEquals(32.0 + 10 * Math.cos(Math.toRadians(30)), aboveAxis.heightIn, EPS);
        assertEquals(40 * Math.cos(Math.toRadians(30)) - 10 * Math.sin(Math.toRadians(30)),
                aboveAxis.horizontalDistanceIn, EPS);
    }

    @Test
    public void classifiesAgainstThresholdMeasuredFromTheGround() {
        assertTrue(tag(35, 0, 30, 37).isAboveThreshold);    // 49 in
        assertFalse(tag(31, 0, 30, 23).isAboveThreshold);   // 35 in
    }

    @Test
    public void tagExactlyAtThresholdIsNotAbove() {
        Tag tag = tag(35, 0, 30, 33);

        assertEquals(45.0, tag.heightIn, EPS);
        assertFalse(tag.isAboveThreshold);
    }

    @Test
    public void projectsTagIntoFieldCoordinatesUsingRobotPose() {
        // Facing +Y, camera "right" is field +X (the audience side).
        Tag tag = tag(35, 13, 35, 37);

        assertEquals(13.0, tag.fieldXIn, EPS);
        assertEquals(-13.0, tag.fieldYIn, EPS);
        assertTrue(tag.isPositionConsistent);
    }

    @Test
    public void flagsTagWhoseFieldPositionDisagreesWithItsCell() {
        // Same detection as above, but tag 31 belongs to the cell on the -X side.
        assertFalse(tag(31, 13, 35, 37).isPositionConsistent);
    }

    @Test
    public void projectsCorrectlyWhenFacingAudience() {
        // Robot on the -X side facing +X: camera right is field -Y.
        Tag tag = tag(31, 2, 27, 37, new CameraPose(-40, -12.75, 0, 0, 0, 12, 0));

        assertEquals(-13.0, tag.fieldXIn, EPS);
        assertEquals(-14.75, tag.fieldYIn, EPS);
        assertTrue(tag.isPositionConsistent);
    }

    @Test
    public void projectsCorrectlyWhenFacingAwayFromAudience() {
        // Robot on the +X side facing -X: camera right is field +Y.
        Tag tag = tag(39, 2, 27, 37, new CameraPose(40, 12.75, 180, 0, 0, 12, 0));

        assertEquals(13.0, tag.fieldXIn, EPS);
        assertEquals(14.75, tag.fieldYIn, EPS);
        assertTrue(tag.isPositionConsistent);
    }

    @Test
    public void appliesCameraMountOffsetsInRobotFrame() {
        // Robot at origin facing +X; camera 5 in forward and 2 in left of robot centre.
        CameraPose pose = new CameraPose(0, 0, 0, 5, 2, 12, 0);

        assertEquals(5.0, pose.cameraFieldXIn(), EPS);
        assertEquals(2.0, pose.cameraFieldYIn(), EPS);
    }

    @Test
    public void appliesCameraLeftOffsetWhenRotated() {
        // Facing +Y, "left" of the robot is field -X.
        CameraPose pose = new CameraPose(0, 0, 90, 5, 2, 12, 0);

        assertEquals(-2.0, pose.cameraFieldXIn(), EPS);
        assertEquals(5.0, pose.cameraFieldYIn(), EPS);
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsNonFiniteCameraPose() {
        new CameraPose(Double.NaN, 0, 0, 0, 0, 12, 30);
    }

    // ---- Tip vote ----

    @Test
    public void highTagsOnRightCellMeanRightTippedUp() {
        Snapshot s = frame(tag(34, 10, 35, 37), tag(35, 13, 35, 37));

        assertEquals(Boolean.TRUE, s.isRightTipped(Hive.RED));
        assertNull(s.isRightTipped(Hive.BLUE));
    }

    @Test
    public void highTagsOnLeftCellMeanLeftTippedUp() {
        assertEquals(Boolean.FALSE, frame(tag(40, 0, 35, 37)).isRightTipped(Hive.BLUE));
    }

    @Test
    public void fallsBackToLowTagsWhenNoTagCrossesThreshold() {
        // Only the lowered red scoring cell (30-33) is visible -> the other side is up.
        assertEquals(Boolean.TRUE, frame(tag(31, 0, 35, 23)).isRightTipped(Hive.RED));
    }

    @Test
    public void loweredRightCellAloneMeansLeftTippedUp() {
        assertEquals(Boolean.FALSE, frame(tag(43, 0, 35, 23)).isRightTipped(Hive.BLUE));
    }

    @Test
    public void lowTagsJustUnderThresholdDoNotVote() {
        // 43 in: below 45 but inside the fallback margin, so it could be a raised tag
        // measured slightly low (wrong pitch/height) - must not flip the answer.
        Snapshot s = frame(tag(31, 0, 35, 31));

        assertFalse(s.tags.get(0).isAboveThreshold);
        assertNull(s.isRightTipped(Hive.RED));
    }

    @Test
    public void highTagsOutrankContradictingLowTags() {
        Snapshot s = frame(
                tag(30, 0, 35, 37),   // high, left cell
                tag(34, 0, 35, 23),   // low, right cell
                tag(35, 0, 35, 23));  // low, right cell

        assertEquals(Boolean.FALSE, s.isRightTipped(Hive.RED));
    }

    @Test
    public void conflictingEvidenceIsUnknown() {
        assertNull(frame(tag(30, 0, 35, 37), tag(34, 0, 35, 37)).isRightTipped(Hive.RED));
    }

    @Test
    public void emptyFrameIsUnknownForBothHives() {
        Snapshot s = frame();

        assertNull(s.isRightTipped(Hive.RED));
        assertNull(s.isRightTipped(Hive.BLUE));
        assertEquals("never seen", s.sourceLabel(Hive.RED, 0));
    }

    @Test
    public void tagListIsImmutable() {
        List<Tag> tags = frame(tag(35, 0, 35, 37)).tags;

        try {
            tags.clear();
        } catch (UnsupportedOperationException expected) {
            return;
        }
        throw new AssertionError("Snapshot.tags must be unmodifiable");
    }

    // ---- Memory across frames ----

    @Test
    public void startsEmpty() {
        assertNull(Snapshot.EMPTY.isRightTipped(Hive.RED));
        assertFalse(Snapshot.EMPTY.isLive(Hive.RED));
    }

    @Test
    public void liveFrameIsReportedAsLive() {
        Snapshot s = frame(tag(35, 0, 35, 37));

        assertTrue(s.isLive(Hive.RED));
        assertEquals("live", s.sourceLabel(Hive.RED, 0));
    }

    @Test
    public void keepsLastKnownTipWhenNoTagsAreVisible() {
        Snapshot blind = frame(tag(30, 0, 35, 37))
                .next(Collections.<Tag>emptyList(), 3 * ONE_SECOND_NS);

        assertFalse(blind.isLive(Hive.RED));
        assertEquals(Boolean.FALSE, blind.isRightTipped(Hive.RED));
        assertEquals(3.0, blind.ageSeconds(Hive.RED, 3 * ONE_SECOND_NS), 1e-9);
        assertEquals("last, 3.0s ago", blind.sourceLabel(Hive.RED, 3 * ONE_SECOND_NS));
        assertNull(blind.isRightTipped(Hive.BLUE));
    }

    @Test
    public void nextDoesNotMutateOriginal() {
        Snapshot.EMPTY.next(Collections.singletonList(tag(35, 0, 35, 37)), 0);

        assertNull(Snapshot.EMPTY.isRightTipped(Hive.RED));
    }

    @Test
    public void hivesAreRememberedIndependently() {
        Snapshot s = frame(tag(35, 0, 35, 37))
                .next(Collections.singletonList(tag(40, 0, 35, 37)), ONE_SECOND_NS);

        assertEquals(Boolean.TRUE, s.isRightTipped(Hive.RED));
        assertEquals(Boolean.FALSE, s.isRightTipped(Hive.BLUE));
        assertFalse(s.isLive(Hive.RED));
        assertTrue(s.isLive(Hive.BLUE));
        assertEquals(2.0, s.ageSeconds(Hive.RED, 2 * ONE_SECOND_NS), 1e-9);
        assertEquals(1.0, s.ageSeconds(Hive.BLUE, 2 * ONE_SECOND_NS), 1e-9);
    }
}
