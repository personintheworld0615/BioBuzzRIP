package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.MyTelem;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Camera;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

import java.util.Locale;

@Config
@Utility(name = "Shooter Turret Tester")
public class ShooterTest extends LinearOpMode {

    // Pick the test on Dashboard. STATE 0 lists what each one does.
    public static int STATE = 0;

    // Read when START is pressed.
    public static double START_X = 72;
    public static double START_Y = 72;
    public static double START_HEADING_DEG = 90;
    public static boolean RED_ALLIANCE = false;

    public static double INTAKE_POWER = 0.0;
    public static double TRANSFER_POWER = 0.0;
    public static double SHOOTER_RPM = 0.0;

    // STATE 5 only: tick to save SHOOTER_RPM at the current distance; it unticks itself.
    public static boolean SAVE_POINT = false;

    private Robot robot;
    private Follower follower;
    private Shooter shooter;
    private Turret turret;
    private Camera camera;

    // Commands each state may run. Leaving a state cancels them all.
    private Command drive;
    private Command feed;
    private Command shooterManual;
    private Command shooterRecord;
    private Command shooterLookup;
    private Command turretAim;
    private Command turretAimWithVision;
    private Command tipDetection;
    private Command[] stateCommands;
    private int activeState = -1;
    private String lastSave = "None yet";

    @Override
    public void runOpMode() {
        MyTelem.init(telemetry);
        Scheduler.reset();

        // Creating the follower also sets the drive motor directions from Constants.
        follower = Constants.createFollower(hardwareMap);
        robot = new Robot(hardwareMap, follower, gamepad1, gamepad2);
        shooter = robot.shooter;
        turret = robot.turret;
        // we need to program the camera into the robot class but we'll do that sunday night
        camera = new Camera(hardwareMap, RED_ALLIANCE);
        buildCommands();

        MyTelem.addLine("Set the start psoe & RED_ALLIANCE boolean");
        MyTelem.addLine("switch into 0");
        MyTelem.update();

        waitForStart();

        camera.setRedAlliance(RED_ALLIANCE);
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        // These run in every state and are scheduled first, so the pose and camera are fresh
        // before the state's commands read them each loop. Only the localizer updates: an idle
        // follower.update() would stop the drive motors that Robot's drive command is setting.
        Scheduler.schedule(
                Command.build().setExecute(follower.localizer::update),
                camera.updateCommand());

        while (opModeIsActive()) {
            if (STATE != activeState) {
                cancelStateCommands();
                activeState = STATE;
            }
            handleSavePoint();
            MyTelem.addData("STATE", STATE);
            camera.setRedAlliance(RED_ALLIANCE);

            switch (STATE) {
                case 0:
                    MyTelem.addLine("Nothing runs in STATE 0. Set STATE on Dashboard:");
                    MyTelem.addLine("1  Shooter PID: spins to SHOOTER_RPM, tune kP kI kD kF under Constants.");
                    MyTelem.addLine("2  Turret: drive, the turret aims from odometry. Tune Turret SERVO_* and the hive position under Constants.");
                    MyTelem.addLine("3  Turret + camera: like 2, the camera trims the aim. Tune Turret VISION_* and Camera TARGET_TIMEOUT_MS.");
                    MyTelem.addLine("4  Hive tip: like 3, plus tip detection switches the goal from the starting CELL to the secondary CELL. Tune Camera TIP_*.");
                    MyTelem.addLine("5  Lookup table: drive to a spot, raise SHOOTER_RPM until shots score, tick SAVE_POINT. SHOOTER_RPM 0 plays the table back.");
                    MyTelem.addLine("6  Everything: drive, aim with camera, tip detection, lookup RPM, feed powers.");
                    MyTelem.addLine("INTAKE_POWER and TRANSFER_POWER run in 1, 5 and 6.");
                    MyTelem.addLine("Each state lists the values to copy into code before you stop.");
                    break;

                case 1:
                    shooter.setPIDFConstants(Constants.kP, Constants.kI, Constants.kD, Constants.kF);
                    run(shooterManual, feed);
                    MyTelem.addLine("Shooter PID Tune kP kI kD kF under Constants.");
                    addShooterTelemetry();
                    addCopyBeforeStopping(pidLine());
                    break;

                case 2:
                    turret.setGoal(hiveGoal());
                    run(drive, turretAim);
                    MyTelem.addLine("Turret: drive with gamepad1, the turret aims at the hive from odometry.");
                    addPoseTelemetry();
                    addTurretTelemetry();
                    addCopyBeforeStopping(turretLine(), fieldLine());
                    break;

                case 3: // Turret + camera
                    turret.setGoal(hiveGoal());
                    run(drive, turretAimWithVision);
                    MyTelem.addLine("Turret + camera: odometry aims, the camera trims out the error.");
                    addPoseTelemetry();
                    addTurretTelemetry();
                    addCameraTelemetry();
                    addCopyBeforeStopping(visionLine(), cameraLine());
                    break;

                case 4: // Turret + camera + hive tip detection
                    turret.setGoal(hiveGoal());
                    run(drive, tipDetection, turretAimWithVision);
                    MyTelem.addLine("Hive tip: like 3, and a tip swaps the goal between the starting and secondary CELL.");
                    addPoseTelemetry();
                    addTurretTelemetry();
                    addCameraTelemetry();
                    addTipTelemetry();
                    addCopyBeforeStopping(tipLine());
                    break;

                case 5: // Lookup table
                    shooter.setPIDFConstants(Constants.kP, Constants.kI, Constants.kD, Constants.kF);
                    turret.setGoal(hiveGoal());
                    run(drive, feed, turretAimWithVision, shooterRecord);
                    MyTelem.addLine(SHOOTER_RPM > 0
                            ? " dedas jst click save ont when shooter is consistent"
                            : "Lookup PLAYBACK: SHOOTER_RPM is 0, so the shooter follows the table.");
                    addPoseTelemetry();
                    addShooterTelemetry();
                    addLookupTelemetry();
                    addCopyBeforeStopping(lookupLines());
                    break;

                case 6: // Everything
                    shooter.setPIDFConstants(Constants.kP, Constants.kI, Constants.kD, Constants.kF);
                    turret.setGoal(hiveGoal());
                    run(drive, feed, tipDetection, turretAimWithVision, shooterLookup);
                    MyTelem.addLine("Everything: drive, aim with camera, tip detection, lookup RPM, feed powers.");
                    addPoseTelemetry();
                    addShooterTelemetry();
                    addTurretTelemetry();
                    addCameraTelemetry();
                    addTipTelemetry();
                    addLookupTelemetry();
                    addCopyBeforeStopping(pidLine(), turretLine(), fieldLine(), visionLine(),
                            cameraLine(), tipLine(), lookupLines()[0], lookupLines()[1]);
                    break;

                default:
                    MyTelem.addLine("No STATE " + STATE + ", pick 0 to 6.");
                    break;
            }

            Scheduler.execute();
            MyTelem.update();
        }

        cancelStateCommands();
        Scheduler.reset();
        camera.close();
    }

    private void buildCommands() {
        drive = robot.getDriveCommand();
        feed = robot.intake.powerCommand(INTAKE_POWER, TRANSFER_POWER);
        shooterManual = shooter.spinToRpmCommand(() -> SHOOTER_RPM);
        shooterRecord = shooter.spinToRpmCommand(
                () -> SHOOTER_RPM > 0 ? SHOOTER_RPM : shooter.distanceToRPM(distanceToHive()));
        shooterLookup = shooter.lookupCommand(this::distanceToHive);
        turretAim = turret.aimCommand();
        turretAimWithVision = turret.aimWithVisionCommand(camera);
        tipDetection = camera.tipDetectionCommand();

        stateCommands = new Command[] {drive, feed, shooterManual, shooterRecord, shooterLookup,
                turretAim, turretAimWithVision, tipDetection};
    }

    // Starts any of the commands that aren't already running.
    private void run(Command... commands) {
        for (Command command : commands) {
            if (!command.isScheduled()) {
                command.schedule();
            }
        }
    }

    // Cancelling runs each command's end, which stops the motors it was driving. Robot's drive
    // has no end, so the drive motors are stopped here.
    private void cancelStateCommands() {
        for (Command command : stateCommands) {
            command.cancel();
        }
        robot.drivetrain.stop();
    }

    private Pose hiveGoal() {
        double x = RED_ALLIANCE ? Constants.RED_HIVE_X : Constants.BLUE_HIVE_X;
        double y = camera.getTipSide() == Camera.TipSide.STARTING
                ? (RED_ALLIANCE ? Constants.RED_STARTING_CELL_Y : Constants.BLUE_STARTING_CELL_Y)
                : (RED_ALLIANCE ? Constants.RED_SECONDARY_CELL_Y : Constants.BLUE_SECONDARY_CELL_Y);
        return new Pose(x, y);
    }

    private double distanceToHive() {
        Pose pose = follower.pose();
        Pose goal = hiveGoal();
        return Math.hypot(goal.x() - pose.x(), goal.y() - pose.y());
    }

    private void handleSavePoint() {
        if (!SAVE_POINT) {
            return;
        }
        SAVE_POINT = false;
        lastSave = STATE == 5 ? savePoint() : "Not saved: SAVE_POINT only works in STATE 5";
        // Push the unticked box and the new point back to the browser.
        FtcDashboard.getInstance().updateConfig();
    }

    private String savePoint() {
        if (SHOOTER_RPM <= 0) {
            return "RETARD YOUR RPM <= 0 ";
        }
        double distance = Math.round(distanceToHive() * 10) / 10.0;
        int slot = shooter.recordPoint(distance, SHOOTER_RPM);
        if (slot < 0) {
            return "Not saved: table full, set an RPM to 0 under Constants to free a slot";
        }
        return String.format(Locale.US, "Slot %d: %.1f in -> %.0f RPM", slot, distance, SHOOTER_RPM);
    }

    //  THESE ARE THE Telemetry METHODS

    private void addStateGuide() {

    }

    private void addShooterTelemetry() {
        MyTelem.addData("Target RPM", shooter.getTargetRpm());
        MyTelem.addData("RPM Motor 1", shooter.getRPM());
        MyTelem.addData("RPM Motor 2", shooter.getRPM2());
        MyTelem.addData("Error Motor 1", shooter.getTargetRpm() - shooter.getRPM());
        MyTelem.addData("Error Motor 2", shooter.getTargetRpm() - shooter.getRPM2());
    }

    private void addPoseTelemetry() {
        Pose pose = follower.pose();
        Pose goal = hiveGoal();
        MyTelem.addData("Alliance", RED_ALLIANCE ? "RED" : "BLUE");
        MyTelem.addData("Pose", String.format(Locale.US, "(%.1f, %.1f) %.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        MyTelem.addData("Goal", String.format(Locale.US, "(%.1f, %.1f)", goal.x(), goal.y()));
        MyTelem.addData("Distance to Goal (in)", distanceToHive());
    }

    private void addTurretTelemetry() {
        MyTelem.addData("Turret Aim Angle (deg)", turret.getAimAngleDeg());
        MyTelem.addData("Turret Vision Trim (deg)", turret.getVisionTrimDeg());
        MyTelem.addData("Turret Servo Position", turret.getServoPosition());
    }

    private void addCameraTelemetry() {
        MyTelem.addData("Hive Visible", camera.hasHiveTarget());
        if (camera.hasHiveTarget()) {
            MyTelem.addData("Hive Range (in)", camera.getHiveRangeIn());
            MyTelem.addData("Hive Bearing (deg)", camera.getHiveBearingDeg());
            MyTelem.addData("Hive Percent Found", camera.getHivePercentFound());
        }
    }

    private void addTipTelemetry() {
        MyTelem.addData("Goal CELL", camera.getTipSide());
        if (camera.hasHiveTarget()) {
            MyTelem.addData("Hive Pitch (deg)", camera.getHivePitchDeg());
        }
    }

    private void addLookupTelemetry() {
        MyTelem.addData("Table RPM Here", shooter.distanceToRPM(distanceToHive()));
        MyTelem.addData("Last Save", lastSave);
    }
    //making this as idiot proof as possible lmao
    private void addCopyBeforeStopping(String... lines) {
        MyTelem.addLine("");
        MyTelem.addLine("=== COPY INTO CODE BEFORE STOPPING ===");
        for (String line : lines) {
            MyTelem.addLine(line);
        }
    }

    private String pidLine() {
        return "Util/Constants.java: kP = " + Constants.kP + "; kI = " + Constants.kI
                + "; kD = " + Constants.kD + "; kF = " + Constants.kF + ";";
    }

    private String turretLine() {
        return "Turret.java: SERVO_CENTER = " + Turret.SERVO_CENTER
                + "; SERVO_RANGE_DEG = " + Turret.SERVO_RANGE_DEG
                + "; SERVO_REVERSED = " + Turret.SERVO_REVERSED + ";";
    }

    private String fieldLine() {
        return "Util/Constants.java: RED_HIVE_X = " + Constants.RED_HIVE_X
                + "; BLUE_HIVE_X = " + Constants.BLUE_HIVE_X
                + "; RED_STARTING_CELL_Y = " + Constants.RED_STARTING_CELL_Y
                + "; RED_SECONDARY_CELL_Y = " + Constants.RED_SECONDARY_CELL_Y
                + "; BLUE_STARTING_CELL_Y = " + Constants.BLUE_STARTING_CELL_Y
                + "; BLUE_SECONDARY_CELL_Y = " + Constants.BLUE_SECONDARY_CELL_Y + ";";
    }

    private String visionLine() {
        return "Turret.java: VISION_GAIN = " + Turret.VISION_GAIN
                + "; VISION_MAX_TRIM_DEG = " + Turret.VISION_MAX_TRIM_DEG + ";";
    }

    private String cameraLine() {
        return "Camera.java: TARGET_TIMEOUT_MS = " + Camera.TARGET_TIMEOUT_MS + ";";
    }

    private String tipLine() {
        return "Camera.java: TIP_PITCH_THRESHOLD_DEG = " + Camera.TIP_PITCH_THRESHOLD_DEG
                + "; TIP_CONFIRM_MS = " + Camera.TIP_CONFIRM_MS + ";";
    }

    private String[] lookupLines() {
        return new String[] {
                "Util/Constants.java: SHOOTER_DISTANCES = new double[] " + toJavaArray(Constants.SHOOTER_DISTANCES) + ";",
                "Util/Constants.java: SHOOTER_RPMS = new double[] " + toJavaArray(Constants.SHOOTER_RPMS) + ";"
        };
    }

    private static String toJavaArray(double[] values) {
        StringBuilder builder = new StringBuilder("{");
        for (int i = 0; i < values.length; i++) {
            builder.append(i == 0 ? "" : ", ").append(values[i]);
        }
        return builder.append("}").toString();
    }
}
