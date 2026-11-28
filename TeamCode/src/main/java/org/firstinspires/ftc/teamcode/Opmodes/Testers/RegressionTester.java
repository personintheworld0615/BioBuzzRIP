package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

@Config
@TeleOp(name = "Regression Tester", group = "Testers")
public class RegressionTester extends LinearOpMode {

    // =========================
    // DASHBOARD CONFIG
    // =========================

    public static double INTAKE_POWER = 0.0;
    public static double TRANSFER_POWER = 0.0;
    public static double shooterRPM = 0.0;

    public static double HOOD_SERVO_POWER = 0.6;
    public static double realRPM = 0;

    // =========================
    // MOTORS
    // =========================

    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    // =========================
    // SERVOS
    // =========================

    private Servo turret1;
    private Turret turret;

//    private CRServo hood;

    private Servo flower1;
    private Shooter shooter;


    @Override
    public void runOpMode() {

        // =========================
        // HARDWARE MAPPING
        // =========================
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        turret1 = hardwareMap.get(Servo.class, "turret1");

//        hood = hardwareMap.get(CRServo.class, "hood");

        flower1 = hardwareMap.get(Servo.class, "flower1");

        // =========================
        // BRAKE BEHAVIOR
        // =========================
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72,72,0));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter = new Shooter(hardwareMap,follower,false);
        Turret turret = new Turret(follower,hardwareMap,false);

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        hood.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Bot Tester Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        // =========================
        // MAIN LOOP
        // =========================

        while (opModeIsActive()) {
            follower.update();

            // Motors
            intakeMotor.setPower(INTAKE_POWER);
            transferMotor.setPower(TRANSFER_POWER);

            shooter.spinToRpm(shooterRPM);
            // Turret Servos
            turret.targetToGoal();

            // Hood Servo
//            hood.setPower(HOOD_SERVO_POWER);


            telemetry.addData("Intake Power", INTAKE_POWER);
            telemetry.addData("Transfer Power", TRANSFER_POWER);
            telemetry.addData("Shooter  rpm", shooterRPM);
            telemetry.addData("Robot Distance",shooter.getDitance());
            telemetry.update();
            realRPM = shooter.getRPM();
            telemetry.addData("ahufa", realRPM);
        }

    }
}