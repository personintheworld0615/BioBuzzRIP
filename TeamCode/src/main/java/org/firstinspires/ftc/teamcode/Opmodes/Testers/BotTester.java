package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@TeleOp(name = "Bot Tester", group = "Testers")
public class BotTester extends LinearOpMode {

    // =========================
    // DASHBOARD CONFIG
    // =========================

    public static double INTAKE_POWER = 0.0;
    public static double TRANSFER_POWER = 0.0;
    public static double shooterRPM = 0.0;
    public static double shootermotorpower = 0.0;
    public static double shootermotorpower2 = 0.0;


    public static double TURRET_SERVO_POWER = 0.5;
    public static double HOOD_SERVO_POWER = 0.5;
    public static double FLOWER_SERVO_POWER = 0.5;

    // =========================
    // MOTORS
    // =========================

    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;

    // =========================
    // SERVOS
    // =========================

    private Servo turret1;
    private Servo turret2;

    private CRServo hood;

    private Servo flower1;
    private Shooter shooter;

    @Override
    public void runOpMode() {

        // =========================
        // HARDWARE MAPPING
        // =========================

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        shooterMotor1 = hardwareMap.get(DcMotor.class, "Shooter1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "Shooter2");

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

//        hood = hardwareMap.get(CRServo.class, "hood");

        flower1 = hardwareMap.get(Servo.class, "flower1");

        // =========================
        // BRAKE BEHAVIOR
        // =========================
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72,72,0));
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter = new Shooter(hardwareMap,follower,true);

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Bot Tester Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        // =========================
        // MAIN LOOP
        // =========================

        while (opModeIsActive()) {

            // Motors
            intakeMotor.setPower(INTAKE_POWER);
            transferMotor.setPower(TRANSFER_POWER);
//            shooterMotor1.setPower(shootermotorpower);
//            shooterMotor2.setPower(shootermotorpower2);

            shooter.spinToRpm(shooterRPM);
            // Turret Servos
            turret1.setPosition(TURRET_SERVO_POWER);
//            turret2.setPosition(TURRET_SERVO_POWER);

            // Hood Servo
//            hood.setPosition(HOOD_SERVO_POWER);

            // Flower Servos
            flower1.setPosition(FLOWER_SERVO_POWER);

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData("Intake Power", INTAKE_POWER);
            telemetry.addData("Transfer Power", TRANSFER_POWER);
            telemetry.addData("Shooter 1 rpm", shooterRPM);
            telemetry.addData("Shooter 1 power", shooterMotor1.getPower());
            telemetry.addData("Shooter 2 power", shooterMotor2.getPower());

            telemetry.addData("Turret Position", TURRET_SERVO_POWER);
            telemetry.addData("Hood Position", HOOD_SERVO_POWER);
            telemetry.addData("Flower Position", FLOWER_SERVO_POWER);

            telemetry.update();
        }

        // =========================
        // STOP MOTORS
        // =========================

        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }
}