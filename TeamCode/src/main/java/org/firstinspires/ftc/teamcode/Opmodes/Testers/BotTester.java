package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Bot Tester", group = "Testers")
public class BotTester extends LinearOpMode {

    // =========================
    // DASHBOARD CONFIG
    // =========================

    public static double INTAKE_POWER = 0.0;
    public static double TRANSFER_POWER = 0.0;
    public static double SHOOTER_1_POWER = 0.0;
    public static double SHOOTER_2_POWER = 0.0;

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

    private Servo hood;

    private Servo flower1;
    private Servo flower2;

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

        hood = hardwareMap.get(Servo.class, "hood");

        flower1 = hardwareMap.get(Servo.class, "flower1");
        flower2 = hardwareMap.get(Servo.class, "flower2");

        // =========================
        // BRAKE BEHAVIOR
        // =========================

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // =========================
        // MOTOR DIRECTIONS
        // =========================

        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
            shooterMotor1.setPower(SHOOTER_1_POWER);
            shooterMotor2.setPower(SHOOTER_2_POWER);

            // Turret Servos
            turret1.setPosition(TURRET_SERVO_POWER);
            turret2.setPosition(TURRET_SERVO_POWER);

            // Hood Servo
            hood.setPosition(HOOD_SERVO_POWER);

            // Flower Servos
            flower1.setPosition(FLOWER_SERVO_POWER);
            flower2.setPosition(FLOWER_SERVO_POWER);

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData("Intake Power", INTAKE_POWER);
            telemetry.addData("Transfer Power", TRANSFER_POWER);
            telemetry.addData("Shooter 1 Power", SHOOTER_1_POWER);
            telemetry.addData("Shooter 2 Power", SHOOTER_2_POWER);

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