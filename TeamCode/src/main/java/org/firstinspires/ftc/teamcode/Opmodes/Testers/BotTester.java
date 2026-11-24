
package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    // =========================
    // MOTORS
    // =========================

    private DcMotor intakeMotor;
    private DcMotor transferMotor;
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;

    @Override
    public void runOpMode() {

        // Hardware mapping
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");
        shooterMotor1 = hardwareMap.get(DcMotor.class, "Shooter1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "Shooter2");

        // Brake behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Motor directions
        //THESE ARE THE CORRECT ONES
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Bot Tester Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Intake
            intakeMotor.setPower(INTAKE_POWER);

            // Transfer
            transferMotor.setPower(TRANSFER_POWER);

            // Shooters
            shooterMotor1.setPower(SHOOTER_1_POWER);
            shooterMotor2.setPower(SHOOTER_2_POWER);

            // Telemetry
            telemetry.addData("Intake Power", INTAKE_POWER);
            telemetry.addData("Transfer Power", TRANSFER_POWER);
            telemetry.addData("Shooter 1 Power", SHOOTER_1_POWER);
            telemetry.addData("Shooter 2 Power", SHOOTER_2_POWER);

            telemetry.update();
        }

        // Stop motors
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }
}
