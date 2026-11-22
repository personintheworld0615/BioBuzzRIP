
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

    // =========================
    // MOTORS
    // =========================

    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    @Override
    public void runOpMode() {

        // Hardware mapping
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");

        // Brake behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Bot Tester Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Intake
            intakeMotor.setPower(INTAKE_POWER);

            // Transfer
            transferMotor.setPower(TRANSFER_POWER);

            // Telemetry
            telemetry.addData("Intake Power", INTAKE_POWER);
            telemetry.addData("Transfer Power", TRANSFER_POWER);

            telemetry.update();
        }

        // Stop motors
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
    }
}
