
package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "drivetrain Tester", group = "Testers")
public class DrivetrainTester extends LinearOpMode {

    // =========================
    // DASHBOARD CONFIG
    // =========================

    public static double fr = 0.0;
    public static double fl = 0.0;
    public static double br = 0.0;
    public static double bl = 0.0;

    // =========================
    // MOTORS
    // =========================

    private DcMotor frmotor;
    private DcMotor flmotor;
    private DcMotor brmotor;
    private DcMotor blmotor;

    @Override
    public void runOpMode() {

        // Hardware mapping
        flmotor = hardwareMap.get(DcMotor.class, "fl");
        frmotor = hardwareMap.get(DcMotor.class, "fr");
        brmotor = hardwareMap.get(DcMotor.class, "br");
        blmotor = hardwareMap.get(DcMotor.class, "bl");

        // Brake behavior// Motor directions
        //THESE ARE THE CORRECT ONES
        flmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        brmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        blmotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Bot Tester Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Intake
            flmotor.setPower(fl);
            frmotor.setPower(fr);
            brmotor.setPower(br);
            blmotor.setPower(bl);

            // Transfer
            // Telemetry
        }

        // Stop motors
    }
}
