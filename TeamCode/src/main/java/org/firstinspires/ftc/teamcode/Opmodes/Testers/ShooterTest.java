package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.subsystem.Outtake;

@Config
@Utility(name = "4-Motor Subsystem Diagnostic")
public class ShooterTest extends LinearOpMode {

    // Changing these live on FTC Dashboard will immediately update motor power
    public static double INTAKE_POWER = 0.0;
    public static double TRANSFER_POWER = 0.0;
    public static double SHOOTER_RPM = 0.0;

    // References to your physical motors
    private DcMotor intakeMotor;
    private DcMotor transferMotor;

    @Override
    public void runOpMode() {
        // 1. Map hardware to the names configured on your Robot Controller
        intakeMotor   = hardwareMap.get(DcMotor.class, "intake");
        transferMotor = hardwareMap.get(DcMotor.class, "transfer");
        Outtake outtake = new Outtake(hardwareMap,telemetry);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Control powers live via FTC Dashboard!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            intakeMotor.setPower(INTAKE_POWER);
            transferMotor.setPower(TRANSFER_POWER);
            outtake.spinToRpm(SHOOTER_RPM);

            telemetry.addData("[1] Intake Power", INTAKE_POWER);
            telemetry.addData("[2] Transfer Power", TRANSFER_POWER);
            telemetry.addData("[3] Shooter 1 Power", SHOOTER_RPM);
            telemetry.update();
        }
    }
}
