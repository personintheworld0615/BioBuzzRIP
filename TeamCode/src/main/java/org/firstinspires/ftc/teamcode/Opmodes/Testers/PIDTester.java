package org.firstinspires.ftc.teamcode.opmodes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;

@Config
@Utility(name = "Shooter PIDF Tuner")
public class PIDTester extends LinearOpMode {

    public static int TARGET_RPM = 0;
    public static double p;
    public static double i;
    public static double d;
    public static double f;

    private Follower follower;
    private Shooter outtake;
    public DcMotorEx Shooter1, Shooter2;

    @Override
    public void runOpMode() {

        Shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        Shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        outtake = new Shooter(Shooter1, Shooter2, hardwareMap.get(Servo.class, "Linkage"));

        telemetry.addData("INIT", 0);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            outtake.setPIDFConstants(p, i, d, f);
            outtake.spinToRpm(TARGET_RPM);
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Actual RPM motor 1", outtake.getRPM());
            telemetry.addData("Error RPM motor 1", TARGET_RPM - outtake.getRPM());
            telemetry.addData("Actual RPM motor 2", outtake.getRPM2());
            telemetry.addData("Error RPM motor 2", TARGET_RPM - outtake.getRPM2());
            telemetry.update();
        }
    }
}
