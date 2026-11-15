package org.firstinspires.ftc.teamcode.opmodes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Util.MyTelem;
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
        MyTelem.init(telemetry);

        Shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        Shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        outtake = new Shooter(hardwareMap);

        MyTelem.addData("INIT", 0);
        MyTelem.update();

        waitForStart();

        while (opModeIsActive()) {
            outtake.setPIDFConstants(p, i, d, f);
            outtake.spinToRpm(TARGET_RPM);
            MyTelem.addData("Target RPM", TARGET_RPM);
            MyTelem.addData("Actual RPM motor 1", outtake.getRPM());
            MyTelem.addData("Error RPM motor 1", TARGET_RPM - outtake.getRPM());
            MyTelem.addData("Actual RPM motor 2", outtake.getRPM2());
            MyTelem.addData("Error RPM motor 2", TARGET_RPM - outtake.getRPM2());
            MyTelem.update();
        }
    }
}
