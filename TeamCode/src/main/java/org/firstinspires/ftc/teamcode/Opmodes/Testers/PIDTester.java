package org.firstinspires.ftc.teamcode.Opmodes.Testers;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import org.firstinspires.ftc.teamcode.robot.subsystem.Outtake;

@Config
@Utility(name = "Shooter PIDF Tuner")
public class PIDTester extends LinearOpMode {

    public static int TARGET_RPM = 0;
    public static double p;
    public static double i;
    public static double d;
    public static double f;

    private Follower follower;
    private Outtake outtake;

    @Override
    public void runOpMode() {

        outtake = new Outtake(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized. Open FTC Dashboard to tune PIDF constants!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            outtake.setPIDFConstants(p, i, d, f);

            // 4. Spin up the shooter using the live target RPM dashboard variable
            outtake.spinToRpm(TARGET_RPM);
            // 5. Send telemetry data to monitor targets vs reality
            // In FTC Dashboard, these will draw clean graphs automatically!
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Actual RPM motor 1", outtake.getRPM());
            telemetry.addData("Error RPM motor 1", TARGET_RPM - outtake.getRPM());
            telemetry.addData("Actual RPM motor 2", outtake.getRPM2());
            telemetry.addData("Error RPM motor 2", TARGET_RPM - outtake.getRPM2());
            telemetry.update();
        }
    }
}
