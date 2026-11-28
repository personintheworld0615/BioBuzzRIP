package org.firstinspires.ftc.teamcode.Opmodes.Prod.Auto;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Autonomous(name = "blue")
@Config
public class bluebad extends LinearOpMode {

    public static Follower follower;
    public static final PoseFactory poseFactory = PoseFactory.degrees();

    public static final Pose start = poseFactory.of(77.21493552679883, 5.262300959067547, 90);
    public static final Pose end = poseFactory.of(132, 26.04, 90);
    public static final Pose control = poseFactory.of(87.2, 27.72, 90);


    public static Robot robot;

    public static Path path1() {
        return Paths.curve(start, control, end)
                .linear(start,end );
    }


    public static Command autoRoutine() {
        return sequential(
                //shoot
//                parallel(
//                        follow(follower, path1()),
//                        sequential(
//                                waitUntil(() -> follower.completion() > 0.6),
//                                robot.autoIntakeCommand(500)
//                        )
//                ),
                follow(follower, path1())

        );
    }

    public static Command shoot() {
        return robot.autonShootSequence(true);
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();

        if (isStopRequested()) return;

        Scheduler.schedule(autoRoutine());

        while (opModeIsActive() && !isStopRequested()) {
            Scheduler.execute();
            follower.update();
        }
    }
}