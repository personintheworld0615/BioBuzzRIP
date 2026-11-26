package org.firstinspires.ftc.teamcode.Opmodes.Prod.Auto;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

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

@Autonomous(name = "Main Auto")
public class RedOptimalAuto extends LinearOpMode {

    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54, 6, 90);
    private final Pose path1 = poseFactory.of(8.7418, 8.2773, 270);
    private final Pose path1Control1 = poseFactory.of(33.0532, 31.4887, 0);
    private final Pose path2 = poseFactory.of(45.7385, 134.1377, 90);
    private final Pose path2Control1 = poseFactory.of(27.2305, 53.1331, 0);
    private final Pose path2Control2 = poseFactory.of(21.9416, 101.1883, 0);
    private final Pose path2Control3 = poseFactory.of(47.0188, 105.3931, 0);
    private final Pose path2Segment1Start = poseFactory.of(45.7385, 134.1377, 270);
    private final Pose path2Segment1End = poseFactory.of(45.7385, 134.1377, 90);
    private final Pose path2Segment2Heading = poseFactory.of(45.7385, 134.1377, 90);
    private final Pose path3 = poseFactory.of(8.7964, 46.5469, 180);
    private final Pose path3Control1 = poseFactory.of(28.2657, 98.8783, 0);
    private final Pose path3Control2 = poseFactory.of(44.0801, 39.2626, 0);
    private final Pose path3Segment1Start = poseFactory.of(8.7964, 46.5469, 90);
    private final Pose path3Segment1End = poseFactory.of(8.7964, 46.5469, 180);
    private final Pose path4 = poseFactory.of(28.0306, 29.144, 91.7451);
    private final Pose path4Control1 = poseFactory.of(29.8583, 47.0098, 0);
    private final Pose path4Control2 = poseFactory.of(27.7133, 39.5587, 0);

    private Robot robot;

    private Path path1() {
        return Paths.curve(start, path1Control1, path1)
                .linear(start, path1);
    }
    private Path path2() {
        return Paths.curve(path1, path2Control1, path2Control2, path2Control3, path2)
                .heading(Interpolator.piecewise()
                        .until(0.8, Interpolator.linear(path2Segment1Start, path2Segment1End))
                        .until(1, Interpolator.constant(path2Segment2Heading)));
    }
    private Path path3() {
        return Paths.curve(path2, path3Control1, path3Control2, path3)
                .heading(Interpolator.piecewise()
                        .until(1, Interpolator.linear(path3Segment1Start, path3Segment1End)));
    }
    private Path path4() {
        return Paths.curve(path3, path4Control1, path4Control2, path4)
                .reverseTangent();
    }


    private Command autoRoutine() {
        return sequential(
//                shoot(),
//                parallel(
//                    follow(follower,path1()),
//                    sequential(
//                        waitUntil(()-> follower.completion()>0.8),
//                        robot.autoIntakeCommand(500)
//
//                    )
//                );
                follow(follower, path1()),

//                shoot(), //idt is needed
                follow(follower, path2()),
//                shoot(),
                follow(follower, path3()),
//                shoot(),
                follow(follower, path4())
        );
    }
    private Command shoot(){
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
