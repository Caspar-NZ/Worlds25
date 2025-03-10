package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Example Auto Blue", group = "Examples")
public class ExampleBucketAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Existing Poses (first 7)
    private final Pose pose1 = new Pose(8, 64, Math.toRadians(0));    // Pose 1
    private final Pose pose2 = new Pose(44, 64, Math.toRadians(0));     // Pose 2
    private final Pose pose3 = new Pose(29, 35, Math.toRadians(0));  // Pose 3
    private final Pose pose4 = new Pose(62, 34, Math.toRadians(0));    // Pose 4
    private final Pose pose5 = new Pose(61, 24, Math.toRadians(0));    // Pose 5
    private final Pose pose6 = new Pose(61, 12, Math.toRadians(0));    // Pose 6
    // (Pose 7 from original sample omitted as we use only lines)

    // Additional Poses (8-20)
    private final Pose pose7  = new Pose(21, 12, Math.toRadians(0));         // Pose 8
    private final Pose pose8  = new Pose(61, 13, Math.toRadians(0));         // Pose 9
    private final Pose pose9 = new Pose(61, 5, Math.toRadians(0));         // Pose 10
    private final Pose pose10 = new Pose(10, 4, Math.toRadians(0));         // Pose 11
    private final Pose pose11 = new Pose(10, 27, Math.toRadians(0));         // Pose 12
    private final Pose pose12 = new Pose(7, 35, Math.toRadians(0));           // Pose 13
    private final Pose pose13 = new Pose(43, 69, Math.toRadians(0));          // Pose 14
    private final Pose pose14 = new Pose(6, 36, Math.toRadians(0));          // Pose 15
    private final Pose pose15 = new Pose(44, 69, Math.toRadians(0));          // Pose 16
    private final Pose pose16 = new Pose(7, 35, Math.toRadians(0));          // Pose 17
    private final Pose pose17 = new Pose(44, 69, Math.toRadians(0));           // Pose 18
    private final Pose pose18 = new Pose(15, 29, Math.toRadians(0));           // Pose 19
    private final Pose pose19 = new Pose(10, 29, Math.toRadians(0));         // Pose 20

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13, path14, path15, path16, path17;

    public void buildPaths() {
        // First segment: startPose -> pose4
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose2), new Point(pose3)))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose3), new Point(pose4)))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose4), new Point(pose5)))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose5), new Point(pose6)))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose6), new Point(pose7)))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose7), new Point(pose8)))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose8), new Point(pose9)))
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
                .build();

        path8 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose9), new Point(pose10)))
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
                .build();

        path9 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose10), new Point(pose11)))
                .setLinearHeadingInterpolation(pose10.getHeading(), pose11.getHeading())
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose11), new Point(pose12)))
                .setLinearHeadingInterpolation(pose11.getHeading(), pose12.getHeading())
                .build();

        path11 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose12), new Point(pose13)))
                .setLinearHeadingInterpolation(pose12.getHeading(), pose13.getHeading())
                .build();

        path12 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose13), new Point(pose14)))
                .setLinearHeadingInterpolation(pose13.getHeading(), pose14.getHeading())
                .build();

        path13 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose14), new Point(pose15)))
                .setLinearHeadingInterpolation(pose14.getHeading(), pose15.getHeading())
                .build();

        path14 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose15), new Point(pose16)))
                .setLinearHeadingInterpolation(pose15.getHeading(), pose16.getHeading())
                .build();

        path15 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose16), new Point(pose17)))
                .setLinearHeadingInterpolation(pose16.getHeading(), pose17.getHeading())
                .build();

        path16 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose17), new Point(pose18)))
                .setLinearHeadingInterpolation(pose17.getHeading(), pose18.getHeading())
                .build();

        path17 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose18), new Point(pose19)))
                .setLinearHeadingInterpolation(pose18.getHeading(), pose19.getHeading())
                .build();



    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(path6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path7, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(path8, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(path9, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(path10, true);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(path11, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(path12, true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(path13, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(path14, true);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(path15, true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(path16, true);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(path17, true);
                    setPathState(17);
                }
                break;
            case 17:
                // Autonomous sequence complete.
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(pose1);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {}
}