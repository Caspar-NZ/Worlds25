package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Example Auto Blue", group = "Examples")
@Disabled
public class ExampleBucketAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Updated Poses with new coordinates
    private final Pose pose1 = new Pose(7.36, 72.20, Math.toRadians(1));    // Start Pose
    private final Pose pose2 = new Pose(41.57, 72.20, Math.toRadians(1));   // Line 1
    private final Pose pose3 = new Pose(33.61, 48.33, Math.toRadians(1));   // Line 2
    private final Pose pose4 = new Pose(45.94, 34.01, Math.toRadians(1));   // Line 3
    private final Pose pose5 = new Pose(8.75, 27.25, Math.toRadians(1));    // Line 4
    private final Pose pose6 = new Pose(47.54, 25.86, Math.toRadians(1));   // Line 5
    private final Pose pose7 = new Pose(45.94, 23.07, Math.toRadians(1));   // Line 6
    private final Pose pose8 = new Pose(9.15, 18.70, Math.toRadians(1));    // Line 7
    private final Pose pose9 = new Pose(46.54, 16.51, Math.toRadians(1));   // Line 8
    private final Pose pose10 = new Pose(45.94, 14.32, Math.toRadians(1));  // Line 9
    private final Pose pose11 = new Pose(8.55, 16.31, Math.toRadians(1));   // Line 10
    private final Pose pose12 = new Pose(42.96, 77.57, Math.toRadians(1));  // Line 11
    private final Pose pose13 = new Pose(9.15, 47.73, Math.toRadians(1));   // Line 12
    private final Pose pose14 = new Pose(41.37, 69.22, Math.toRadians(1));  // Line 13
    private final Pose pose15 = new Pose(9.55, 47.73, Math.toRadians(1));   // Line 14
    private final Pose pose16 = new Pose(41.57, 69.61, Math.toRadians(1));  // Line 15
    private final Pose pose17 = new Pose(9.35, 47.54, Math.toRadians(1));   // Line 16
    private final Pose pose18 = new Pose(41.57, 69.81, Math.toRadians(1));  // Line 17
    private final Pose pose19 = new Pose(8.95, 47.54, Math.toRadians(1));   // Line 18
    private final Pose pose20 = new Pose(41.37, 69.61, Math.toRadians(1));  // Line 19
    private final Pose pose21 = new Pose(9.35, 47.54, Math.toRadians(1));   // Line 20

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13, path14, path15, path16, path17, path18, path19, path20, path21;

    // Build the paths
    public void buildPaths() {
        path1 = createPath(pose1, pose2);   // Start from pose1
        path2 = createPath(pose2, pose3);   // Go to pose3
        path3 = createPath(pose3, pose4);   // Path 3
        path4 = createPath(pose4, pose5);   // Path 4
        path5 = createPath(pose5, pose6);   // Path 5
        path6 = createPath(pose6, pose7);   // Path 6
        path7 = createPath(pose7, pose8);   // Path 7
        path8 = createPath(pose8, pose9);   // Path 8
        path9 = createPath(pose9, pose10);  // Path 9
        path10 = createPath(pose10, pose11); // Path 10
        path11 = createPath(pose11, pose12); // Path 11
        path12 = createPath(pose12, pose13); // Path 12
        path13 = createPath(pose13, pose14); // Path 13
        path14 = createPath(pose14, pose15); // Path 14
        path15 = createPath(pose15, pose16); // Path 15
        path16 = createPath(pose16, pose17); // Path 16
        path17 = createPath(pose17, pose18); // Path 17
        path18 = createPath(pose18, pose19); // Path 18
        path19 = createPath(pose19, pose20); // Path 19
        path20 = createPath(pose20, pose21); // Path 20
    }

    // Helper function to create paths
    private PathChain createPath(Pose startPose, Pose endPose) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(endPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }

    // Update autonomous path state
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.setMaxPower(100); // Set speed to 100%
                follower.followPath(path1, false);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path2, false);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path3, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path4, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path5, false);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path6, false);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path7, false);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path8, false);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path9, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path10, false);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path11, false);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path12, false);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path13, false);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path14, false);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path15, false);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path16, false);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path17, false);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path18, false);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path19, false);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path20, false);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    telemetry.addData("Autonomous", "Completed all paths.");
                }
                break;
        }
    }


    // Set the current path state and reset the timer
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(pose1);  // The robot starts at pose1
        buildPaths();  // Build all the paths
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);  // Start the first path
    }

    @Override
    public void stop() {}
}
