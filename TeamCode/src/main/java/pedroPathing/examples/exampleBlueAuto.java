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

@Autonomous(name = "example Auto Blue", group = "Examples")
@Disabled
public class exampleBlueAuto extends OpMode {

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

    private PathChain combinedPath;

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13, path14, path15, path16, path17, path18, path19, path20, path21;

    // Build the paths
    public void buildPaths() {

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

    public void buildPaths2() {
        // Create the path chain
        PathChain pathChain = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pose1), new Point(pose2))) // Path from pose1 to pose2
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())  // Interpolate heading between pose1 and pose2
                .addPath(new BezierLine(new Point(pose2), new Point(pose3))) // Path from pose2 to pose3
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())  // Interpolate heading between pose2 and pose3
                .addPath(new BezierLine(new Point(pose3), new Point(pose4))) // Path from pose3 to pose4
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())  // Interpolate heading between pose3 and pose4
                .addPath(new BezierLine(new Point(pose4), new Point(pose5))) // Path from pose4 to pose5
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())  // Interpolate heading between pose4 and pose5
                .addPath(new BezierLine(new Point(pose5), new Point(pose6))) // Path from pose5 to pose6
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())  // Interpolate heading between pose5 and pose6
                .addPath(new BezierLine(new Point(pose6), new Point(pose7))) // Path from pose6 to pose7
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())  // Interpolate heading between pose6 and pose7
                .addPath(new BezierLine(new Point(pose7), new Point(pose8))) // Path from pose7 to pose8
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())  // Interpolate heading between pose7 and pose8
                .addPath(new BezierLine(new Point(pose8), new Point(pose9))) // Path from pose8 to pose9
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())  // Interpolate heading between pose8 and pose9
                .addPath(new BezierLine(new Point(pose9), new Point(pose10))) // Path from pose9 to pose10
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())  // Interpolate heading between pose9 and pose10
                .setPathEndTimeoutConstraint(3.0) // Timeout for the entire path chain
                .build();

        combinedPath = pathChain; // Assign the pathChain to the combinedPath
    }
    public void autonomousPathUpdate() {
        switch (pathState) {


            case 0:
                if(!follower.isBusy()){
                    follower.setMaxPower(100);
                    follower.followPath(combinedPath, false);
                    setPathState(1);

                }
            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path12, false);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path13, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path14, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path15, false);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path16, false);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path17, false);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path18, false);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path19, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.setMaxPower(100); // Set speed to 100%
                    follower.followPath(path20, false);
                    setPathState(10);
                }
                break;
            case 10:
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
