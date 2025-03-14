package pedroPathing.examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
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

    // Updated Poses with the new coordinates
    private final Pose pose1 = new Pose(0.9944751381215469, 75.97790055248619, Math.toRadians(0));   // Start Point
    private final Pose pose2 = new Pose(33.414364640883974, 74.98342541436463, Math.toRadians(0));  // Line 1
    private final Pose pose3 = new Pose(29.43646408839779, 74.78453038674033, Math.toRadians(0));  // Line 2
    private final Pose pose4 = new Pose(29.23756906077348, 45.34806629834254, Math.toRadians(0));  // Line 3
    private final Pose pose5 = new Pose(64.64088397790056, 35.204419889502766, Math.toRadians(0)); // Line 4
    private final Pose pose6 = new Pose(7.160220994475138, 29.03867403314917, Math.toRadians(0));   // Line 5
    private final Pose pose7 = new Pose(62.65193370165746, 31.8232044198895, Math.toRadians(0));   // Line 6
    private final Pose pose8 = new Pose(61.85635359116022, 24.26519337016574, Math.toRadians(0));  // Line 7
    private final Pose pose9 = new Pose(7.7569060773480665, 21.67955801104973, Math.toRadians(0));  // Line 8
    private final Pose pose10 = new Pose(61.85635359116022, 20.287292817679564, Math.toRadians(0)); // Line 9
    private final Pose pose11 = new Pose(61.4585635359116, 15.513812154696133, Math.toRadians(0)); // Line 10
    private final Pose pose12 = new Pose(7.160220994475138, 17.502762430939228, Math.toRadians(0)); // Line 11
    private final Pose pose13 = new Pose(10.143646408839778, 45.745856353591165, Math.toRadians(0)); // Line 12
    private final Pose pose14 = new Pose(1.1933701657458564, 45.54696132596686, Math.toRadians(0)); // Line 13
    private final Pose pose15 = new Pose(33.414364640883974, 73.19337016574586, Math.toRadians(0));  // Line 14
    private final Pose pose16 = new Pose(1.3922651933701657, 45.54696132596686, Math.toRadians(0));  // Line 15
    private final Pose pose17 = new Pose(33.21546961325967, 74.98342541436463, Math.toRadians(0));  // Line 16
    private final Pose pose18 = new Pose(1.591160220994475, 45.54696132596686, Math.toRadians(0));   // Line 17
    private final Pose pose19 = new Pose(33.613259668508285, 72.79558011049724, Math.toRadians(0));  // Line 18
    private final Pose pose20 = new Pose(1.7900552486187844, 45.745856353591165, Math.toRadians(0)); // Line 19
    private final Pose pose21 = new Pose(34.209944751381215, 71.20441988950276, Math.toRadians(0)); // Line 20
    private final Pose pose22 = new Pose(1.7900552486187844, 45.149171270718234, Math.toRadians(0)); // Line 21
    private final Pose pose23 = new Pose(34.209944751381215, 69.41436464088397, Math.toRadians(0)); // Line 22

    private PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11, path12, path13, path14, path15, path16, path17, path18, path19, path20, path21, path22, path23;

    // Build the paths
    public void buildPaths() {
        path1 = createPath(pose1, pose2);   // Line 1
        path2 = createPath(pose2, pose3);   // Line 2
        path3 = createPath(pose3, pose4);   // Line 3
        path4 = createPath(pose4, pose5);   // Line 4
        path5 = createPath(pose5, pose6);   // Line 5
        path6 = createPath(pose6, pose7);   // Line 6
        path7 = createPath(pose7, pose8);   // Line 7
        path8 = createPath(pose8, pose9);   // Line 8
        path9 = createPath(pose9, pose10);  // Line 9
        path10 = createPath(pose10, pose11);  // Line 10
        path11 = createPath(pose11, pose12);  // Line 11
        path12 = createPath(pose12, pose13);  // Line 12
        path13 = createPath(pose13, pose14);  // Line 13
        path14 = createPath(pose14, pose15);  // Line 14
        path15 = createPath(pose15, pose16);  // Line 15
        path16 = createPath(pose16, pose17);  // Line 16
        path17 = createPath(pose17, pose18);  // Line 17
        path18 = createPath(pose18, pose19);  // Line 18
        path19 = createPath(pose19, pose20);  // Line 19
        path20 = createPath(pose20, pose21);  // Line 20
        path21 = createPath(pose21, pose22);  // Line 21
        path22 = createPath(pose22, pose23);  // Line 22
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
                follower.followPath(path1, false);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path2, false);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path4, false);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(path5, false);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(path6, false);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(path7, false);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(path8, false);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(path9, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(path10, false);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(path11, false);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(path12, false);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(path13, false);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(path14, false);
                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(path15, false);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    follower.followPath(path16, false);
                    setPathState(16);
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    follower.followPath(path17, false);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(path18, false);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(path19, false);
                    setPathState(19);
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    follower.followPath(path20, false);
                    setPathState(20);
                }
                break;
            case 20:
                if (!follower.isBusy()) {
                    follower.followPath(path21, false);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(path22, false);
                    setPathState(22);
                }
                break;
            case 22:
                if (!follower.isBusy()) {
                    // Autonomous sequence complete
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
