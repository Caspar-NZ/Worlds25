package pedroPathing.examples;

import static org.firstinspires.ftc.teamcode.functions.intake.RotationMode.TRANSFER;
import static org.firstinspires.ftc.teamcode.functions.intake.RotationMode.TUCKED;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "testAuto", group = "Examples")
public class Test extends OpMode {



    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(7.32, 72.2, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(41.57, 72.2, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose backoff = new Pose(33, 48, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose sample1 = new Pose(45, 36, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose sample1back = new Pose(14, 27, Math.toRadians(0));

    private final Pose line6 = new Pose(45.0, 26.0, Math.toRadians(1));
    private final Pose line7 = new Pose(14, 18.0, Math.toRadians(1));
    private final Pose line8 = new Pose(47.0, 17.0, Math.toRadians(1));
    private final Pose line9 = new Pose(45.0, 15.0, Math.toRadians(1));
    private final Pose line10 = new Pose(8.6, 16.0, Math.toRadians(45));
    private final Pose line11 = new Pose(41.5, 75.0, Math.toRadians(1));
    private final Pose line12 = new Pose(10.0, 48.0, Math.toRadians(1));
    private final Pose line13 = new Pose(41.0, 69.0, Math.toRadians(1));
    private final Pose line14 = new Pose(10.0, 48.0, Math.toRadians(1));
    private final Pose line15 = new Pose(41.0, 69.0, Math.toRadians(1));
    private final Pose line16 = new Pose(10.0, 48.0, Math.toRadians(1));
    private final Pose line17 = new Pose(41.0, 69.0, Math.toRadians(1));
    private final Pose line18 = new Pose(10.0, 48.0, Math.toRadians(1));
    private final Pose line19 = new Pose(41.0, 69.0, Math.toRadians(1));
    private final Pose line20 = new Pose(10.0, 48.0, Math.toRadians(1));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain firstDo, sample, third, fourth, fifth, sixth, seventh, eighth, ninth, tenth, eleventh, twelfth;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        firstDo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        sample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(backoff)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), backoff.getHeading())
                .addPath(new BezierLine(new Point(backoff), new Point(sample1)))
                .setLinearHeadingInterpolation(backoff.getHeading(), sample1.getHeading())
                .addPath(new BezierLine(new Point(sample1), new Point(sample1back)))
                .setLinearHeadingInterpolation(sample1.getHeading(), sample1back.getHeading())
                .addPath(new BezierLine(new Point(sample1back), new Point(line6)))
                .setLinearHeadingInterpolation(sample1.getHeading(), line6.getHeading())
                .addPath(new BezierLine(new Point(line6), new Point(line7)))
                .setLinearHeadingInterpolation(line6.getHeading(), line7.getHeading())
                .addPath(new BezierLine(new Point(line7), new Point(line8)))
                .setLinearHeadingInterpolation(line7.getHeading(), line8.getHeading())
                .addPath(new BezierLine(new Point(line8), new Point(line9)))
                .setLinearHeadingInterpolation(line8.getHeading(), line9.getHeading())
                .addPath(new BezierLine(new Point(line9), new Point(line10)))
                .setLinearHeadingInterpolation(line9.getHeading(), line10.getHeading())
                .build();

        third = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line10), new Point(line11)))
                .setLinearHeadingInterpolation(line10.getHeading(), line11.getHeading())
                .build();

        fourth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line11), new Point(line12)))
                .setLinearHeadingInterpolation(line11.getHeading(), line12.getHeading())
                .build();

        fifth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line12), new Point(line13)))
                .setLinearHeadingInterpolation(line12.getHeading(), line13.getHeading())
                .build();

        sixth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line13), new Point(line14)))
                .setLinearHeadingInterpolation(line13.getHeading(), line14.getHeading())
                .build();

        seventh = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line14), new Point(line15)))
                .setLinearHeadingInterpolation(line14.getHeading(), line15.getHeading())
                .build();

        eighth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line15), new Point(line16)))
                .setLinearHeadingInterpolation(line15.getHeading(), line16.getHeading())
                .build();

        ninth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line16), new Point(line17)))
                .setLinearHeadingInterpolation(line16.getHeading(), line17.getHeading())
                .build();

        tenth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line17), new Point(line18)))
                .setLinearHeadingInterpolation(line17.getHeading(), line18.getHeading())
                .build();

        eleventh = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line18), new Point(line19)))
                .setLinearHeadingInterpolation(line18.getHeading(), line19.getHeading())
                .build();

        twelfth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(line19), new Point(line20)))
                .setLinearHeadingInterpolation(line19.getHeading(), line20.getHeading())
                .build();





    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(firstDo, false);
                setPathState(1);
                break;
            case 1:
                follower.setMaxPower(100);
                if(!follower.isBusy()){
                    follower.followPath(sample, false);
                    setPathState(2);
                }
                break;

            case 2:
                if(!follower.isBusy()){
                    follower.followPath(third, false);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy()){
                    follower.followPath(fourth, false);
                    setPathState(4);
                }
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        horiSlides horizontalSlides = new horiSlides(hardwareMap);
        vertSlide verticalSlides = new vertSlide(hardwareMap);
        outtake outtake = new outtake(hardwareMap);
        intake intake = new intake(hardwareMap);

        // Initialize intake and outtake positions.
        // Default manual configuration (right trigger): inner blocker closed, outer blocker open.
        intake.setRotation(TRANSFER);
        intake.setInnerBlockOpen(false);
        intake.setOuterBlockOpen(true);
        horizontalSlides.setPosition(0);
        intake.update();
        horizontalSlides.update();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}