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

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "testAuto", group = "Examples")
public class Test extends OpMode {

    private horiSlides horizontalSlides;
    private vertSlide verticalSlides;
    private outtake outtake;
    private intake intake;

    private Thread asyncUpdatesThread;
    private volatile boolean asyncThread = true;

    private ScheduledExecutorService scheduler;





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
    private final Pose startPose = new Pose(7.32, 77.2, Math.toRadians(0));//72.2

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(41.57, 77.2, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose backoff = new Pose(36, 50, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose sample1 = new Pose(42.2, 36, Math.toRadians(0));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose sample1back = new Pose(16, 27, Math.toRadians(0));

    private final Pose sample2 = new Pose(34.0, 27, Math.toRadians(0));

    private final Pose sample2T = new Pose(43, 21.8, Math.toRadians(0));
    private final Pose sample2B = new Pose(16, 18.0, Math.toRadians(0));
    private final Pose sample3 = new Pose(35.0, 17.4, Math.toRadians(0));
    private final Pose sample3T = new Pose(43, 13, Math.toRadians(0));
    private final Pose sample3B = new Pose(8, 13, Math.toRadians(0));
    private final Pose drop1 = new Pose(41.5, 72.0, Math.toRadians(0));
    private final Pose pickup2 = new Pose(8.9, 47.5, Math.toRadians(0));
    private final Pose drop2 = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose pickup3 = new Pose(8.9, 47.5, Math.toRadians(0));
    private final Pose drop3 = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose pickup4 = new Pose(8.9, 47.5, Math.toRadians(0));
    private final Pose drop4 = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose pickup5 = new Pose(8.9, 47.5, Math.toRadians(0));
    private final Pose drop5 = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose line20 = new Pose(8.9, 47.0, Math.toRadians(0));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain firstDo, sample, third, fourth, fifth, sixth, seventh;//, eighth, ninth, tenth, eleventh, twelfth;

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
                .addPath(new BezierLine(new Point(sample1back), new Point(sample2)))
                .setLinearHeadingInterpolation(sample1.getHeading(), sample2.getHeading())
                .addPath(new BezierLine(new Point(sample2), new Point(sample2T)))
                .setLinearHeadingInterpolation(sample2.getHeading(), sample2T.getHeading())
                .addPath(new BezierLine(new Point(sample2T), new Point(sample2B)))
                .setLinearHeadingInterpolation(sample2T.getHeading(), sample2B.getHeading())
                .addPath(new BezierLine(new Point(sample2B), new Point(sample3)))
                .setLinearHeadingInterpolation(sample2B.getHeading(), sample3.getHeading())
                .addPath(new BezierLine(new Point(sample3), new Point(sample3T)))
                .setLinearHeadingInterpolation(sample3.getHeading(), sample3T.getHeading())
                .addPath(new BezierLine(new Point(sample3), new Point(sample3B)))
                .setLinearHeadingInterpolation(sample3T.getHeading(), sample3B.getHeading())
                .build();

        third = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3B), new Point(drop1)))
                .setLinearHeadingInterpolation(sample3B.getHeading(), drop1.getHeading())
                .addPath(new BezierLine(new Point(drop1), new Point(pickup2)))
                .setLinearHeadingInterpolation(drop1.getHeading(), pickup2.getHeading())
                .build();

        fourth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2), new Point(drop2)))
                .setLinearHeadingInterpolation(pickup2.getHeading(), drop2.getHeading())
                .addPath(new BezierLine(new Point(drop2), new Point(pickup3)))
                .setLinearHeadingInterpolation(drop2.getHeading(), pickup3.getHeading())
                .build();

        fifth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3), new Point(drop3)))
                .setLinearHeadingInterpolation(pickup3.getHeading(), drop3.getHeading())
                .addPath(new BezierLine(new Point(drop3), new Point(pickup4)))
                .setLinearHeadingInterpolation(drop3.getHeading(), pickup4.getHeading())
                .build();


        sixth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup4), new Point(drop4)))
                .setLinearHeadingInterpolation(pickup4.getHeading(), drop4.getHeading())
                .addPath(new BezierLine(new Point(drop4), new Point(pickup5)))
                .setLinearHeadingInterpolation(drop4.getHeading(), pickup5.getHeading())
                .build();


        seventh = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup5), new Point(drop5)))
                .setLinearHeadingInterpolation(pickup5.getHeading(), drop5.getHeading())
                .addPath(new BezierLine(new Point(drop5), new Point(line20)))
                .setLinearHeadingInterpolation(drop5.getHeading(), line20.getHeading())
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
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION), 0);
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
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(fifth, false);
                    setPathState(5);
                }
                break;

            case 5:
                if(!follower.isBusy()){
                    follower.followPath(sixth, false);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    follower.followPath(seventh, false);
                    setPathState(7);
                }
                break;

//            case 7:
//                if(!follower.isBusy()){
//                    follower.followPath(eighth, false);
//                    setPathState(8);
//                }
//                break;
//
//            case 8:
//                if(!follower.isBusy()){
//                    follower.followPath(ninth, false);
//                    setPathState(9);
//                }
//                break;

//            case 9:
//                if(!follower.isBusy()){
//                    follower.followPath(tenth, false);
//                    setPathState(10);
//                }
//                break;
//
//            case 10:
//                if(!follower.isBusy()){
//                    follower.followPath(eleventh, false);
//                    setPathState(11);
//                }
//                break;
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

        horizontalSlides = new horiSlides(hardwareMap);
        verticalSlides = new vertSlide(hardwareMap);
        outtake = new outtake(hardwareMap);
        intake = new intake(hardwareMap);

        scheduler = Executors.newScheduledThreadPool(1);

        // Define the runnable for async updates
        Runnable asyncUpdates = new Runnable() {
            @Override
            public void run() {
                while (asyncThread && !Thread.currentThread().isInterrupted()) {
                    horizontalSlides.update();
                    verticalSlides.update();
                    outtake.update();
                    intake.update();

                    // Add slight delay to avoid rapid updates causing jitter
                    try {
                        Thread.sleep(5); // Sleep for 5 ms to give hardware time to respond
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt(); // Ensure the thread exits if interrupted
                    }
                }
            }
        };

        // Initialize but do not start the thread yet
        asyncUpdatesThread = new Thread(asyncUpdates);



        asyncThread = true;
        asyncUpdatesThread.start();




        // Initialize intake and outtake positions.
        // Default manual configuration (right trigger): inner blocker closed, outer blocker open.
        intake.setRotation(TRANSFER);
        intake.setInnerBlockOpen(false);
        intake.setOuterBlockOpen(true);
        horizontalSlides.setPosition(0);
        outtake.hookAtIntake(false,true);
        outtake.clawOpen(true);
        outtake.specDropAtIntakePos(true);
        outtake.specDropOpen(false);
        verticalSlides.setPosition(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driv
    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 50), 0);
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        shutdownThread();
        scheduler.shutdown();
    }

    private void delayedRun(Runnable action, long delayInMillis) {
        scheduler.schedule(action, delayInMillis, TimeUnit.MILLISECONDS);
    }

    // Method to properly stop the asyncUpdatesThread
    private void shutdownThread() {
        asyncThread = false; // Stop the loop in the async runnable
        if (asyncUpdatesThread != null) {
            asyncUpdatesThread.interrupt(); // Interrupt any sleeping or waiting thread
            try {
                asyncUpdatesThread.join(); // Wait for the thread to terminate properly
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Handle interruption and ensure the main thread isn't left interrupted
            }
        }
    }



}