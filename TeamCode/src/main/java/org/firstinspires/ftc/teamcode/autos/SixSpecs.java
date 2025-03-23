package org.firstinspires.ftc.teamcode.autos;


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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "SixSpecs", group = "Worlds")
public class SixSpecs extends OpMode {
    private double timeout = 0;

    private double slowdown = 1.0;

    //private LynxModule expansionHub;

    private horiSlides horizontalSlides;
    private vertSlide verticalSlides;
    private outtake outtake;
    private intake intake;

    private Thread asyncUpdatesThread;
    private volatile boolean asyncThread = true;

    private ScheduledExecutorService scheduler;

    private double waitingTimer;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // Define all poses and paths

    private final Pose startPose = new Pose(7.32, 77.2, Math.toRadians(0));
    private final Pose pre1stDrop = new Pose(35, 77.2, Math.toRadians(0));
    private final Pose slow1stDrop = new Pose(41, 77.2, Math.toRadians(0));
    private final Pose preFirstSample = new Pose(36, 50, Math.toRadians(0));
    private final Pose firstSample = new Pose(42.2, 36, Math.toRadians(0));
    private final Pose firstPush = new Pose(16, 27, Math.toRadians(0));
    private final Pose secondSample = new Pose(34.0, 27, Math.toRadians(0));
    private final Pose preSecondPush = new Pose(43, 21.8, Math.toRadians(0));
    private final Pose secondPush = new Pose(16, 18.0, Math.toRadians(0));
    private final Pose thirdSample = new Pose(35.0, 17.4, Math.toRadians(0));
    private final Pose preThirdPush = new Pose(43, 13, Math.toRadians(0));
    private final Pose preFarPickUp = new Pose(12, 13, Math.toRadians(0));
    private final Pose farPickUp = new Pose(8, 13, Math.toRadians(0));
    private final Pose pre2ndDrop = new Pose(34.0, 70.0, Math.toRadians(0));
    private final Pose slow2ndDrop = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose preClosePickUp = new Pose(12, 47, Math.toRadians(0));
    private final Pose closePickUp = new Pose(8, 47, Math.toRadians(0));
    private final Pose pre3rdDrop = new Pose(34.0, 70.0, Math.toRadians(0));
    private final Pose slow3rdDrop = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose pre4thDrop = new Pose(34.0, 70.0, Math.toRadians(0));
    private final Pose slow4thDrop = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose pre5thDrop = new Pose(34.0, 70.0, Math.toRadians(0));
    private final Pose slow5thDrop = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose pre6thDrop = new Pose(34.0, 70.0, Math.toRadians(0));
    private final Pose slow6thDrop = new Pose(38.0, 70.0, Math.toRadians(0));
    private final Pose parkPose = new Pose(10.0, 40.0, Math.toRadians(0));


    // Paths and PathChains
    private PathChain preFirstSpec, firstSpec, toFirstSample, pushFirstSample, toSecondSample, pushSecondSample, toThirdSample, pushThirdSample, firstPickUp, preSecondSpec, secondSpec, preSecondPickUp, secondPickUp, preThirdSpec, thirdSpec, preThirdPickUp, thirdPickUp, preFourthSpec, fourthSpec, preFourthPickUp, fourthPickUp, preFithSpec, fithSpec, preFithPickUp, fithPickUp, preSixthSpec, sixthSpec, park;


    public void buildPaths() {
        preFirstSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(pre1stDrop)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pre1stDrop.getHeading())
                .build();
        firstSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre1stDrop), new Point(slow1stDrop)))
                .setLinearHeadingInterpolation(pre1stDrop.getHeading(), slow1stDrop.getHeading())
                .build();
        toFirstSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slow1stDrop), new Point(preFirstSample)))
                .setLinearHeadingInterpolation(slow1stDrop.getHeading(), preFirstSample.getHeading())
                .addPath(new BezierLine(new Point(preFirstSample), new Point(firstSample)))
                .setLinearHeadingInterpolation(preFirstSample.getHeading(), firstSample.getHeading())
                .build();
        pushFirstSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSample), new Point(firstPush)))
                .setLinearHeadingInterpolation(firstSample.getHeading(), firstPush.getHeading())
                .build();
        toSecondSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPush), new Point(secondSample)))
                .setLinearHeadingInterpolation(firstPush.getHeading(), secondSample.getHeading())
                .build();
        pushSecondSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSample), new Point(preSecondPush)))
                .setLinearHeadingInterpolation(secondSample.getHeading(), preSecondPush.getHeading())
                .addPath(new BezierLine(new Point(preSecondPush), new Point(secondPush)))
                .setLinearHeadingInterpolation(preSecondPush.getHeading(), secondPush.getHeading())
                .build();
        toThirdSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPush), new Point(thirdSample)))
                .setLinearHeadingInterpolation(secondPush.getHeading(), thirdSample.getHeading())
                .build();
        pushThirdSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSample), new Point(preThirdPush)))
                .setLinearHeadingInterpolation(thirdSample.getHeading(), preThirdPush.getHeading())
                .addPath(new BezierLine(new Point(preThirdPush), new Point(preFarPickUp)))
                .setLinearHeadingInterpolation(preThirdPush.getHeading(), preFarPickUp.getHeading())
                .build();
        firstPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preFarPickUp), new Point(farPickUp)))
                .setLinearHeadingInterpolation(preFarPickUp.getHeading(), farPickUp.getHeading())
                .build();
        preSecondSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(farPickUp), new Point(pre2ndDrop)))
                .setLinearHeadingInterpolation(farPickUp.getHeading(), pre2ndDrop.getHeading())
                .build();
        secondSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre2ndDrop), new Point(slow2ndDrop)))
                .setLinearHeadingInterpolation(pre2ndDrop.getHeading(), slow2ndDrop.getHeading())
                .build();
        preSecondPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slow2ndDrop), new Point(preClosePickUp)))
                .setLinearHeadingInterpolation(slow2ndDrop.getHeading(), preClosePickUp.getHeading())
                .build();
        secondPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preClosePickUp), new Point(closePickUp)))
                .setLinearHeadingInterpolation(preClosePickUp.getHeading(), closePickUp.getHeading())
                .build();
        preThirdSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(closePickUp), new Point(pre3rdDrop)))
                .setLinearHeadingInterpolation(closePickUp.getHeading(), pre3rdDrop.getHeading())
                .build();
        thirdSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre3rdDrop), new Point(slow3rdDrop)))
                .setLinearHeadingInterpolation(pre3rdDrop.getHeading(), slow3rdDrop.getHeading())
                .build();
        preThirdPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slow3rdDrop), new Point(preClosePickUp)))
                .setLinearHeadingInterpolation(slow3rdDrop.getHeading(), preClosePickUp.getHeading())
                .build();
        thirdPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preClosePickUp), new Point(closePickUp)))
                .setLinearHeadingInterpolation(preClosePickUp.getHeading(), closePickUp.getHeading())
                .build();
        preFourthSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(closePickUp), new Point(pre4thDrop)))
                .setLinearHeadingInterpolation(closePickUp.getHeading(), pre4thDrop.getHeading())
                .build();
        fourthSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre4thDrop), new Point(slow4thDrop)))
                .setLinearHeadingInterpolation(pre4thDrop.getHeading(), slow4thDrop.getHeading())
                .build();
        preFourthPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slow4thDrop), new Point(preClosePickUp)))
                .setLinearHeadingInterpolation(slow4thDrop.getHeading(), preClosePickUp.getHeading())
                .build();
        fourthPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preClosePickUp), new Point(closePickUp)))
                .setLinearHeadingInterpolation(preClosePickUp.getHeading(), closePickUp.getHeading())
                .build();
        preFithSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(closePickUp), new Point(pre5thDrop)))
                .setLinearHeadingInterpolation(closePickUp.getHeading(), pre5thDrop.getHeading())
                .build();
        fithSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre5thDrop), new Point(slow5thDrop)))
                .setLinearHeadingInterpolation(pre5thDrop.getHeading(), slow5thDrop.getHeading())
                .build();
        preFithPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slow5thDrop), new Point(preClosePickUp)))
                .setLinearHeadingInterpolation(slow5thDrop.getHeading(), preClosePickUp.getHeading())
                .build();
        fithPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preClosePickUp), new Point(closePickUp)))
                .setLinearHeadingInterpolation(preClosePickUp.getHeading(), closePickUp.getHeading())
                .build();
        preSixthSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(closePickUp), new Point(pre6thDrop)))
                .setLinearHeadingInterpolation(closePickUp.getHeading(), pre6thDrop.getHeading())
                .build();
        sixthSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pre6thDrop), new Point(slow6thDrop)))
                .setLinearHeadingInterpolation(pre6thDrop.getHeading(), slow6thDrop.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(slow6thDrop), new Point(parkPose)))
                .setLinearHeadingInterpolation(slow6thDrop.getHeading(), parkPose.getHeading())
                .build();


    }

    public void autonomousPathUpdate() {
        double y = 5.0;
        switch(pathState) {
            case 0:
                // This path transitions from startPose to pre1stDrop (the slowdown point before first drop).
                timeout = y; // TODO: set appropriate timeout value for preFirstSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preFirstSpec, false);
                    setPathState(pathState + 1);
                }
                break;
            case 1:
                // This path transitions from pre1stDrop to slow1stDrop (first drop slowdown).
                timeout = y; // TODO: set appropriate timeout value for firstSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(firstSpec, true);
                    setPathState(pathState + 1);
                }
                break;
            case 2:
                // This path transitions from slow1stDrop to firstSample (moving to first sample).
                timeout = y; // TODO: set appropriate timeout value for toFirstSample
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(toFirstSample, false);
                    setPathState(pathState + 1);
                }
                break;
            case 3:
                // This path transitions from firstSample to firstPush (pushing the first sample).
                timeout = y; // TODO: set appropriate timeout value for pushFirstSample
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(pushFirstSample, false);
                    setPathState(pathState + 1);
                }
                break;
            case 4:
                // This path transitions from firstPush to secondSample (moving to second sample).
                timeout = y; // TODO: set appropriate timeout value for toSecondSample
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(toSecondSample, false);
                    setPathState(pathState + 1);
                }
                break;
            case 5:
                // This path transitions from secondSample to secondPush (pushing the second sample).
                timeout = y; // TODO: set appropriate timeout value for pushSecondSample
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(pushSecondSample, false);
                    setPathState(pathState + 1);
                }
                break;
            case 6:
                // This path transitions from secondPush to thirdSample (moving to third sample).
                timeout = y; // TODO: set appropriate timeout value for toThirdSample
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(toThirdSample, false);
                    setPathState(pathState + 1);
                }
                break;
            case 7:
                // This path transitions from thirdSample to preFarPickUp (pushing the third sample).
                timeout = y; // TODO: set appropriate timeout value for pushThirdSample
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(pushThirdSample, false);
                    setPathState(pathState + 1);
                }
                break;
            case 8:
                // This path transitions from preFarPickUp to farPickUp (first pickup).
                timeout = y; // TODO: set appropriate timeout value for firstPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(firstPickUp, true);
                    setPathState(pathState + 1);
                }
                break;
            case 9:
                // This path transitions from farPickUp to pre2ndDrop (preparing for second drop).
                timeout = y; // TODO: set appropriate timeout value for preSecondSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preSecondSpec, false);
                    setPathState(pathState + 1);
                }
                break;
            case 10:
                // This path transitions from pre2ndDrop to slow2ndDrop (second drop slowdown).
                timeout = y; // TODO: set appropriate timeout value for secondSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(secondSpec, true);
                    setPathState(pathState + 1);
                }
                break;
            case 11:
                // This path transitions from slow2ndDrop to preClosePickUp (preparing for second pickup).
                timeout = y; // TODO: set appropriate timeout value for preSecondPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preSecondPickUp, false);
                    setPathState(pathState + 1);
                }
                break;
            case 12:
                // This path transitions from preClosePickUp to closePickUp (second pickup).
                timeout = y; // TODO: set appropriate timeout value for secondPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(secondPickUp, true);
                    setPathState(pathState + 1);
                }
                break;
            case 13:
                // This path transitions from closePickUp to pre3rdDrop (preparing for third drop).
                timeout = y; // TODO: set appropriate timeout value for preThirdSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preThirdSpec, false);
                    setPathState(pathState + 1);
                }
                break;
            case 14:
                // This path transitions from pre3rdDrop to slow3rdDrop (third drop slowdown).
                timeout = y; // TODO: set appropriate timeout value for thirdSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(thirdSpec, true);
                    setPathState(pathState + 1);
                }
                break;
            case 15:
                // This path transitions from slow3rdDrop to preClosePickUp (preparing for third pickup).
                timeout = y; // TODO: set appropriate timeout value for preThirdPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preThirdPickUp, false);
                    setPathState(pathState + 1);
                }
                break;
            case 16:
                // This path transitions from preClosePickUp to closePickUp (third pickup).
                timeout = y; // TODO: set appropriate timeout value for thirdPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(thirdPickUp, true);
                    setPathState(pathState + 1);
                }
                break;
            case 17:
                // This path transitions from closePickUp to pre4thDrop (preparing for fourth drop).
                timeout = y; // TODO: set appropriate timeout value for preFourthSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preFourthSpec, false);
                    setPathState(pathState + 1);
                }
                break;
            case 18:
                // This path transitions from pre4thDrop to slow4thDrop (fourth drop slowdown).
                timeout = y; // TODO: set appropriate timeout value for fourthSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(fourthSpec, true);
                    setPathState(pathState + 1);
                }
                break;
            case 19:
                // This path transitions from slow4thDrop to preClosePickUp (preparing for fourth pickup).
                timeout = y; // TODO: set appropriate timeout value for preFourthPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preFourthPickUp, false);
                    setPathState(pathState + 1);
                }
                break;
            case 20:
                // This path transitions from preClosePickUp to closePickUp (fourth pickup).
                timeout = y; // TODO: set appropriate timeout value for fourthPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(fourthPickUp, true);
                    setPathState(pathState + 1);
                }
                break;
            case 21:
                // This path transitions from closePickUp to pre5thDrop (preparing for fifth drop).
                timeout = y; // TODO: set appropriate timeout value for preFithSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preFithSpec, false);
                    setPathState(pathState + 1);
                }
                break;
            case 22:
                // This path transitions from pre5thDrop to slow5thDrop (fifth drop slowdown).
                timeout = y; // TODO: set appropriate timeout value for fithSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(fithSpec, true);
                    setPathState(pathState + 1);
                }
                break;
            case 23:
                // This path transitions from slow5thDrop to preClosePickUp (preparing for fifth pickup).
                timeout = y; // TODO: set appropriate timeout value for preFithPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preFithPickUp, false);
                    setPathState(pathState + 1);
                }
                break;
            case 24:
                // This path transitions from preClosePickUp to closePickUp (fifth pickup).
                timeout = y; // TODO: set appropriate timeout value for fithPickUp
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(fithPickUp, true);
                    setPathState(pathState + 1);
                }
                break;
            case 25:
                // This path transitions from closePickUp to pre6thDrop (preparing for sixth drop).
                timeout = y; // TODO: set appropriate timeout value for preSixthSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(preSixthSpec, false);
                    setPathState(pathState + 1);
                }
                break;
            case 26:
                // This path transitions from pre6thDrop to slow6thDrop (sixth drop slowdown).
                timeout = y; // TODO: set appropriate timeout value for sixthSpec
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 0.3; // Slowdown is 0.3
                    follower.followPath(sixthSpec, true);
                    setPathState(pathState + 1);
                }
                break;
            case 27:
                // This path transitions from slow6thDrop to parkPose (parking pose).
                timeout = y; // TODO: set appropriate timeout value for park
                if (!follower.isBusy() || waitingTimer + timeout <= getRuntime()) {
                    slowdown = 1.0; // Slowdown is 1.0
                    follower.followPath(park, false);
                    setPathState(pathState + 1);
                }
                break;
        }


    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        // If any old threads or scheduler exist, shut them down to avoid duplicate threads.
        shutdownThread();
        if (scheduler != null && !scheduler.isShutdown()) {
            scheduler.shutdownNow();
        }
        scheduler = Executors.newScheduledThreadPool(1);

        telemetry.update();

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

        // Define the runnable for async updates
        Runnable asyncUpdates = new Runnable() {
            @Override
            public void run() {
                // Use a try-catch block to avoid unexpected thread termination.
                try {
                    while (asyncThread && !Thread.currentThread().isInterrupted()) {
                        horizontalSlides.update();
                        verticalSlides.update();
                        outtake.update();
                        intake.update();
                        // Uncomment the next line if you need to clear bulk cache
                        // expansionHub.clearBulkCache();
                    }
                } catch (Exception e) {
                    // Log or handle exception if needed
                }
            }
        };

        // Create and configure the thread.
        asyncThread = true;
        asyncUpdatesThread = new Thread(asyncUpdates);
        asyncUpdatesThread.setDaemon(true); // Optional: marks the thread as daemon.
        asyncUpdatesThread.start();

        // Initialize intake and outtake positions.
        intake.setRotation(TRANSFER);
        intake.setInnerBlockOpen(false);
        intake.setOuterBlockOpen(true);
        horizontalSlides.setPosition(1);
        outtake.hookAtIntake(false, true);
        outtake.clawOpen(false);
        outtake.specDropAtIntakePos(true);
        outtake.specDropOpen(false);
        verticalSlides.setPosition(0);
    }

    @Override
    public void loop() {
        follower.setMaxPower(slowdown);
        follower.update();
        autonomousPathUpdate();
    }

    @Override
    public void init_loop() {
        // Can add code to update telemetry or diagnostics here if needed.
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        shutdownThread();
        if (scheduler != null && !scheduler.isShutdown()) {
            scheduler.shutdownNow();
        }
    }

    private void delayedRun(Runnable action, long delayInMillis) {
        scheduler.schedule(action, delayInMillis, TimeUnit.MILLISECONDS);
    }

    // Method to properly stop the asyncUpdatesThread.
    private void shutdownThread() {
        asyncThread = false; // Signal the thread to stop.
        if (asyncUpdatesThread != null && asyncUpdatesThread.isAlive()) {
            asyncUpdatesThread.interrupt(); // Interrupt the thread.
            try {
                // Wait for the thread to finish (max 1 second).
                asyncUpdatesThread.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Preserve the interruption status.
            }
        }
    }
}
