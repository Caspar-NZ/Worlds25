package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.teamcode.functions.intake.RotationMode.INTAKE;
import static org.firstinspires.ftc.teamcode.functions.intake.RotationMode.TRANSFER;
import static org.firstinspires.ftc.teamcode.functions.intake.RotationMode.TUCKED;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.functions.AllianceColour;
import org.firstinspires.ftc.teamcode.functions.AllianceInfo;
import org.firstinspires.ftc.teamcode.functions.TargetState;
import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Samples", group = "Worlds", preselectTeleOp="TeleOp")
public class Samples extends OpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    private double timeout = 0;
    private boolean rejecting = false;
    private double rejectionEndTime = 0.0;  // time when a non-target piece was last detected

    // Go Home (target delivery) state variables
    private boolean goingHome = false;

    private double slowdown = 1.0;

    //private LynxModule expansionHub;

    private horiSlides horizontalSlides;
    private vertSlide verticalSlides;
    private outtake outtake;
    private intake intake;


    private ScheduledExecutorService scheduler;

    private double waitingTimer;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    double sampleTimer = 0.0;

    boolean sampleCollected = false;
    boolean timeoutCollection = false;
    TargetState thisIntake;
    boolean doubleDoubleTarget = false;
    boolean doubleDouble = false;

    double slideTarget = 140;

    double xOffset = 0;
    double yOffset = 0;
    boolean readyToIntake = false;
    // Define all poses and paths
    boolean initLoop = true;
    boolean firstLoop = true;
    double lastYOffset = yOffset;

    PathChain firstDelivery, firstCollection, secondDelivery, secondCollection, thirdDelivery, thirdCollection, fourthDelivery, goingToSub, preSub, firstSubPickup, goingToBuckets, preFifthDropLoc, slowFifthDrop, goingToSub2nd, preSub2nd, secondSubPickup, goingToBuckets2nd, preSixthDropLoc, slowSixthDrop, goingToSub3rd, preSub3rd, thirdSubPickup, goingToBuckets3rd, preSeventhDropLoc, slowSeventhDrop;
    Pose startPose, firstDrop, firstPickUp, secondDrop, secondPickUp, thirdDrop, thirdPickup, fourthDrop, towardsSub, preFirstSubCollection, firstSubCollection, towardsBuckets, preFifthDrop, fifthDrop, towardsSubSecondTrip, preSecondSubCollection, secondSubCollection, towardsBucketsSecondTrip, preSixthDrop, sixthDrop, towardsSubThirdTrip, preThirdSubCollection, thirdSubCollection, towardsBucketsThirdTrip, preSeventhDrop, seventhDrop;

    public void buildPaths() {
        startPose= new Pose(7.3, 112, Math.toRadians(0));
        firstDrop = new Pose(7.3, 130, Math.toRadians(0));
        firstPickUp = new Pose(24, 120, Math.toRadians(0));
        secondDrop = new Pose(7.3, 130, Math.toRadians(0));
        secondPickUp = new Pose(24, 130, Math.toRadians(0));
        thirdDrop = new Pose(7.3, 130, Math.toRadians(0));
        thirdPickup = new Pose(28, 128, Math.toRadians(35));
        fourthDrop = new Pose(7.3, 130, Math.toRadians(0));

        towardsSub = new Pose(60, 108, Math.toRadians(-90));
        preFirstSubCollection = new Pose(60, 100, Math.toRadians(-90));
        firstSubCollection = new Pose(60, 95, Math.toRadians(-90));
        towardsBuckets = new Pose(60, 108, Math.toRadians(-90));
        preFifthDrop = new Pose(24, 124, Math.toRadians(0));
        fifthDrop = new Pose(7.3, 130, Math.toRadians(0));

        towardsSubSecondTrip = new Pose(60, 108, Math.toRadians(-90));
        preSecondSubCollection = new Pose(60, 100, Math.toRadians(-90));
        secondSubCollection = new Pose(60, 95, Math.toRadians(-90));
        towardsBucketsSecondTrip = new Pose(60, 108, Math.toRadians(-90));
        preSixthDrop = new Pose(24, 124, Math.toRadians(0));
        sixthDrop = new Pose(7.3, 130, Math.toRadians(0));

        towardsSubThirdTrip = new Pose(60, 108, Math.toRadians(-90));
        preThirdSubCollection = new Pose(60, 100, Math.toRadians(-90));
        thirdSubCollection = new Pose(60, 95, Math.toRadians(-90));
        towardsBucketsThirdTrip = new Pose(60, 108, Math.toRadians(-90));
        preSeventhDrop = new Pose(24, 124, Math.toRadians(0));
        seventhDrop = new Pose(7.3, 130, Math.toRadians(0));

        // Paths and PathChains
        firstDelivery = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(firstDrop)))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstDrop.getHeading())
                .build();
        firstCollection = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstDrop), new Point(firstPickUp)))
                .setLinearHeadingInterpolation(firstDrop.getHeading(), firstPickUp.getHeading())
                .build();
        secondDelivery = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPickUp), new Point(secondDrop)))
                .setLinearHeadingInterpolation(firstPickUp.getHeading(), secondDrop.getHeading())
                .build();
        secondCollection = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondDrop), new Point(secondPickUp)))
                .setLinearHeadingInterpolation(secondDrop.getHeading(), secondPickUp.getHeading())
                .build();
        thirdDelivery = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPickUp), new Point(thirdDrop)))
                .setLinearHeadingInterpolation(secondPickUp.getHeading(), thirdDrop.getHeading())
                .build();
        thirdCollection = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdDrop), new Point(thirdPickup)))
                .setLinearHeadingInterpolation(thirdDrop.getHeading(), thirdPickup.getHeading())
                .build();
        fourthDelivery = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPickup), new Point(fourthDrop)))
                .setLinearHeadingInterpolation(thirdPickup.getHeading(), fourthDrop.getHeading())
                .build();
        goingToSub = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fourthDrop), new Point(towardsSub)))
                .setLinearHeadingInterpolation(fourthDrop.getHeading(), towardsSub.getHeading())
                .build();
        preSub = follower.pathBuilder()
                .addPath(new BezierLine(new Point(towardsSub), new Point(preFirstSubCollection)))
                .setLinearHeadingInterpolation(towardsSub.getHeading(), preFirstSubCollection.getHeading())
                .build();
        firstSubPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preFirstSubCollection), new Point(firstSubCollection)))
                .setLinearHeadingInterpolation(preFirstSubCollection.getHeading(), firstSubCollection.getHeading())
                .build();
        goingToBuckets = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSubCollection), new Point(towardsBuckets)))
                .setLinearHeadingInterpolation(firstSubCollection.getHeading(), towardsBuckets.getHeading())
                .build();
        preFifthDropLoc = follower.pathBuilder()
                .addPath(new BezierLine(new Point(towardsBuckets), new Point(preFifthDrop)))
                .setLinearHeadingInterpolation(towardsBuckets.getHeading(), preFifthDrop.getHeading())
                .build();
        slowFifthDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preFifthDrop), new Point(fifthDrop)))
                .setLinearHeadingInterpolation(preFifthDrop.getHeading(), fifthDrop.getHeading())
                .build();
        goingToSub2nd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(fifthDrop), new Point(towardsSubSecondTrip)))
                .setLinearHeadingInterpolation(fifthDrop.getHeading(), towardsSubSecondTrip.getHeading())
                .build();
        preSub2nd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(towardsSubSecondTrip), new Point(preSecondSubCollection)))
                .setLinearHeadingInterpolation(towardsSubSecondTrip.getHeading(), preSecondSubCollection.getHeading())
                .build();
        secondSubPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preSecondSubCollection), new Point(secondSubCollection)))
                .setLinearHeadingInterpolation(preSecondSubCollection.getHeading(), secondSubCollection.getHeading())
                .build();
        goingToBuckets2nd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSubCollection), new Point(towardsBucketsSecondTrip)))
                .setLinearHeadingInterpolation(secondSubCollection.getHeading(), towardsBucketsSecondTrip.getHeading())
                .build();
        preSixthDropLoc = follower.pathBuilder()
                .addPath(new BezierLine(new Point(towardsBucketsSecondTrip), new Point(preSixthDrop)))
                .setLinearHeadingInterpolation(towardsBucketsSecondTrip.getHeading(), preSixthDrop.getHeading())
                .build();
        slowSixthDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preSixthDrop), new Point(sixthDrop)))
                .setLinearHeadingInterpolation(preSixthDrop.getHeading(), sixthDrop.getHeading())
                .build();
        goingToSub3rd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sixthDrop), new Point(towardsSubThirdTrip)))
                .setLinearHeadingInterpolation(sixthDrop.getHeading(), towardsSubThirdTrip.getHeading())
                .build();
        preSub3rd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(towardsSubThirdTrip), new Point(preThirdSubCollection)))
                .setLinearHeadingInterpolation(towardsSubThirdTrip.getHeading(), preThirdSubCollection.getHeading())
                .build();
        thirdSubPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preThirdSubCollection), new Point(thirdSubCollection)))
                .setLinearHeadingInterpolation(preThirdSubCollection.getHeading(), thirdSubCollection.getHeading())
                .build();
        goingToBuckets3rd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSubCollection), new Point(towardsBucketsThirdTrip)))
                .setLinearHeadingInterpolation(thirdSubCollection.getHeading(), towardsBucketsThirdTrip.getHeading())
                .build();
        preSeventhDropLoc = follower.pathBuilder()
                .addPath(new BezierLine(new Point(towardsBucketsThirdTrip), new Point(preSeventhDrop)))
                .setLinearHeadingInterpolation(towardsBucketsThirdTrip.getHeading(), preSeventhDrop.getHeading())
                .build();
        slowSeventhDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preSeventhDrop), new Point(seventhDrop)))
                .setLinearHeadingInterpolation(preSeventhDrop.getHeading(), seventhDrop.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            // CASE 0: Start firstDelivery immediately.
            // firstDelivery: from startPose (7.3,112) to firstDrop (7.3,130) – moving along Y.
            case 0:
                slowdown = 0.5;
                follower.followPath(firstDelivery, true);
                verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1169);
                horizontalSlides.setPosition(150);
                outtake.sampleAtIntakePos(false);
                outtake.sampleReleaseOpen(false);
                setPathState(pathState + 1);
                outtake.specDropAtIntakePos(false);
                break;

            // CASE 1: While firstDelivery is running, cancel early when Y >= 129.
            // Then start firstCollection (from firstDrop (7.3,130) to firstPickUp (24,120) – primarily along X).
            case 1:
                if (!follower.isBusy() || follower.getPose().getY() >= 129) {
                    outtake.sampleReleaseOpen(true);
                    delayedRun(() -> outtake.sampleReleaseOpen(false), 500);
                    delayedRun(() -> outtake.sampleAtIntakePos(true), 500);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 500);
                    delayedRun(() -> intake.setRotation(INTAKE), 500);
                    slowdown = 0.5;
                    follower.followPath(firstCollection, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 2: Check firstCollection (from (7.3,130) to (24,120)) – X increasing; cancel when X >= 23.
            // Then start secondDelivery.
            case 2:
                if (!follower.isBusy() || follower.getPose().getX() >= 23) {
                    if (!sampleCollected){
                        collectSample();
                    } else {
                        slowdown = 0.5;
                        follower.followPath(secondDelivery, true);
                        setPathState(pathState + 1);
                        timeout = getRuntime();
                    }
                }
                break;

            // CASE 3: Check secondDelivery (from firstPickUp (24,120) to secondDrop (7.3,130)) – X decreasing; cancel when X <= 8.3.
            // Then start secondCollection.
            case 3:
                if (!follower.isBusy() || follower.getPose().getX() <= 8.3) {
                    slowdown = 0.5;
                    follower.followPath(secondCollection, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 4: Check secondCollection (from secondDrop (7.3,130) to secondPickUp (24,130)) – X increasing; cancel when X >= 23.
            // Then start thirdDelivery.
            case 4:
                if (!follower.isBusy() || follower.getPose().getX() >= 23) {
                    slowdown = 0.5;
                    follower.followPath(thirdDelivery, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 5: Check thirdDelivery (from secondPickUp (24,130) to thirdDrop (7.3,130)) – X decreasing; cancel when X <= 8.3.
            // Then start thirdCollection.
            case 5:
                if (!follower.isBusy() || follower.getPose().getX() <= 8.3) {
                    slowdown = 0.5;
                    follower.followPath(thirdCollection, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 6: Check thirdCollection (from thirdDrop (7.3,130) to thirdPickup (28,128)) – X increasing; cancel when X >= 27.
            // Then start fourthDelivery.
            case 6:
                if (!follower.isBusy() || follower.getPose().getX() >= 27) {
                    slowdown = 0.5;
                    follower.followPath(fourthDelivery, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 7: Check fourthDelivery (from thirdPickup (28,128) to fourthDrop (7.3,130)) – X decreasing; cancel when X <= 8.3.
            // Then start goingToSub.
            case 7:
                if (!follower.isBusy() || follower.getPose().getX() <= 8.3) {
                    slowdown = 1.0;
                    follower.followPath(goingToSub, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 8: Check goingToSub (from fourthDrop (7.3,130) to towardsSub (60,108)) – X increasing; cancel when X >= 59.
            // Then start preSub.
            case 8:
                if (!follower.isBusy() || follower.getPose().getX() >= 59) {
                    slowdown = 1.0;
                    follower.followPath(preSub, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 9: Check preSub (from towardsSub (60,108) to preFirstSubCollection (60,100)) – Y decreasing; cancel when Y <= 101.
            // Then start firstSubPickup.
            case 9:
                if (!follower.isBusy() || follower.getPose().getY() <= 101) {
                    slowdown = 0.5;
                    follower.followPath(firstSubPickup, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 10: Check firstSubPickup (from preFirstSubCollection (60,100) to firstSubCollection (60,95)) – Y decreasing; cancel when Y <= 96.
            // Then start goingToBuckets.
            case 10:
                if (!follower.isBusy() || follower.getPose().getY() <= 96) {
                    slowdown = 1.0;
                    follower.followPath(goingToBuckets, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 11: Check goingToBuckets (from firstSubCollection (60,95) to towardsBuckets (60,108)) – Y increasing; cancel when Y >= 107.
            // Then start preFifthDropLoc.
            case 11:
                if (!follower.isBusy() || follower.getPose().getY() >= 107) {
                    slowdown = 1.0;
                    follower.followPath(preFifthDropLoc, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 12: Check preFifthDropLoc (from towardsBuckets (60,108) to preFifthDrop (24,124)) – X decreasing; cancel when X <= 25.
            // Then start slowFifthDrop.
            case 12:
                if (!follower.isBusy() || follower.getPose().getX() <= 25) {
                    slowdown = 0.5;
                    follower.followPath(slowFifthDrop, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 13: Check slowFifthDrop (from preFifthDrop (24,124) to fifthDrop (7.3,130)) – X decreasing; cancel when X <= 8.3.
            // Then start goingToSub2nd.
            case 13:
                if (!follower.isBusy() || follower.getPose().getX() <= 8.3) {
                    slowdown = 1.0;
                    follower.followPath(goingToSub2nd, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 14: Check goingToSub2nd (from fifthDrop (7.3,130) to towardsSubSecondTrip (60,108)) – X increasing; cancel when X >= 59.
            // Then start preSub2nd.
            case 14:
                if (!follower.isBusy() || follower.getPose().getX() >= 59) {
                    slowdown = 1.0;
                    follower.followPath(preSub2nd, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 15: Check preSub2nd (from towardsSubSecondTrip (60,108) to preSecondSubCollection (60,100)) – Y decreasing; cancel when Y <= 101.
            // Then start secondSubPickup.
            case 15:
                if (!follower.isBusy() || follower.getPose().getY() <= 101) {
                    slowdown = 0.5;
                    follower.followPath(secondSubPickup, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 16: Check secondSubPickup (from preSecondSubCollection (60,100) to secondSubCollection (60,95)) – Y decreasing; cancel when Y <= 96.
            // Then start goingToBuckets2nd.
            case 16:
                if (!follower.isBusy() || follower.getPose().getY() <= 96) {
                    slowdown = 1.0;
                    follower.followPath(goingToBuckets2nd, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 17: Check goingToBuckets2nd (from secondSubCollection (60,95) to towardsBucketsSecondTrip (60,108)) – Y increasing; cancel when Y >= 107.
            // Then start preSixthDropLoc.
            case 17:
                if (!follower.isBusy() || follower.getPose().getY() >= 107) {
                    slowdown = 1.0;
                    follower.followPath(preSixthDropLoc, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 18: Check preSixthDropLoc (from towardsBucketsSecondTrip (60,108) to preSixthDrop (24,124)) – X decreasing; cancel when X <= 25.
            // Then start slowSixthDrop.
            case 18:
                if (!follower.isBusy() || follower.getPose().getX() <= 25) {
                    slowdown = 0.5;
                    follower.followPath(slowSixthDrop, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 19: Check slowSixthDrop (from preSixthDrop (24,124) to sixthDrop (7.3,130)) – X decreasing; cancel when X <= 8.3.
            // Then start goingToSub3rd.
            case 19:
                if (!follower.isBusy() || follower.getPose().getX() <= 8.3) {
                    slowdown = 1.0;
                    follower.followPath(goingToSub3rd, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 20: Check goingToSub3rd (from sixthDrop (7.3,130) to towardsSubThirdTrip (60,108)) – X increasing; cancel when X >= 59.
            // Then start preSub3rd.
            case 20:
                if (!follower.isBusy() || follower.getPose().getX() >= 59) {
                    slowdown = 1.0;
                    follower.followPath(preSub3rd, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 21: Check preSub3rd (from towardsSubThirdTrip (60,108) to preThirdSubCollection (60,100)) – Y decreasing; cancel when Y <= 101.
            // Then start thirdSubPickup.
            case 21:
                if (!follower.isBusy() || follower.getPose().getY() <= 101) {
                    slowdown = 0.5;
                    follower.followPath(thirdSubPickup, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 22: Check thirdSubPickup (from preThirdSubCollection (60,100) to thirdSubCollection (60,95)) – Y decreasing; cancel when Y <= 96.
            // Then start goingToBuckets3rd.
            case 22:
                if (!follower.isBusy() || follower.getPose().getY() <= 96) {
                    slowdown = 1.0;
                    follower.followPath(goingToBuckets3rd, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 23: Check goingToBuckets3rd (from thirdSubCollection (60,95) to towardsBucketsThirdTrip (60,108)) – Y increasing; cancel when Y >= 107.
            // Then start preSeventhDropLoc.
            case 23:
                if (!follower.isBusy() || follower.getPose().getY() >= 107) {
                    slowdown = 1.0;
                    follower.followPath(preSeventhDropLoc, false);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 24: Check preSeventhDropLoc (from towardsBucketsThirdTrip (60,108) to preSeventhDrop (24,124)) – X decreasing; cancel when X <= 25.
            // Then start slowSeventhDrop.
            case 24:
                if (!follower.isBusy() || follower.getPose().getX() <= 25) {
                    slowdown = 0.5;
                    follower.followPath(slowSeventhDrop, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
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
        buildPaths();
        follower.setStartingPose(startPose);
        buildPaths();


        //buildPaths();

        horizontalSlides = new horiSlides(hardwareMap);
        verticalSlides = new vertSlide(hardwareMap);
        outtake = new outtake(hardwareMap);
        intake = new intake(hardwareMap);

        outtake.sampleReleaseOpen(false);
        outtake.setSampleOutOfWay();

        // Initialize intake and outtake positions.
        intake.setRotation(TUCKED);
        intake.setInnerBlockOpen(false);
        intake.setOuterBlockOpen(true);
        outtake.hookAtIntake(false, true);
        outtake.clawOpen(false);
        outtake.specDropAtIntakePos(true);
        outtake.specDropOpen(false);
        verticalSlides.setPosition(0);

        AllianceInfo.alliance = AllianceColour.RED;
        intake.setTarget(0, 1, 0);
        horizontalSlides.update();
        verticalSlides.update();
        outtake.update();
        intake.update();
    }

    @Override
    public void loop() {
        follower.setMaxPower(slowdown);
        follower.update();
        autonomousPathUpdate();
        horizontalSlides.update();
        verticalSlides.update();
        outtake.update();
        intake.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        if (scheduler != null && !scheduler.isShutdown()) {
            scheduler.shutdownNow();
        }
    }

    private void delayedRun(Runnable action, long delayInMillis) {
        scheduler.schedule(action, delayInMillis, TimeUnit.MILLISECONDS);
    }

    // Method to properly stop the asyncUpdatesThread.
    public void collectSample(){
        intake.setTarget(1, 0, 0);
        TargetState detectedColor = intake.getDetectedColor();
        TargetState previousIntake = thisIntake;
        thisIntake = detectedColor;

        doubleDouble = Objects.equals(previousIntake, thisIntake);

        doubleDoubleTarget = doubleDouble && intake.isTarget();

        double currentTime = getRuntime();

        if (!goingHome) {
            horizontalSlides.setPosition(horizontalSlides.getCurrentPosition() + 12);
            if (detectedColor != TargetState.NONE && detectedColor != TargetState.MIXED && doubleDouble) {
                if (doubleDoubleTarget) {
                    // TARGET PIECE: Stop intake immediately and start goHome sequence.
                    intake.setSpeed(0, 0);
                    goingHome = true;
                    rejecting = false;
                } else {
                    // NON-TARGET: Reject it.
                    // Force both blockers open so the piece passes through.
                    intake.setInnerBlockOpen(true);
                    intake.setOuterBlockOpen(true);
                    // Use full-speed rejection (in the same direction as the manual command).
                    intake.setSpeed(1, 1);
                    rejecting = true;
                    rejectionEndTime = currentTime;
                }
            } else {
                // No piece detected.
                if (rejecting) {
                    // Wait 0.2 sec after the piece disappears before restoring manual control.
                    if (currentTime - rejectionEndTime >= 0.05) {
                        intake.setInnerBlockOpen(false);
                        rejecting = false;
                    } else {
                        intake.setSpeed(1, 1);
                    }
                } else {
                    intake.setSpeed(1,1);

                }
            }
        } else {
            intake.setRotation(TRANSFER);
            horizontalSlides.setPosition(horizontalSlides.MIN_POSITION + 1);
            sampleCollected = true;
        }
        if ((horizontalSlides.getCurrentPosition() > (horizontalSlides.MIN_POSITION + 800)) || (sampleTimer+4.0 <getRuntime())) {
            intake.setRotation(TRANSFER);
            horizontalSlides.setPosition(horizontalSlides.MIN_POSITION + 1);
            if (detectedColor != TargetState.NONE && detectedColor != TargetState.MIXED && doubleDouble && !doubleDoubleTarget){
                intake.setTimedIntake(-1,-1,2);
            }
            timeoutCollection = true;
        }

    }

}