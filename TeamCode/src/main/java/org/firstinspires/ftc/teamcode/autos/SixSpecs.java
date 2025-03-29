package org.firstinspires.ftc.teamcode.autos;


import static org.firstinspires.ftc.teamcode.functions.intake.RotationMode.INTAKE;
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
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.functions.TargetState;
import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.vertSlide;
import org.firstinspires.ftc.teamcode.teleop.teleOp;

import java.util.List;
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "SixSpecs", group = "Worlds")
public class SixSpecs extends OpMode {
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    private double timeout = 0;
    private boolean rejecting = false;
    private double rejectionEndTime = 0.0;  // time when a non-target piece was last detected

    // Go Home (target delivery) state variables
    private boolean goingHome = false;
    public enum AllianceColor {
        RED,
        BLUE
    }
    private AllianceColor alliance = AllianceColor.RED;



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

    private final Pose startPose = new Pose(7.32, 77.2, Math.toRadians(0));
    private final Pose pre1stDrop = new Pose(36.6, 77.2, Math.toRadians(0));
    private final Pose slow1stDrop = new Pose(42, 77.2, Math.toRadians(0));
    private final Pose preFirstSample = new Pose(35, 54, Math.toRadians(0));//48
    private final Pose firstSample = new Pose(43, 35, Math.toRadians(0));//y32.5
    private final Pose firstPush = new Pose(16, 27, Math.toRadians(0));
    private final Pose secondSample = new Pose(43, 27, Math.toRadians(0));
    private final Pose preSecondPush = new Pose(43, 20.3, Math.toRadians(0));
    private final Pose secondPush = new Pose(16, 20.3, Math.toRadians(0));
    private final Pose thirdSample = new Pose(35.0, 19.9, Math.toRadians(0));
    private final Pose preThirdPush = new Pose(43, 14.7, Math.toRadians(0));
    private final Pose preFarPickUp = new Pose(13, 14.7, Math.toRadians(0)); // 16
    private final Pose farPickUp = new Pose(7.2, 14.7, Math.toRadians(0)); //16
    private final Pose pre2ndDrop = new Pose(36.0, (70.0 + yOffset), Math.toRadians(0));
    private final Pose slow2ndDrop = new Pose(43, (74.0 + yOffset), Math.toRadians(0));
    private final Pose preClosePickUpless2 = new Pose(14, 49, Math.toRadians(0));
    private final Pose closePickUpless2 = new Pose(8, 43, Math.toRadians(0));
    private final Pose preClosePickUp = new Pose(14, 51, Math.toRadians(0));
    private final Pose closePickUp = new Pose(7.5, 45, Math.toRadians(0));
    private final Pose closePickUpx6 = new Pose(7.1, 45, Math.toRadians(0));
    private final Pose closePickUpx65 = new Pose(7.5, 45, Math.toRadians(0));
    private final Pose pre3rdDrop = new Pose(36.0, 68.0, Math.toRadians(0));
    private final Pose slow3rdDrop = new Pose(41.5, 72, Math.toRadians(0));
    private final Pose pre4thDrop = new Pose(36.0, 67.0, Math.toRadians(0));
    private final Pose slow4thDrop = new Pose(41.5, 71, Math.toRadians(0));
    private final Pose pre5thDrop = new Pose(36.0, 66, Math.toRadians(0));
    private final Pose slow5thDrop = new Pose(41.5, 70, Math.toRadians(0));
    private final Pose pre6thDrop = new Pose(36.0, 65, Math.toRadians(0));
    private final Pose slow6thDrop = new Pose(41.5, 69, Math.toRadians(0));
    private final Pose parkPose = new Pose(30.0, 60.0, Math.toRadians(-135)); //was 10 40


    // Paths and PathChains
    private PathChain preFirstSpec, firstSpec, toFirstSample, pushFirstSample, toSecondSample, moveYTo2ndSample, pushSecondSample, toThirdSample, moveYTo3rdSample, pushThirdSample, firstPickUp, preSecondSpec, secondSpec, preSecondPickUp, secondPickUp, preThirdSpec, thirdSpec, preThirdPickUp, thirdPickUp, preFourthSpec, fourthSpec, preFourthPickUp, fourthPickUp, preFithSpec, fithSpec, preFithPickUp, fithPickUp, preSixthSpec, sixthSpec, park;


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
        moveYTo2ndSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSample), new Point(preSecondPush)))
                .setLinearHeadingInterpolation(secondSample.getHeading(), preSecondPush.getHeading())
                .build();
        pushSecondSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preSecondPush), new Point(secondPush)))
                .setLinearHeadingInterpolation(preSecondPush.getHeading(), secondPush.getHeading())
                .build();
        toThirdSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPush), new Point(thirdSample)))
                .setLinearHeadingInterpolation(secondPush.getHeading(), thirdSample.getHeading())
                .build();
        moveYTo3rdSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSample), new Point(preThirdPush)))
                .setLinearHeadingInterpolation(thirdSample.getHeading(), preThirdPush.getHeading())
                .build();
        pushThirdSample = follower.pathBuilder()
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
                .addPath(new BezierLine(new Point(slow2ndDrop), new Point(preClosePickUpless2)))
                .setLinearHeadingInterpolation(slow2ndDrop.getHeading(), preClosePickUpless2.getHeading())
                .build();
        secondPickUp = follower.pathBuilder()
                .addPath(new BezierLine(new Point(preClosePickUpless2), new Point(closePickUpless2)))
                .setLinearHeadingInterpolation(preClosePickUpless2.getHeading(), closePickUpless2.getHeading())
                .build();
        preThirdSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(closePickUpless2), new Point(pre3rdDrop)))
                .setLinearHeadingInterpolation(closePickUpless2.getHeading(), pre3rdDrop.getHeading())
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
                .addPath(new BezierLine(new Point(preClosePickUp), new Point(closePickUpx65)))
                .setLinearHeadingInterpolation(preClosePickUp.getHeading(), closePickUpx65.getHeading())
                .build();
        preFithSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(closePickUpx65), new Point(pre5thDrop)))
                .setLinearHeadingInterpolation(closePickUpx65.getHeading(), pre5thDrop.getHeading())
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
                .addPath(new BezierLine(new Point(preClosePickUp), new Point(closePickUpx6)))
                .setLinearHeadingInterpolation(preClosePickUp.getHeading(), closePickUpx6.getHeading())
                .build();
        preSixthSpec = follower.pathBuilder()
                .addPath(new BezierLine(new Point(closePickUpx6), new Point(pre6thDrop)))
                .setLinearHeadingInterpolation(closePickUpx6.getHeading(), pre6thDrop.getHeading())
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
        switch(pathState) {
            // CASE 0: Start high-speed path from startPose to pre1stDrop.
            case 0:
                slowdown = 1.0;
                follower.followPath(preFirstSpec, false);
                verticalSlides.setPosition(verticalSlides.MIN_POSITION + 440);
                outtake.hookAtIntake(false, false);
                setPathState(pathState + 1);
                break;

            // CASE 1: Monitor preFirstSpec.
            // Cancel early when the robot’s X reaches 34.5.
            case 1:
                if (!follower.isBusy() || follower.getPose().getX() >= 26.5) {
                    slowdown = 0.3;
                    follower.followPath(firstSpec, true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 550);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 2: Wait for firstSpec (slow speed) to complete.
            case 2:
                if (!follower.isBusy()) {
                    outtake.clawOpen(true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 100);
                    delayedRun(() -> outtake.hookAtIntake(true, false), 200);
                    slowdown = 1.0;
                    follower.followPath(toFirstSample, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 3: Monitor toFirstSample.
            // This diagonal path (dominant Y drop) is cancelled early when Y reaches 50.
            case 3:
                if (!follower.isBusy() || (follower.getPose().getY() <40)) {
                    slowdown = 1.0;
                    follower.followPath(pushFirstSample, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 4: Monitor pushFirstSample.
            // Movement mainly along X decreasing; cancel when X ≤ 22.
            case 4:
                if (!follower.isBusy() || follower.getPose().getX() <= 19.50) {
                    slowdown = 1.0;
                    follower.followPath(toSecondSample, false);
                    setPathState(pathState + 1);
                }
                break;
            case 5:
                if (!follower.isBusy() || follower.getPose().getX() >= 31.0) {
                    slowdown = 1.0;
                    follower.followPath(moveYTo2ndSample, false);
                    setPathState(pathState + 1);
                }
                break;
            // CASE 5: Monitor toSecondSample.
            // Movement along X increasing; cancel when X reaches about 33.
            case 6:
                if (!follower.isBusy() || (follower.getPose().getY() <= 26)) {
                    slowdown = 1.0;
                    follower.followPath(pushSecondSample, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 6: Monitor pushSecondSample.
            // Composite path ending near secondPush; cancel when X drops to ~20.
            case 7:
                if (!follower.isBusy() || follower.getPose().getX() <= 20.0) {
                    slowdown = 1.0;
                    follower.followPath(toThirdSample, false);
                    setPathState(pathState + 1);
                }
                break;

            case 8:
                if (!follower.isBusy() || follower.getPose().getX() >= 31.0) {
                    slowdown = 1.0;
                    follower.followPath(moveYTo3rdSample, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 7: Monitor toThirdSample.
            // Movement along X increasing; cancel when X reaches about 33.
            case 9:
                if (!follower.isBusy() || follower.getPose().getY() >= 15.5) {
                    slowdown = 1.0;
                    follower.followPath(pushThirdSample, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 8: Monitor pushThirdSample.
            // Composite path moving from thirdSample to preFarPickUp; cancel early when X is near 15.
            case 10:
                if (!follower.isBusy() || follower.getPose().getX() <= 16) {
                    slowdown = 0.3;
                    follower.followPath(firstPickUp, true);

                    delayedRun(() -> horizontalSlides.setPosition(horizontalSlides.MIN_POSITION + 1), 0);
                    delayedRun(() -> outtake.clawOpen(false), 100);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 9: Wait for firstPickUp (precision move) to complete.
            case 11:
                if (!follower.isBusy() || getRuntime() > timeout + 0.5) {
                    outtake.clawOpen(false);

                    delayedRun(() -> horizontalSlides.setPosition(horizontalSlides.MIN_POSITION + 1), 0);
                    delayedRun(() -> outtake.hookAtIntake(false, false), 70);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 440), 500);


                    slowdown = 1.0;
                    follower.followPath(preSecondSpec, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 10: Monitor preSecondSpec.
            // Diagonal move (dominant Y increase); cancel early when Y reaches about 60.

            case 12:
                if (!follower.isBusy() || follower.getPose().getX() >= 35.0) {
                    intake.setInnerBlockOpen(true);

                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 800);

                    delayedRun(() -> horizontalSlides.setPosition(horizontalSlides.MIN_POSITION + slideTarget +80), 0);
                    delayedRun(() -> intake.setAutoPos(), 350);
                    delayedRun(() -> horizontalSlides.setPosition(horizontalSlides.MIN_POSITION + slideTarget +120), 350);
                    delayedRun(() -> intake.setRotation(INTAKE), 650);
                    delayedRun(() -> intake.setInnerBlockOpen(false), 750);
                    delayedRun(() -> horizontalSlides.setPosition(horizontalSlides.MIN_POSITION + slideTarget), 550);
                    delayedRun(() -> readyToIntake = true,750);
                    delayedRun(() -> intake.setTimedIntake(-1, -1, 0.5),300);

                    intake.setTimedIntake(-1, -1, 0.4);
                    slowdown = 0.5;
                    follower.followPath(secondSpec, true);
                    setPathState(pathState + 1);
                }
                break;





            // CASE 11: Wait for secondSpec (slow speed) to complete.
            case 13:
                if (follower.getPose().getX() >= 37.0 && readyToIntake){
                    if (sampleTimer == 0.0) {
                        sampleTimer = getRuntime();
                    }
                    double currentTime = getRuntime();

                    TargetState detectedColor = intake.getDetectedColor();
                    TargetState previousIntake = thisIntake;
                    thisIntake = detectedColor;

                    doubleDouble = Objects.equals(previousIntake, thisIntake);

                    doubleDoubleTarget = doubleDouble && intake.isTarget();

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
                        timeoutCollection = true;
                    }

                    if (sampleCollected || timeoutCollection) {

                        outtake.clawOpen(true);
                        outtake.specDropOpen(false);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 100);
                        delayedRun(() -> outtake.hookAtIntake(true, false), 200);
                        delayedRun(() -> intake.setTimedIntake(-1, -1, 0.8), 600);
                        //delayedRun(() -> outtake.specDropAtIntakePos(false), 900);
                        delayedRun(() -> outtake.setSpecDropAtAuto(), 900);
                        slowdown = 1.0;
                        follower.followPath(preSecondPickUp, false);
                        setPathState(pathState + 1);
                    }
                }
                break;






            // CASE 12: Monitor preSecondPickUp.
            // Diagonal move with dominant X decrease; cancel when X ≤ 22.
            case 14:
                if (!follower.isBusy() || follower.getPose().getX() <= 16) {
                    outtake.specDropOpen(true);
                    outtake.setSpecDropAtAuto();
                    //outtake.specDropAtIntakePos(false);
                    delayedRun(() -> outtake.specDropAtIntakePos(true), 800);
                    delayedRun(() -> outtake.specDropOpen(false), 1000);
                    slowdown = 0.3;
                    follower.followPath(secondPickUp, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 13: Wait for secondPickUp (precision pickup) to complete.
            case 15:
                if (!follower.isBusy() || getRuntime() > timeout + 0.7) {
                    outtake.clawOpen(false);
                    delayedRun(() -> outtake.hookAtIntake(false, false), 70);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 440), 150);
                    slowdown = 1.0;
                    follower.followPath(preThirdSpec, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 14: Monitor preThirdSpec.
            // Diagonal move (dominant X increase); cancel when X reaches about 30.
            case 16:
                if (!follower.isBusy() || follower.getPose().getX() >= 35.0) {

                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 200);
                    slowdown = 0.3;
                    follower.followPath(thirdSpec, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 15: Wait for thirdSpec (slow speed) to complete.
            case 17:
                if (!follower.isBusy() || getRuntime() > timeout + 0.5) {

                    outtake.clawOpen(true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 100);
                    delayedRun(() -> outtake.hookAtIntake(true, false), 200);
                    slowdown = 1.0;
                    follower.followPath(preThirdPickUp, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 16: Monitor preThirdPickUp.
            // Diagonal move with dominant X decrease; cancel when X ≤ 22.
            case 18:
                if (!follower.isBusy() || follower.getPose().getX() <= 16) {
                    slowdown = 0.3;
                    follower.followPath(thirdPickUp, true);
                    delayedRun(() -> outtake.clawOpen(false), 600);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 17: Wait for thirdPickUp (precision pickup) to complete.
            case 19:
                if (!follower.isBusy() || getRuntime() > timeout + 0.7) {
                    outtake.clawOpen(false);
                    delayedRun(() -> outtake.hookAtIntake(false, false), 70);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 450), 150);
                    slowdown = 1.0;
                    follower.followPath(preFourthSpec, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 18: Monitor preFourthSpec.
            // Diagonal move (dominant X increase); cancel when X reaches about 30.
            case 20:
                if (!follower.isBusy() || follower.getPose().getX() >= 35.0) {

                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 200);
                    slowdown = 0.3;
                    follower.followPath(fourthSpec, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 19: Wait for fourthSpec (slow speed) to complete.
            case 21:
                if (!follower.isBusy() || getRuntime() > timeout + 0.5) {

                    outtake.clawOpen(true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 100);
                    delayedRun(() -> outtake.hookAtIntake(true, false), 200);
                    slowdown = 1.0;
                    follower.followPath(preFourthPickUp, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 20: Monitor preFourthPickUp.
            // Diagonal move with dominant X decrease; cancel when X ≤ 22.
            case 22:
                if (!follower.isBusy() || follower.getPose().getX() <= 15.8) {
                    slowdown = 0.3;
                    follower.followPath(fourthPickUp, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 21: Wait for fourthPickUp (precision pickup) to complete.
            case 23:
                if (!follower.isBusy() || getRuntime() > timeout + 0.7) {
                    outtake.clawOpen(false);
                    delayedRun(() -> outtake.hookAtIntake(false, false), 70);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 450), 150);
                    slowdown = 1.0;
                    follower.followPath(preFithSpec, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 22: Monitor preFithSpec.
            // Diagonal move (dominant X increase); cancel when X reaches about 30.
            case 24:
                if (!follower.isBusy() || follower.getPose().getX() >= 35.0) {
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 200);
                    slowdown = 0.3;
                    follower.followPath(fithSpec, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 23: Wait for fithSpec (slow speed) to complete.
            case 25:
                if (!follower.isBusy() || getRuntime() > timeout + 0.5) {
                    outtake.clawOpen(true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 100);
                    delayedRun(() -> outtake.hookAtIntake(true, false), 200);
                    slowdown = 1.0;
                    follower.followPath(preFithPickUp, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 24: Monitor preFithPickUp.
            // This path goes from preClosePickUp (X ≈ 12) toward closePickUp (X ≈ 8).
            // Cancel early when the robot’s X reaches 9.0 or less.
            case 26:
                if (!follower.isBusy() || follower.getPose().getX() <= 16.2) {
                    slowdown = 0.3;
                    follower.followPath(fithPickUp, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 25: Wait for fithPickUp (precision pickup) to complete.
            case 27:
                if (!follower.isBusy() || getRuntime() > timeout + 0.7) {
                    outtake.clawOpen(false);
                    delayedRun(() -> outtake.hookAtIntake(false, false), 70);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 450), 150);
                    slowdown = 1.0;
                    follower.followPath(preSixthSpec, false);
                    setPathState(pathState + 1);
                }
                break;

            // CASE 26: Monitor preSixthSpec.
            // Diagonal move (dominant X increase); cancel when X reaches about 30.
            case 28:
                if (!follower.isBusy() || follower.getPose().getX() >= 34.0) {
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 200);
                    slowdown = 0.3;
                    follower.followPath(sixthSpec, true);
                    setPathState(pathState + 1);
                    timeout = getRuntime();
                }
                break;

            // CASE 27: Wait for sixthSpec (slow speed) to complete, then start the parking path.
            case 29:
                if (!follower.isBusy() || getRuntime() > timeout + 0.5) {
                    outtake.clawOpen(true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 100);
                    delayedRun(() -> outtake.hookAtIntake(true, false), 200);
                    delayedRun(() -> horizontalSlides.setPosition(horizontalSlides.MIN_POSITION + 800), 600);
                    slowdown = 1.0;
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
        // Can add code to update telemetry or diagnostics here if needed.
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        double currentxOffset = xOffset;
        double currentyOffset = yOffset;

            // Toggle alliance on touchpad press.
        if ((currentGamepad2.touchpad && !previousGamepad2.touchpad) ||
                (currentGamepad1.touchpad && !previousGamepad1.touchpad)) {
            if (intake.target2 == TargetState.RED) {
                intake.setTarget(0, 0, 1);
                alliance = AllianceColor.BLUE;
            } else {
                intake.setTarget(0, 1, 0);
                alliance = AllianceColor.RED;
            }
        }
        // Increase xOffset (dpad up) if not exceeding 20
        if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) ||
                (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)) {
            if (xOffset < 20) {
                xOffset += 0.5;
            }
        }

        // Decrease xOffset (dpad down) if not below 0
        if ((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) ||
                (currentGamepad2.dpad_down && !previousGamepad2.dpad_down)) {
            if (xOffset > 0) {
                xOffset -= 0.5;
            }
        }

        // Increase yOffset (dpad right) if not exceeding +4
        if ((currentGamepad1.dpad_right && !previousGamepad1.dpad_right) ||
                (currentGamepad2.dpad_right && !previousGamepad2.dpad_right)) {
            if (yOffset < 4) {
                yOffset += 0.5;
            }
        }

        // Decrease yOffset (dpad left) if not below -4
        if ((currentGamepad1.dpad_left && !previousGamepad1.dpad_left) ||
                (currentGamepad2.dpad_left && !previousGamepad2.dpad_left)) {
            if (yOffset > -4) {
                yOffset -= 0.5;
            }
        }

        slideTarget = 70 + (xOffset * 30);

        if (currentxOffset != xOffset || currentyOffset != yOffset){
            follower = new Follower(hardwareMap);
            follower.setStartingPose(startPose);
            buildPaths();
        }

        telemetry.addData("yOffset", yOffset);
        telemetry.addData("xOffset", xOffset);
        telemetry.addData("Alliance", alliance.toString());
        telemetry.update();
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

}
