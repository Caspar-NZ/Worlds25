package pedroPathing.examples;

import static org.firstinspires.ftc.teamcode.functions.intake.RotationMode.TRANSFER;

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

import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "testAutjghvhjo", group = "Examples")
public class TestV2 extends OpMode {

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
    private final Pose prescorepos = new Pose(35, 77.2, Math.toRadians(0));
    private final Pose scorePose = new Pose(41, 77.2, Math.toRadians(0));
    private final Pose backoff = new Pose(36, 50, Math.toRadians(0));
    private final Pose sample1 = new Pose(42.2, 36, Math.toRadians(0));
    private final Pose sample1back = new Pose(16, 27, Math.toRadians(0));
    private final Pose sample2 = new Pose(34.0, 27, Math.toRadians(0));
    private final Pose sample2T = new Pose(43, 21.8, Math.toRadians(0));
    private final Pose sample2B = new Pose(16, 18.0, Math.toRadians(0));
    private final Pose sample3 = new Pose(35.0, 17.4, Math.toRadians(0));
    private final Pose sample3T = new Pose(43, 13, Math.toRadians(0));
    private final Pose sample3B = new Pose(8, 13, Math.toRadians(0));

    private final Pose drop1Aid = new Pose(35.5, 72.0, Math.toRadians(0));
    private final Pose drop1 = new Pose(41.5, 72.0, Math.toRadians(0));

    private final Pose pickup2Aid = new Pose(13.9, 47.5, Math.toRadians(0));
    private final Pose pickup2 = new Pose(8.9, 47.5, Math.toRadians(0));

    private final Pose drop2Aid = new Pose(35.0, 70.0, Math.toRadians(0));
    private final Pose drop2 = new Pose(41.0, 70.0, Math.toRadians(0));

    private final Pose pickup3Aid = new Pose(13.5, 47.5, Math.toRadians(0));
    private final Pose pickup3 = new Pose(8.9, 47.5, Math.toRadians(0));

    private final Pose drop3Aid = new Pose(35.0, 70.0, Math.toRadians(0));
    private final Pose drop3 = new Pose(41.0, 70.0, Math.toRadians(0));

    private final Pose pickup4Aid = new Pose(13.5, 47.5, Math.toRadians(0));
    private final Pose pickup4 = new Pose(8.9, 47.5, Math.toRadians(0));

    private final Pose drop4Aid = new Pose(35.0, 70.0, Math.toRadians(0));
    private final Pose drop4 = new Pose(41, 70.0, Math.toRadians(0));

    private final Pose pickup5Aid = new Pose(13.5, 47.5, Math.toRadians(0));
    private final Pose pickup5 = new Pose(8.9, 47.5, Math.toRadians(0));

    private final Pose drop5Aid = new Pose(35.0, 70.0, Math.toRadians(0));
    private final Pose drop5 = new Pose(41, 70.0, Math.toRadians(0));

    private final Pose line20 = new Pose(8.9, 47.0, Math.toRadians(0));

    // Paths and PathChains
    private Path scorePreload, park;
    private PathChain firstDo, firstAdd, sample, zero, first, second,  third, fourth, fifth, sixth, seventh, eighth, ninth, tenth, eleventh, twelfth, thirteenth, fourteenth, fifteenth, sixteenth, seventeenth;

    public void buildPaths() {
        firstDo = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(prescorepos)))
                .setLinearHeadingInterpolation(startPose.getHeading(), prescorepos.getHeading())
                .build();

        firstAdd = follower.pathBuilder()
                .addPath(new BezierLine(new Point(prescorepos), new Point(scorePose)))
                .setLinearHeadingInterpolation(prescorepos.getHeading(), scorePose.getHeading())
                .build();

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

        zero = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3B), new Point(drop1Aid)))
                .setLinearHeadingInterpolation(sample3B.getHeading(), drop1Aid.getHeading())
                .build();

        first  = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop1Aid), new Point(drop1)))
                .setLinearHeadingInterpolation(drop1Aid.getHeading(), drop1.getHeading())
                .build();
        second = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop1), new Point(pickup2Aid)))
                .setLinearHeadingInterpolation(drop1.getHeading(), pickup2Aid.getHeading())
                .build();

        third = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Aid), new Point(pickup2)))
                .setLinearHeadingInterpolation(pickup2Aid.getHeading(), pickup2.getHeading())
                .build();

        fourth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2), new Point(drop2Aid)))
                .setLinearHeadingInterpolation(pickup2.getHeading(), drop2Aid.getHeading())
                .build();

        fifth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop2Aid), new Point(drop2)))
                .setLinearHeadingInterpolation(drop2Aid.getHeading(), drop2.getHeading())
                .build();

        sixth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop2), new Point(pickup3Aid)))
                .setLinearHeadingInterpolation(drop2.getHeading(), pickup3Aid.getHeading())
                .build();

        seventh = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Aid), new Point(pickup3)))
                .setLinearHeadingInterpolation(pickup3Aid.getHeading(), pickup3.getHeading())
                .build();

        eighth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3), new Point(drop3Aid)))
                .setLinearHeadingInterpolation(pickup3.getHeading(), drop3Aid.getHeading())
                .build();

        ninth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop3Aid), new Point(drop3)))
                .setLinearHeadingInterpolation(drop3Aid.getHeading(), drop3.getHeading())
                .build();

        tenth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop3), new Point(pickup4Aid)))
                .setLinearHeadingInterpolation(drop3.getHeading(), pickup4Aid.getHeading())
                .build();

        eleventh = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup4Aid), new Point(pickup4)))
                .setLinearHeadingInterpolation(pickup4Aid.getHeading(), pickup4.getHeading())
                .build();

        twelfth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup4), new Point(drop4Aid)))
                .setLinearHeadingInterpolation(pickup4.getHeading(), drop4Aid.getHeading())
                .build();

        thirteenth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop4Aid), new Point(drop4)))
                .setLinearHeadingInterpolation(drop4Aid.getHeading(), drop4.getHeading())
                .build();

        fourteenth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop4), new Point(pickup5Aid)))
                .setLinearHeadingInterpolation(drop4.getHeading(), pickup5Aid.getHeading())
                .build();

        fifteenth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup5Aid), new Point(pickup5)))
                .setLinearHeadingInterpolation(pickup5Aid.getHeading(), pickup5.getHeading())
                .build();

        sixteenth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup5), new Point(drop5Aid)))
                .setLinearHeadingInterpolation(pickup5.getHeading(), drop5Aid.getHeading())
                .build();

        seventeenth = follower.pathBuilder()
                .addPath(new BezierLine(new Point(drop5Aid), new Point(drop5)))
                .setLinearHeadingInterpolation(drop5Aid.getHeading(), drop5.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                slowdown = 1.0;
                delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 300), 0);
                follower.followPath(firstDo, false);
                outtake.hookAtIntake(false, false);
                setPathState(1);
                waitingTimer = getRuntime();
                break;

            case 1:
                if (!follower.isBusy() || waitingTimer + 0.8 <= getRuntime()) {
                    slowdown = 0.3;
                    follower.followPath(firstAdd, true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 350);
                    delayedRun(() -> outtake.clawOpen(true), 700);
                    setPathState(2);
                    waitingTimer = getRuntime();
                }

            case 2:
                follower.setMaxPower(100);
                if (waitingTimer + 0 < getRuntime()) {
                    if (!follower.isBusy()) {
                        slowdown = 1.0;
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 1000);
                        delayedRun(() -> outtake.hookAtIntake(true, false), 1000);
                        follower.followPath(sample, false);
                        setPathState(3);
                        waitingTimer = getRuntime();
                    }
                }
                break;


            case 3:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(zero, false);
                        setPathState(4);
                    }
                }
                break;

            case 4:
                if (!follower.isBusy() || waitingTimer + 0.8 <= getRuntime()) {
                    slowdown = 0.3;
                    follower.followPath(firstAdd, true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 350);
                    delayedRun(() -> outtake.clawOpen(true), 700);
                    follower.followPath(first, false);
                    setPathState(5);
                }
                break; // Note: no break here, falling through might be intentional or not
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(second, false);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(third, false);
                        setPathState(7);
                    }
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(zero, false);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                if (!follower.isBusy() || waitingTimer + 0.8 <= getRuntime()) {
                    slowdown = 0.3;
                    follower.followPath(firstAdd, true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 350);
                    delayedRun(() -> outtake.clawOpen(true), 700);
                    follower.followPath(first, false);
                    setPathState(9);
                }
                break; // Note: no break here, falling through might be intentional or not
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(second, false);
                    setPathState(10);
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(third, false);
                        setPathState(11);
                    }
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(zero, false);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (!follower.isBusy() || waitingTimer + 0.8 <= getRuntime()) {
                    slowdown = 0.3;
                    follower.followPath(firstAdd, true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 350);
                    delayedRun(() -> outtake.clawOpen(true), 700);
                    follower.followPath(first, false);
                    setPathState(13);
                }
                break; // Note: no break here, falling through might be intentional or not
            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(second, false);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(third, false);
                        setPathState(15);
                    }
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(zero, false);
                        setPathState(16);
                    }
                }
                break;

            case 16:
                if (!follower.isBusy() || waitingTimer + 0.8 <= getRuntime()) {
                    slowdown = 0.3;
                    follower.followPath(firstAdd, true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 350);
                    delayedRun(() -> outtake.clawOpen(true), 700);
                    follower.followPath(first, false);
                    setPathState(17);
                }
                break; // Note: no break here, falling through might be intentional or not
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(second, false);
                    setPathState(18);
                }
                break;

            case 18:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(third, false);
                        setPathState(19);
                    }
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(zero, false);
                        setPathState(20);
                    }
                }
                break;

            case 20:
                if (!follower.isBusy() || waitingTimer + 0.8 <= getRuntime()) {
                    slowdown = 0.3;
                    follower.followPath(firstAdd, true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 350);
                    delayedRun(() -> outtake.clawOpen(true), 700);
                    follower.followPath(first, false);
                    setPathState(21);
                }
                break; // Note: no break here, falling through might be intentional or not
            case 21:
                if (!follower.isBusy()) {
                    follower.followPath(second, false);
                    setPathState(22);
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.0 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(third, false);
                        setPathState(-1);
                    }
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
