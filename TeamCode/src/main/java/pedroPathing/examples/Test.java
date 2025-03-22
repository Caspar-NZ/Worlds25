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

@Autonomous(name = "testAuto", group = "Examples")
public class Test extends OpMode {

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

    // Paths and PathChains
    private Path scorePreload, park;
    private PathChain firstDo, firstAdd, sample, third, fourth, fifth, sixth, seventh;

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

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                slowdown = 1.0;
                delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 300), 0);
                follower.followPath(firstDo, false);
                outtake.hookAtIntake(false, false);
                setPathState(17);
                waitingTimer = getRuntime();
                break;
            case 1:
                follower.setMaxPower(100);
                if (waitingTimer + 0 < getRuntime()) {
                    if (!follower.isBusy()) {
                        slowdown = 1.0;
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 1), 1000);
                        delayedRun(() -> outtake.hookAtIntake(true, false), 1000);
                        follower.followPath(sample, false);
                        setPathState(2);
                        waitingTimer = getRuntime();
                    }
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    if (waitingTimer + 0.2 < getRuntime()) {
                        delayedRun(() -> outtake.clawOpen(false), 0);
                        delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 465), 50);
                        delayedRun(() -> outtake.hookAtIntake(false, false), 500);
                        follower.followPath(third, false);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(fourth, false);
                    setPathState(4);
                }
                break; // Note: no break here, falling through might be intentional or not
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(fifth, false);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(sixth, false);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(seventh, false);
                    setPathState(7);
                }
                break;
            case 17:
                if (!follower.isBusy() || waitingTimer + 0.8 <= getRuntime()) {
                    slowdown = 0.3;
                    follower.followPath(firstAdd, true);
                    delayedRun(() -> verticalSlides.setPosition(verticalSlides.MIN_POSITION + 650), 350);
                    delayedRun(() -> outtake.clawOpen(true), 700);
                    setPathState(1);
                    waitingTimer = getRuntime();
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
