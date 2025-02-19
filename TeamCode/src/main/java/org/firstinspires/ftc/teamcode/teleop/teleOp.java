package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.functions.intake.RotationMode.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.List;
import java.util.Objects;

import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

@TeleOp
public class teleOp extends LinearOpMode {

    private static final double MAX_INPUT_SCALING = 200;
    private static final double JOYSTICK_DEADZONE = 0.05;

    // Slide target position (now fixed numbers: 400, 200, 0, etc.)
    private double horiSlidesTarget = 0;
    private double vertSlidesTarget = 0;

    // Gamepad state copies
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // Loop timing
    double previousStartTime = 0;
    double loopTime = 0;

    // Rejection state variables
    private boolean rejecting = false;
    private double rejectionEndTime = 0.0;  // time when a non-target piece was last detected

    // Go Home (target delivery) state variables
    private boolean goingHome = false;
    private double goHomeWaitStart = 0; // used when slide is below 200: wait 0.4 sec before retracting

    private double specBucketRunTimer = 0.0;
    private boolean specRunTimeRunning = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set bulk caching mode for all hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ------------------- DRIVE MOTOR INITIALIZATION -------------------
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack  = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront= hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // ------------------- SUBSYSTEM INITIALIZATION -------------------
        horiSlides horizontalSlides = new horiSlides(hardwareMap);
        vertSlide verticalSlides = new vertSlide(hardwareMap);
        outtake outtake = new outtake(hardwareMap);
        intake intake = new intake(hardwareMap);

        // Initialize intake and outtake positions.
        // Default manual configuration (right trigger): inner blocker closed, outer blocker open.
        intake.setRotation(TUCKED);
        intake.setInnerBlockOpen(false);
        intake.setOuterBlockOpen(true);
        // Maintain target values continuously.
        intake.setTarget(1, 0, 0);
        intake.update();

        outtake.hookAtIntake(true);
        outtake.clawOpen(true);
        outtake.specDropAtIntakePos(true);
        outtake.specDropOpen(false);
        outtake.update();

        waitForStart();

        while (opModeIsActive()) {
            loopTime = getRuntime() - previousStartTime;
            previousStartTime = getRuntime();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Always maintain intake target.
            intake.setTarget(1, 0, 0);

            // ------------------- DRIVE CONTROL -------------------
            double y  = -currentGamepad1.left_stick_y;
            double x  = currentGamepad1.left_stick_x * 1.1;
            double rx = currentGamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower  = (y + x + rx) / denominator;
            double leftBackPower   = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower  = (y + x - rx) / denominator;
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // ------------------- RIGHT BUMPER TO TOGGLE INTAKE DEPLOYMENT -------------------
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                if (horizontalSlides.getCurrentPosition()>120) {

                    if (intake.getRotationMode().equalsIgnoreCase("TUCKED")) {
                        intake.setRotation(INTAKE);
                        intake.setTimedIntake(-1, -1, 0.4);
                        intake.setInnerBlockOpen(true);
                        intake.setOuterBlockOpen(true);
                    } else {
                        // If already deployed, then if nothing is detected, tuck the intake.
                        if (Objects.equals(intake.getDetectedColor(), "NA")) {
                            if (intake.getRotationMode().equalsIgnoreCase("TRANSFER")) {
                                intake.setRotation(INTAKE);
                            } else {
                                intake.setRotation(TUCKED);
                            }
                        } else {
                            // Otherwise, toggle between INTAKE and TRANSFER.
                            if (intake.getRotationMode().equalsIgnoreCase("INTAKE"))
                                intake.setRotation(TRANSFER);
                            else
                                intake.setRotation(INTAKE);
                            // Keep both blockers open while a piece is present.
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(true);
                        }
                    }
                } else {
                    horiSlidesTarget = 120;
                }

            }

            // ------------------- MAIN INTAKE LOGIC (when not in goHome sequence) -------------------
            if (!goingHome && intake.getRotationMode().equalsIgnoreCase("INTAKE")) {
                if (!Objects.equals(intake.getDetectedColor(), "NA")) {
                    if (intake.isTarget()) {
                        // TARGET PIECE: Stop intake immediately and start goHome sequence.
                        intake.setSpeed(0, 0);
                        goingHome = true;
                        goHomeWaitStart = getRuntime();
                    } else {
                        // NON-TARGET: Reject it.
                        // Force both blockers open so the piece passes through.
                        intake.setInnerBlockOpen(true);
                        intake.setOuterBlockOpen(true);
                        // Use full-speed rejection (in the same direction as the manual command).
                        double rejectionSpeed = (currentGamepad2.right_trigger > currentGamepad2.left_trigger) ? 1.0 : -1.0;
                        intake.setSpeed(rejectionSpeed, rejectionSpeed);
                        rejecting = true;
                        rejectionEndTime = getRuntime();
                    }
                } else {
                    // No piece detected.
                    if (rejecting) {
                        // Wait 0.2 sec after the piece disappears before restoring manual control.
                        if (getRuntime() - rejectionEndTime >= 0.2) {
                            if (currentGamepad2.right_trigger > 0.1) {
                                intake.setInnerBlockOpen(false);
                                intake.setOuterBlockOpen(true);
                            } else if (currentGamepad2.left_trigger > 0.1) {
                                intake.setInnerBlockOpen(true);
                                intake.setOuterBlockOpen(false);
                            }
                            rejecting = false;
                            intake.setSpeed(0, 0);
                        } else {
                            double rejectionSpeed = (currentGamepad2.right_trigger > currentGamepad2.left_trigger) ? 1.0 : -1.0;
                            intake.setSpeed(rejectionSpeed, rejectionSpeed);
                        }
                    } else {
                        // NORMAL MANUAL INTAKE CONTROL.
                        if (currentGamepad2.right_trigger > 0.1) {
                            intake.setInnerBlockOpen(false);
                            intake.setOuterBlockOpen(true);
                            intake.setSpeed(currentGamepad2.right_trigger, currentGamepad2.right_trigger);
                        } else if (currentGamepad2.left_trigger > 0.1) {
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(false);
                            intake.setSpeed(-currentGamepad2.left_trigger, -currentGamepad2.left_trigger);
                        } else {
                            intake.setSpeed(0, 0);
                        }
                    }
                }
            }



            // ------------------- GO HOME (TARGET DELIVERY) SEQUENCE -------------------
            if (goingHome) {
                // Stop the intake.
                intake.setSpeed(0, 0);
                outtake.specDropAtIntakePos(true);
                outtake.specDropOpen(false);

                double slidePos = horizontalSlides.getCurrentPosition();

                // If the slide is extended beyond 400, command retraction to 400.
                if (slidePos > 200) {
                    horiSlidesTarget = 0.1;
                }

                if (slidePos<400){
                    intake.setRotation(TRANSFER);
                    intake.setInnerBlockOpen(true);
                    intake.setOuterBlockOpen(true);
                }
                if (slidePos < 200 && getRuntime() - goHomeWaitStart >= 0.4){
                    horiSlidesTarget = 0.1;
                }
                // When the slide is nearly retracted (below 25), run the intake wheels for 0.8 sec
                // to drop the piece. Do not change the intake rotation; leave it in TRANSFER.
                if (slidePos < 25) {
                    intake.setTimedIntake(-1, -1, 0.8);
                    goingHome = false;
                }
            }

            if (!goingHome) {
                double currentHoriPos = horizontalSlides.getCurrentPosition();
                double horiInput = -currentGamepad2.right_stick_y;

                if (Math.abs(horiInput) > JOYSTICK_DEADZONE) {
                    if (Math.abs(horiInput) < 0.6){
                        horiInput = horiInput/2;
                    }
                    if (intake.getRotationMode().equalsIgnoreCase("INTAKE")) {
                        horiSlidesTarget = currentHoriPos + (horiInput * MAX_INPUT_SCALING);
                        horiSlidesTarget = Math.max(horiSlidesTarget, 120);
                    } else {
                        horiSlidesTarget = currentHoriPos + (horiInput * MAX_INPUT_SCALING);
                    }

                } else {
                    horiSlidesTarget = currentHoriPos;
                }

            }

            horizontalSlides.setPosition(horiSlidesTarget);


            if (specRunTimeRunning && specBucketRunTimer +0.4> getRuntime()){
                outtake.specDropAtIntakePos(true);
                outtake.specDropOpen(false);
                specRunTimeRunning = false;
            }

            if ((currentGamepad2.a && !previousGamepad2.a)) {
                if (outtake.BucketPositionAtIntake){
                    outtake.specDropAtIntakePos(false);
                } else{
                    outtake.specDropAtIntakePos(true);
                    outtake.specDropOpen(false);
                    specRunTimeRunning = false;
                }
            }

            if (currentGamepad2.b && !previousGamepad2.b) {
                if (!outtake.BucketPositionAtIntake){
                    outtake.specDropOpen(true);
                    specBucketRunTimer = getRuntime();
                    specRunTimeRunning = true;
                }
            }



            // ------------------- MANUAL VERTICAL SLIDE CONTROL -------------------
            double currentVertPos = verticalSlides.getCurrentPosition();
            double stickVal = currentGamepad2.left_stick_y;
            if (Math.abs(stickVal) > JOYSTICK_DEADZONE) {
                vertSlidesTarget = currentVertPos + (-stickVal * MAX_INPUT_SCALING);
            }
            verticalSlides.setPosition(vertSlidesTarget);

            // ------------------- SUBSYSTEM UPDATES -------------------
            horizontalSlides.update();
            verticalSlides.update();
            intake.update();
            outtake.update();

            // ------------------- TELEMETRY -------------------
            telemetry.addData("Slide Pos", horizontalSlides.getCurrentPosition());
            telemetry.addData("Slide Target", horiSlidesTarget);
            telemetry.addData("Intake Rotation", intake.getRotationMode());
            telemetry.addData("Rejecting", rejecting);
            telemetry.addData("GoHome", goingHome);
            telemetry.addData("Loop Time", loopTime);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
}