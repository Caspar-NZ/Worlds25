package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;
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
    private static final double JOYSTICK_DEADZONE = 0.02;

    // Slide target positions
    private double horiSlidesTarget = 0;
    private double vertSlidesTarget = 0;

    // Gamepad state copies
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // Alliance and targeting variables
    public String Alliance = "Red";
    public String rejectAlliance = "Blue";
    public String otherReject = "Yellow";
    public boolean sampleInBasket;
    public boolean blocking = true;
    double previousStartTime = 0;
    double loopTime = 0;
    public String target = "";
    int i = 0;

    private double presenceEndTime = 0.0; // When we last saw a piece exit
    private boolean wasPresent = false;   // Tracks if we had presence in the previous loop
    private boolean rejecting = false;    // True if we’re currently rejecting a non-yellow piece
    private double rejectSpeed = 0.0;
    private boolean firstLoop = false;
    private boolean trigger1 = false;
    private String lastClosed = "";
    private boolean goingHome = false;
    private double rotationStartTime = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set bulk caching mode for all hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ------------------- Drive Motor Initialization -------------------
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // ------------------- Subsystem Initialization -------------------
        horiSlides horizontalSlides = new horiSlides(hardwareMap);
        vertSlide verticalSlides = new vertSlide(hardwareMap);
        outtake outtake = new outtake(hardwareMap);
        intake intake = new intake(hardwareMap);

        // Initialize intake and outtake positions
        intake.setRotation(TUCKED);
        intake.setOuterBlockOpen(true);
        intake.setInnerBlockOpen(true);
        intake.update();

        outtake.hookAtIntake(true);
        outtake.clawOpen(true);
        outtake.specDropAtIntakePos(true);
        outtake.specDropOpen(false);
        outtake.update();

        waitForStart();

        // ------------------- Main OpMode Loop -------------------
        while (opModeIsActive()) {
            loopTime = getRuntime() - previousStartTime;
            previousStartTime = getRuntime();

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Toggle "otherReject" via Gamepad Y for outtake targeting
            if ((currentGamepad1.y && !previousGamepad1.y) || (currentGamepad2.y && !previousGamepad2.y)) {
                otherReject = otherReject.equals("Yellow") ? Alliance : "Yellow";
            }

            // Determine target color for LED feedback and outtake
            if (!otherReject.equals("Yellow")) {
                target = "Yellow";
            } else if (Alliance.equals("Red")) {
                target = "Red";
            } else {
                target = "Blue";
            }

            // LED feedback: first 100 loops show target color; then alternate based on blocking flag
            if (i < 100) {
                if (target.equals("Red")) {
                    gamepad1.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
                } else if (target.equals("Yellow")) {
                    gamepad1.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
                } else if (target.equals("Blue")) {
                    gamepad1.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
                }
                i++;
            } else {
                if (!blocking) {
                    gamepad1.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
                } else {
                    gamepad1.setLedColor(0, 255, 0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(0, 255, 0, LED_DURATION_CONTINUOUS);
                }
                i++;
                if (i > 200) { i = 0; }
            }

            // ------------------- Drive Control -------------------
            double y = -currentGamepad1.left_stick_y;
            double x = currentGamepad1.left_stick_x * 1.1;
            double rx = currentGamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // ------------------- Manual Horizontal Slide Control -------------------
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper){ //rotate intake
                if (intake.getRotationMode().equalsIgnoreCase("TUCKED")){
                    intake.setRotation(INTAKE);
                    intake.setTimedIntake(-1,-1,0.4);
                }
                if (intake.getRotationMode().equalsIgnoreCase("INTAKE")){
                    if (!Objects.equals(intake.getDetectedColor(), "NA")){
                        intake.setRotation(TRANSFER);
                    } else {
                        intake.setRotation(TUCKED);
                    }
                }
                if (intake.getRotationMode().equalsIgnoreCase("TRANSFER")){
                    if (!Objects.equals(intake.getDetectedColor(), "NA")){
                        intake.setRotation(INTAKE);
                    } else {
                        intake.setRotation(TUCKED);
                    }
                }
            }

            if (intake.getRotationMode().equalsIgnoreCase("INTAKE") && !goingHome) {
                if (!Objects.equals(intake.getDetectedColor(), "NA")) {
                    if ("target game piece colour") {
                        goingHome = true;
                        firstLoop = true;
                        trigger1 = true;
                    } else if (intake.isInnerOpen()) {
                        rejectSpeed = -1.0;
                        rejecting = true;
                        lastClosed = "OUTER";
                    } else {
                        rejectSpeed = 1.0;
                        rejecting = true;
                        lastClosed = "INNER";
                    }
                    wasPresent = true;
                } else {
                    // No presence
                    // If we just transitioned from presence to no presence, record the time
                    if (wasPresent) {
                        presenceEndTime = getRuntime();
                        wasPresent = false;
                    }
                    // If we were rejecting, keep the blocker open until 0.2s after the piece left
                    if (rejecting) {
                        if ((getRuntime() - presenceEndTime) >= 0.2) {
                            // Enough time has passed => close blocker, stop rejecting
                            if (lastClosed.equals("OUTER")) {
                                intake.setOuterBlockOpen(false);
                            } else {
                                intake.setInnerBlockOpen(false);
                            }
                            rejecting = false;
                        } else {
                            // Keep the blocker open while waiting
                            intake.setOuterBlockOpen(true);
                            intake.setInnerBlockOpen(true);
                        }
                    }

                    // If not rejecting, respond to the trigger input
                    if (!rejecting) {
                        if (currentGamepad2.right_trigger > 0.1) {
                            intake.setInnerBlockOpen(false);
                            intake.setOuterBlockOpen(true);
                            intake.setSpeed(currentGamepad2.right_trigger, currentGamepad2.right_trigger);
                        } else if (currentGamepad2.left_trigger > 0.1) {
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(false);
                            intake.setSpeed(-currentGamepad2.left_trigger, -currentGamepad2.left_trigger);
                        }
                    } else {
                        // If we are still in the "waiting to close" window, keep rejecting at full power
                        intake.setSpeed(rejectSpeed, rejectSpeed);
                    }
                }
            }

            if (goingHome){
                wasPresent = false;
                outtake.specDropAtIntakePos(true);
                if (horizontalSlides.getCurrentPosition()>200 && firstLoop){
                    horiSlidesTarget = 200;
                    firstLoop = false;
                }
                //horiSlidesTarget = 0;
                if (horizontalSlides.getCurrentPosition()<500 && trigger1){
                    intake.setOuterBlockOpen(true);
                    intake.setInnerBlockOpen(true);
                    intake.setRotation(TRANSFER);
                    rotationStartTime = getRuntime();
                    trigger1 = false;
                }
                if (intake.getRotationMode().equalsIgnoreCase("TRANSFER") && horiSlidesTarget != 0){
                    if ((horizontalSlides.getCurrentPosition()>200) || (getRuntime() > rotationStartTime +400)){
                        horiSlidesTarget = 0;
                    }
                }
                if (horizontalSlides.getCurrentPosition()<15){
                    intake.setTimedIntake(-1,-1,0.8);
                    goingHome = false;
                }

            }

            double currentHoriPos = horizontalSlides.getCurrentPosition();

            double horiInput = -currentGamepad2.right_stick_y;
            if (Math.abs(horiInput) > JOYSTICK_DEADZONE) {
                horiSlidesTarget = currentHoriPos + (horiInput * MAX_INPUT_SCALING);
            } else {
                horiSlidesTarget = currentHoriPos;
            }


            horizontalSlides.setPosition(horiSlidesTarget);






            // ------------------- Manual Vertical Slide Control -------------------
            double currentVertPos = verticalSlides.getCurrentPosition();
            double stickVal = currentGamepad2.left_stick_y;
            if (Math.abs(stickVal) > JOYSTICK_DEADZONE) {
                vertSlidesTarget = currentVertPos + (-stickVal * MAX_INPUT_SCALING);
            }


            verticalSlides.setPosition(vertSlidesTarget);


            // ------------------- Subsystem Updates -------------------
            horizontalSlides.update();
            verticalSlides.update();
            intake.update();
            outtake.update();

            // ------------------- Telemetry -------------------
            telemetry.addData("Vert power", verticalSlides.currentPower());
            telemetry.addData("Horiz power", horizontalSlides.currentPower());
            telemetry.addData("Sample in bucket", sampleInBasket);
            telemetry.addData("Vert current", verticalSlides.getCurrentPosition());
            telemetry.addData("Vert min pos", vertSlide.MIN_POSITION);
            telemetry.addData("Vert target", vertSlidesTarget);
            telemetry.addData("Horiz current", horizontalSlides.getCurrentPosition());
            telemetry.addData("Horiz target", horiSlidesTarget);
            telemetry.addData("Horiz minpos", horizontalSlides.MIN_POSITION);
            telemetry.addData("Horiz mags", horizontalSlides.magResult());
            telemetry.addData("Target colour", target);
            telemetry.addData("Alliance", Alliance);
            telemetry.addData("Reject colour", otherReject);
            telemetry.addData("Loop time", loopTime);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
}
