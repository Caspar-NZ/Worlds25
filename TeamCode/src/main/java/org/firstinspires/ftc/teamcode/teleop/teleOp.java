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
            // For horizontal slides, we want:
            // - Pushing the right stick UP (which gives a negative value) to extend the slide (increase target)
            // - Pulling the stick DOWN (positive value) to retract (decrease target)
            double currentHoriPos = horizontalSlides.getCurrentPosition();

            double horiInput = -currentGamepad2.right_stick_y;
            if (Math.abs(horiInput) > JOYSTICK_DEADZONE) {
                horiSlidesTarget = currentHoriPos + (horiInput * MAX_INPUT_SCALING);
            } else {
                // No input: hold the current position.
                horiSlidesTarget = currentHoriPos;
            }
            //horiSlidesTarget = Math.max(horizontalSlides.MIN_POSITION, Math.min(horiSlidesTarget, horizontalSlides.MAX_POSITION));
            horizontalSlides.setPosition(horiSlidesTarget);

            // ------------------- Manual Vertical Slide Control -------------------
            // Retrieve current vertical slide position
            double currentVertPos = verticalSlides.getCurrentPosition();
// Read the vertical stick value from Gamepad2
            double stickVal = currentGamepad2.left_stick_y;

// If the stick is outside the deadzone, update the target position.
            if (Math.abs(stickVal) > JOYSTICK_DEADZONE) {
                // Negative stick value (pushing up) should raise the slide.
                vertSlidesTarget = currentVertPos + (-stickVal * MAX_INPUT_SCALING);
                // Optionally, you could log that a new target was set.
            }
// Otherwise, if the stick is within the deadzone, do not update the target.
// This "latches" the previous target so the slide holds its position.
            verticalSlides.setPosition(vertSlidesTarget);


            // ------------------- Subsystem Updates -------------------
            horizontalSlides.update();
            verticalSlides.update();
            // intake.update();   // Uncomment if intake control is desired
            // outtake.update();  // Uncomment if outtake control is desired

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
