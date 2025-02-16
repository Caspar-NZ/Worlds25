package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ZZColourTest", group = "Calibration")
public class ZZColourTest extends LinearOpMode {

    private CRServo leftIntake, rightIntake;
    private Servo innerBlocker, leftRotate, rightRotate;
    private DigitalChannel pin0, pin1;

    // Blocker servo positions
    private static final double BLOCK_POS = 0.53;
    private static final double OPEN_POS = 0.11;

    // Rotation servo positions
    private static final double LEFT_ROTATE_POS = 0.28;
    private static final double RIGHT_ROTATE_POS = 0.73;

    private double presenceEndTime = 0.0; // When we last saw a piece exit
    private boolean wasPresent = false;   // Tracks if we had presence in the previous loop
    private boolean rejecting = false;    // True if we’re currently rejecting a non-yellow piece

    @Override
    public void runOpMode() {
        // --- Hardware initialization ---
        leftIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");
        innerBlocker = hardwareMap.get(Servo.class, "innerBlock");
        leftRotate = hardwareMap.get(Servo.class, "leftRotate");
        rightRotate = hardwareMap.get(Servo.class, "rightRotate");

        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");

        // Ensure digital channels are inputs
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);

        // Set initial servo positions
        innerBlocker.setPosition(BLOCK_POS); // start blocked
        leftRotate.setPosition(LEFT_ROTATE_POS);
        rightRotate.setPosition(RIGHT_ROTATE_POS);

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1) Determine how hard the driver is pulling the trigger
            double triggerValue = gamepad1.right_trigger;

            // 2) Detect color presence via pin states
            boolean pin0State = pin0.getState();
            boolean pin1State = pin1.getState();
            boolean presence = pin0State || pin1State; // If either pin is true => presence

            // Identify which color is seen (if any)
            //   pin0=true & pin1=true   => Yellow
            //   pin0=true & pin1=false  => Blue
            //   pin0=false & pin1=true  => Red
            //   pin0=false & pin1=false => None/Distance>30mm
            String colorDetected = "None";
            if (presence) {
                if (pin0State && pin1State) {
                    colorDetected = "Yellow";
                } else if (pin0State && !pin1State) {
                    colorDetected = "Blue";
                } else if (!pin0State && pin1State) {
                    colorDetected = "Red";
                }
            }

            // 3) Intake logic
            double leftPower;
            double rightPower;

            if (presence) {
                // If a piece is present
                if (colorDetected.equals("Yellow")) {
                    // If it's yellow, stop the intake
                    leftPower = 0;
                    rightPower = 0;
                    innerBlocker.setPosition(BLOCK_POS);
                    rejecting = false; // definitely not rejecting
                } else {
                    // If it's NOT yellow => reject it at full power
                    // and open the blocker to let it pass.
                    // Reversed: left intake gets positive power for intake, right intake negative.
                    leftPower = 1;
                    rightPower = -1;
                    innerBlocker.setPosition(OPEN_POS);
                    rejecting = true;
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
                        innerBlocker.setPosition(BLOCK_POS);
                        rejecting = false;
                    } else {
                        // Keep the blocker open while waiting
                        innerBlocker.setPosition(OPEN_POS);
                    }
                }

                // If not rejecting, respond to the trigger input
                if (!rejecting) {
                    leftPower = triggerValue;
                    rightPower = -triggerValue;
                } else {
                    // If we are still in the "waiting to close" window, keep rejecting at full power
                    leftPower = 1;
                    rightPower = -1;
                }
            }

            // Set the CR servos
            leftIntake.setPower(leftPower);
            rightIntake.setPower(rightPower);

            // 4) Telemetry
            telemetry.addData("Trigger Value", triggerValue);
            telemetry.addData("Pin0", pin0State);
            telemetry.addData("Pin1", pin1State);
            telemetry.addData("Color Detected", colorDetected);
            telemetry.addData("Blocker Pos", innerBlocker.getPosition());
            telemetry.addData("Rejecting?", rejecting);
            telemetry.update();

            idle();
        }
    }
}
