package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.CRServo;  // Continuous rotation servo
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class intake {

    // --- Hardware Devices ---
    private final CRServo leftIntake, rightIntake;
    private final Servo innerBlock, outerBlock, leftRotate, rightRotate;
    public DigitalChannel pin0, pin1, epin0, epin1;  // For color detection

    // --- Constants: Inner & Outer Block positions ---
    // For innerBlock: 0.53 = closed (block), 0.11 = open.
    private static final double INNER_BLOCK_CLOSED = 0.53;
    private static final double INNER_BLOCK_OPEN   = 0.11;

    // For outerBlock: 0.55 = closed (block), 0.15 = open.
    private static final double OUTER_BLOCK_CLOSED = 0.55;
    private static final double OUTER_BLOCK_OPEN   = 0.15;

    // --- Constants: Rotation servo positions ---
    // Presets for leftRotate and rightRotate (linked inversely)
    private static final double LEFT_DOWN_TRANSFER  = 0.86; //0.84
    private static final double RIGHT_DOWN_TRANSFER = 0.14; //0.16

    private static final double LEFT_DOWN_INTAKE    = 0.30; //0.28
    private static final double RIGHT_DOWN_INTAKE   = 0.71; //0.73
    private static final double LEFT_AUTOPOS    = 0.22; //0.28
    private static final double RIGHT_AUTOPOS   = 0.79; //0.73

    private static final double LEFT_DOWN_TUCKED    = 0.03;
    private static final double RIGHT_DOWN_TUCKED   = 0.98;

    // --- Internal target variables ---
    private double leftIntakePower = 0;
    private double rightIntakePower = 0;

    private double innerBlockTarget = INNER_BLOCK_CLOSED;
    private double outerBlockTarget = OUTER_BLOCK_CLOSED;

    private double leftRotateTarget = LEFT_DOWN_INTAKE;  // Default to INTAKE mode.
    private double rightRotateTarget = RIGHT_DOWN_INTAKE;

    // For rotation, store current mode so getters can return a friendly string.
    public enum RotationMode { TRANSFER, INTAKE, TUCKED }
    private RotationMode currentRotation = RotationMode.INTAKE;

    // --- Timed Intake Command State ---
    private boolean timedIntakeActive = false;
    private double timedLeftSpeed = 0;
    private double timedRightSpeed = 0;
    private long timedEndTimeMs = 0;
    public TargetState target1 = TargetState.NONE;
    public TargetState target2 = TargetState.NONE;


    // --- Constructor ---
    public intake(HardwareMap hardwareMap) {
        // Initialize CR servos for intake wheels.
        leftIntake  = hardwareMap.get(CRServo.class, "leftIntake");
        rightIntake = hardwareMap.get(CRServo.class, "rightIntake");

        // Initialize servos for blockers and rotation.
        innerBlock = hardwareMap.get(Servo.class, "innerBlock");
        outerBlock = hardwareMap.get(Servo.class, "outerBlock");
        leftRotate   = hardwareMap.get(Servo.class, "leftRotate");
        rightRotate  = hardwareMap.get(Servo.class, "rightRotate");

        // Initialize digital channels for color detection.
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
        epin0 = hardwareMap.digitalChannel.get("edigital0");
        epin1 = hardwareMap.digitalChannel.get("edigital1");
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);
        epin0.setMode(DigitalChannel.Mode.INPUT);
        epin1.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * update() sends the current target values to the hardware.
     * This should be called as frequently as possible in your teleop loop.
     */
    public void update() {
        // If a timed command is active, override the intake speeds accordingly.
        if (timedIntakeActive) {
            if (System.currentTimeMillis() <= timedEndTimeMs) {
                leftIntakePower = timedLeftSpeed;
                rightIntakePower = timedRightSpeed;
            } else {
                // Timed command expired; stop the intake and clear the flag.
                leftIntakePower = 0;
                rightIntakePower = 0;
                timedIntakeActive = false;
            }
        }

        // Set intake wheel speeds.
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(-rightIntakePower);

        // Set blocker positions.
        innerBlock.setPosition(innerBlockTarget);
        outerBlock.setPosition(outerBlockTarget);

        // Set rotation servo positions.
        leftRotate.setPosition(leftRotateTarget);
        rightRotate.setPosition(rightRotateTarget);
    }

    // --- Getter Methods ---

    /**
     * Returns the currently detected color.
     * Logic: (pin0 && pin1) = "Yellow", (pin0 && !pin1) = "Blue", (!pin0 && pin1) = "Red", else "NA".
     */
    public TargetState getDetectedColor() {
        boolean state0 = pin0.getState();
        boolean state1 = pin1.getState();
        boolean estate0 = epin0.getState();
        boolean estate1 = epin1.getState();

        TargetState sensor1;
        TargetState sensor2;

        if (state0 && state1) {
            sensor1 = TargetState.YELLOW;
        } else if (state0) {
            sensor1 = TargetState.BLUE;
        } else if (state1) {
            sensor1 = TargetState.RED;
        } else {
            sensor1 = TargetState.NONE;
        }

        if (estate0 && estate1) {
            sensor2 = TargetState.YELLOW;
        } else if (estate0) {
            sensor2 = TargetState.BLUE;
        } else if (estate1) {
            sensor2 = TargetState.RED;
        } else {
            sensor2 = TargetState.NONE;
        }

        // If both sensors agree, return that value.
        if (sensor1 == sensor2) {
            return sensor1;
        } else {
            return TargetState.MIXED;
        }
    }
    /**
     * Returns the current rotation mode as a String.
     * Will be one of "TRANSFER", "INTAKE", or "TUCKED".
     */
    public String getRotationMode() {
        return currentRotation.toString();
    }

    // --- Setter Methods ---

    /**
     * Set intake speed.
     * Use positive values for standard intake (e.g. 1,1) and negative for reverse (-1,-1).
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
        // This will be overridden if a timed command is active in update()
        leftIntakePower  = leftSpeed;
        rightIntakePower = rightSpeed;
    }

    /**
     * Set the inner block state.
     * @param open true to open the inner block; false to close it.
     */
    public void setInnerBlockOpen(boolean open) {
        innerBlockTarget = open ? INNER_BLOCK_OPEN : INNER_BLOCK_CLOSED;
    }

    /**
     * Set the outer block state.
     * @param open true to open the outer block; false to close it.
     */
    public void setOuterBlockOpen(boolean open) {
        outerBlockTarget = open ? OUTER_BLOCK_OPEN : OUTER_BLOCK_CLOSED;
    }

    /**
     * Set the rotation mode for the intake.
     * This updates the positions for the leftRotate and rightRotate servos as well as the current mode.
     */
    public void setRotation(RotationMode mode) {
        currentRotation = mode;
        switch(mode) {
            case TRANSFER:
                leftRotateTarget  = LEFT_DOWN_TRANSFER;
                rightRotateTarget = RIGHT_DOWN_TRANSFER;
                break;
            case INTAKE:
                leftRotateTarget  = LEFT_DOWN_INTAKE;
                rightRotateTarget = RIGHT_DOWN_INTAKE;
                break;
            case TUCKED:
                leftRotateTarget  = LEFT_DOWN_TUCKED;
                rightRotateTarget = RIGHT_DOWN_TUCKED;
                break;
        }
    }

    public void setAutoPos() {
        leftRotateTarget  = LEFT_AUTOPOS;
        rightRotateTarget = RIGHT_AUTOPOS;
    }


    /**
     * Set the intake speeds for a specific duration.
     * The speeds will remain active until durationSec has elapsed,
     * after which they will be automatically set back to 0 in update().
     *
     * @param leftSpeed Speed for the left intake wheel.
     * @param rightSpeed Speed for the right intake wheel.
     * @param durationSec Duration in seconds to run the intake.
     */
    public void setTimedIntake(double leftSpeed, double rightSpeed, double durationSec) {
        timedLeftSpeed = leftSpeed;
        timedRightSpeed = rightSpeed;
        timedEndTimeMs = System.currentTimeMillis() + (long)(durationSec * 1000);
        timedIntakeActive = true;
    }

    public void setTarget(int yellow, int red, int blue){
        if (yellow + red + blue > 1) {
            target1 = TargetState.YELLOW;
            target2 = (red > 0) ? TargetState.RED : TargetState.BLUE;
        } else {
            if (yellow > 0) {
                target1 = TargetState.YELLOW;
                target2 = TargetState.NONE;
            } else if (red > 0) {
                target1 = TargetState.NONE;
                target2 = TargetState.RED;
            } else {
                target1 = TargetState.NONE;
                target2 = TargetState.BLUE;
            }
        }
    }


    public boolean isTarget(){
        TargetState detected = getDetectedColor();
        return (detected == target1 || detected == target2);
    }

}