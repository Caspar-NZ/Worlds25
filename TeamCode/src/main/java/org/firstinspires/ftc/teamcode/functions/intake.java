package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.CRServo;  // Continuous rotation servo
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Intake subsystem class.
 *
 * This class encapsulates control of:
 *   - Intake wheels (CRServos)
 *   - Inner and Outer blocker servos
 *   - Rotation servos (leftRotate and rightRotate), which share an inverse relationship.
 *
 * Teleop code can change target values via setter functions (e.g. setSpeed(), setInnerBlock(), setRotation(), etc.)
 * and then call update() each loop to send the latest values to hardware.
 */
public class intake {

    // --- Hardware Devices ---
    private CRServo leftIntake, rightIntake;
    private Servo innerBlock, outerBlock;
    private Servo leftRotate, rightRotate;
    private DigitalChannel pin0, pin1;  // For color detection

    // --- Constants: Inner & Outer Block positions ---
    // For innerBlock: 0.53 = closed (block), 0.11 = open.
    private static final double INNER_BLOCK_CLOSED = 0.53;
    private static final double INNER_BLOCK_OPEN   = 0.11;

    // For outerBlock: 0.55 = closed (block), 0.15 = open.
    private static final double OUTER_BLOCK_CLOSED = 0.55;
    private static final double OUTER_BLOCK_OPEN   = 0.15;

    // --- Constants: Rotation servo positions ---
    // Presets for leftRotate and rightRotate (linked inversely)
    private static final double LEFT_DOWN_TRANSFER  = 0.84;
    private static final double RIGHT_DOWN_TRANSFER = 0.16;

    private static final double LEFT_DOWN_INTAKE    = 0.28;
    private static final double RIGHT_DOWN_INTAKE   = 0.73;

    private static final double LEFT_DOWN_TUCKED    = 0.01;
    private static final double RIGHT_DOWN_TUCKED   = 1.0;

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
        pin0.setMode(DigitalChannel.Mode.INPUT);
        pin1.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * update() sends the current target values to the hardware.
     * This should be called as frequently as possible in your teleop loop.
     */
    public void update() {
        // Set intake wheel speeds.
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);

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
    public String getDetectedColor() {
        boolean state0 = pin0.getState();
        boolean state1 = pin1.getState();
        if (state0 && state1)      return "Yellow";
        else if (state0 && !state1) return "Blue";
        else if (!state0 && state1) return "Red";
        else                      return "NA";
    }

    /**
     * Returns the current inner block state as a String ("open" or "closed").
     */
    public String getInnerBlockState() {
        return (innerBlockTarget == INNER_BLOCK_OPEN) ? "open" : "closed";
    }

    /**
     * Returns the current outer block state as a String ("open" or "closed").
     */
    public String getOuterBlockState() {
        return (outerBlockTarget == OUTER_BLOCK_OPEN) ? "open" : "closed";
    }

    /**
     * Returns the current rotation mode as a String.
     * Will be one of "TRANSFER", "INTAKE", or "TUCKED".
     */
    public String getRotationMode() {
        return currentRotation.toString();
    }

    /**
     * Returns the current intake speeds as an array: [leftIntakePower, rightIntakePower].
     */
    public double[] getIntakeSpeeds() {
        return new double[] { leftIntakePower, rightIntakePower };
    }

    // --- Setter Methods ---

    /**
     * Set intake speed.
     * Use positive values for standard intake (e.g. 1,1) and negative for reverse (-1,-1).
     */
    public void setSpeed(double leftSpeed, double rightSpeed) {
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

    // --- Additional Helper Methods (optional) ---

    /**
     * Toggle the inner block state.
     */
    public void toggleInnerBlock() {
        if(innerBlockTarget == INNER_BLOCK_CLOSED) {
            innerBlockTarget = INNER_BLOCK_OPEN;
        } else {
            innerBlockTarget = INNER_BLOCK_CLOSED;
        }
    }

    /**
     * Toggle the outer block state.
     */
    public void toggleOuterBlock() {
        if(outerBlockTarget == OUTER_BLOCK_CLOSED) {
            outerBlockTarget = OUTER_BLOCK_OPEN;
        } else {
            outerBlockTarget = OUTER_BLOCK_CLOSED;
        }
    }
}
