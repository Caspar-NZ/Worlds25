package org.firstinspires.ftc.teamcode.functions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {
    private Servo hookRotate, claw, bucket;

    final private double rotateAtIntake = 0.83;
    final private double rotateAtDelivery = 0.04;
    final private double clawOpen = 0.0;
    final private double clawClosed = 0.27;
    final private double bucketAtIntake = 0.0;
    final private double bucketAtDelivery = 0.3;
    private final int bucketMoveSteps = 6; // Number of steps to complete the movement
    private int bucketMoveCounter = 0; // Counter for tracking bucket movement steps
    private double bucketIncrement; // Incremental change per step

    public boolean isClawOpen;
    public boolean BucketPositionAtIntake;

    private double setHookPos;
    private double setClawPos;
    private double setBucketPos;

    public outtake(HardwareMap hardwareMap){
        hookRotate = hardwareMap.servo.get("hookRotate");
        claw = hardwareMap.servo.get("claw");
        bucket = hardwareMap.servo.get("bucket");
    }

    public void update(){
        hookRotate.setPosition(setHookPos);
        claw.setPosition(setClawPos);

        // Gradual bucket movement if counter is active
        if (bucketMoveCounter > 0 && bucketMoveCounter <= bucketMoveSteps) {
            setBucketPos += bucketIncrement;
            bucketMoveCounter++;
        }

        bucket.setPosition(setBucketPos);
    }

    public void hookAtIntake(Boolean intake69){
        if (intake69){
            setHookPos = rotateAtIntake;
        } else{
            setHookPos = rotateAtDelivery;
        }
    }

    public void clawOpen(boolean open){
        if (open){
            isClawOpen = true;
            setClawPos = clawOpen;
        } else{
            isClawOpen = false;
            setClawPos = clawClosed;
        }
    }

    public void bucketAtIntakePos(Boolean intake){
        if (intake){
            BucketPositionAtIntake = true;
            setBucketPos = bucketAtIntake;
            bucketMoveCounter = 0; // Reset movement counter if moving to intake
        } else {
            BucketPositionAtIntake = false;
            bucketIncrement = (bucketAtDelivery - setBucketPos) / bucketMoveSteps;
            bucketMoveCounter = 1; // Start moving gradually to delivery position
        }
    }
}
