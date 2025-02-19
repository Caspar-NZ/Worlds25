package org.firstinspires.ftc.teamcode.functions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {
    private Servo clawRotate, claw, specDrop, specRelease;
    final private double rotateAtIntake = 1.0;
    final private double rotateAtDelivery = 0.32;
    final private double clawOpen = 0.0;
    final private double clawClosed = 0.23;
    final private double specDropAtIntake = 1.0;
    final private double specDropAtDelivery = 0.76;
    final private double specReleaseBlocking = 0.05;
    final private double specReleaseOpen = 0.28;

    public boolean isClawOpen;
    public boolean BucketPositionAtIntake;
    public boolean clawAtIntake;
    public boolean specDropOpen;
    private double setHookPos;
    private double setClawPos;
    private double setBucketPos;
    private double setReleasePos;

    public outtake(HardwareMap hardwareMap){
        clawRotate = hardwareMap.servo.get("clawRotate");
        claw = hardwareMap.servo.get("claw");
        specDrop = hardwareMap.servo.get("specDrop");
        specRelease = hardwareMap.servo.get("specRelease");
    }
    public void update(){
        clawRotate.setPosition(setHookPos);
        claw.setPosition(setClawPos);
        specDrop.setPosition(setBucketPos);
        specRelease.setPosition(setReleasePos);
    }
    public void hookAtIntake(Boolean intake){
        if (intake){
            setHookPos = rotateAtIntake;
            clawAtIntake = true;
        } else{
            setHookPos = rotateAtDelivery;
            clawAtIntake = false;
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
    public void specDropAtIntakePos(Boolean intake){
        if (intake){
            BucketPositionAtIntake = true;
            setBucketPos = specDropAtIntake;
        } else {
            BucketPositionAtIntake = false;
            setBucketPos = specDropAtDelivery;
        }
    }
    public void specDropOpen(Boolean intake){
        if (intake){
            specDropOpen = true;
            setReleasePos = specReleaseOpen;
        } else {
            specDropOpen = false;
            setReleasePos = specReleaseBlocking;
        }
    }
}
