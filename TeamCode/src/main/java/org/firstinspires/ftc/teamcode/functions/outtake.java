package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class outtake {
    private final Servo clawRotate, claw, specDrop, specRelease, sampleRotate, sampleRelease;
    final private double rotateAtIntake = 0.97; //0.94
    final private double rotateAtAutoStart =0.63;
    final private double rotateAtDelivery = 0.35; //0.32

    final private double clawOpen = 0.0;
    final private double clawClosed = 0.22;//0.22
    final private double specDropAtIntake = 1.0;
    final private double specDropAtAuto = 0.82; //was 0.88
    final private double specDropAtDelivery = 0.76;
    final private double specReleaseBlocking = 0.05;
    final private double specReleaseOpen = 0.32;
    final private double sampleOpen = 0.9;
    final private double sampleClosed = 0.65;
    final private double sampleAtIntake = 0.45;
    final private double sampleAtDelivery = 0.1;
    final private double sampleOutOfWay = 0;
    final private double sampleStartPos = 0.25;
    public boolean isClawOpen, isSampleOpen, samplePositionAtIntake, BucketPositionAtIntake,clawAtIntake,specDropOpen;
    private double setHookPos, setClawPos, setBucketPos, setReleasePos, setSamplePos, setSampleRotatePos;
    public boolean dumpingYellows = true;
    public boolean scoringSamples = false;
    public boolean scoringSpecs = true;
    public double incrementSpec = 1.0;
    public boolean slowlyRotating = false;

    public outtake(HardwareMap hardwareMap){
        clawRotate = hardwareMap.servo.get("clawRotate");
        claw = hardwareMap.servo.get("claw");
        specDrop = hardwareMap.servo.get("specDrop");
        specRelease = hardwareMap.servo.get("specRelease");
        sampleRotate = hardwareMap.servo.get("sampleRotate");
        sampleRelease = hardwareMap.servo.get("sampleRelease");
    }
    public void update(){
        clawRotate.setPosition(setHookPos);
        claw.setPosition(setClawPos);

        if (slowlyRotating){
            incrementSpec -= 0.01;
            setBucketPos = incrementSpec;
            if (incrementSpec <= 0.76){
                slowlyRotating = false;
            }
        }

        specDrop.setPosition(setBucketPos);
        specRelease.setPosition(setReleasePos);
        sampleRelease.setPosition(setSamplePos);
        sampleRotate.setPosition(setSampleRotatePos);
    }
    public void hookAtIntake(Boolean intake, Boolean AutoStartPos){
        if (AutoStartPos){
            setHookPos = rotateAtAutoStart;
            clawAtIntake = false;
        } else if (intake){
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
    public void sampleReleaseOpen(boolean open){
        if (open){
            isSampleOpen = true;
            setSamplePos = sampleOpen;
        } else{
            isSampleOpen = false;
            setSamplePos = sampleClosed;
        }
    }
    public void sampleAtIntakePos(Boolean intake){
        if (intake){
            samplePositionAtIntake = true;
            setSampleRotatePos = sampleAtIntake;
        } else {
            samplePositionAtIntake = false;
            setSampleRotatePos = sampleAtDelivery;
        }
    }
    public void setSampleOutOfWay(){
        setSampleRotatePos = sampleOutOfWay;
    }
    public void specDropAtIntakePos(Boolean intake){
        if (intake){
            BucketPositionAtIntake = true;
            setBucketPos = specDropAtIntake;
        } else {
            BucketPositionAtIntake = false;
            slowlyRotating = false;
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

    public void specSlowRotate(){
        incrementSpec = 1.0;
        slowlyRotating = true;
        BucketPositionAtIntake = false;
    }
    public void setSpecDropAtAuto(){
        setBucketPos = specDropAtAuto;
    }
    public void setSampleInside18(){
        setSampleRotatePos = sampleStartPos;
    }
}
