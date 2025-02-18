package org.firstinspires.ftc.teamcode.functions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class horiSlides {

    // The slide range is 865 counts.
    public static int MIN_POSITION = 0;
    public static int MAX_POSITION = MIN_POSITION + 865;
    private static final double DEADZONE = 5;
    private static final double MAX_POWER = 1;

    private DcMotor leftHori;
    private DcMotor rightHori;
    private double targetPosition = 0;

    // Touch sensors for horizontal slide limits.
    private TouchSensor leftMag;
    private TouchSensor rightMag;

    private BasicPID pidController;
    double Power;

    public horiSlides(HardwareMap hardwareMap) {
        leftHori = hardwareMap.get(DcMotor.class, "leftHori");
        rightHori = hardwareMap.get(DcMotor.class, "rightHori");

        // Reverse leftHori so that the encoder increases as the slide extends.
        leftHori.setDirection(DcMotorSimple.Direction.REVERSE);
        // rightHori remains normal.

        leftMag = hardwareMap.get(TouchSensor.class, "leftMag");
        rightMag = hardwareMap.get(TouchSensor.class, "rightMag");

        leftHori.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightHori.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftHori.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightHori.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double kP = 0.05;
        double kI = 0.0;
        double kD = 0.0;
        PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
        pidController = new BasicPID(pidCoefficients);
    }

    public void setPosition(double position) {
        targetPosition = position;
    }

    public void update() {
        int currentPosLeft = leftHori.getCurrentPosition();
        int currentPosRight = rightHori.getCurrentPosition();
        int averagePosition = (currentPosLeft + currentPosRight) / 2;

        if (leftMag.isPressed() || rightMag.isPressed()) {
            MIN_POSITION = averagePosition;
        }

        if ((targetPosition < averagePosition) && (targetPosition < MIN_POSITION) && (!leftMag.isPressed() && !rightMag.isPressed())) { //this means we're requesting to go down
            MIN_POSITION -= 10;
            targetPosition = MIN_POSITION + 1;
        }

        MAX_POSITION = MIN_POSITION + 800;


        if (targetPosition > MAX_POSITION){
            targetPosition = MAX_POSITION;
        }

        if (targetPosition < MIN_POSITION){
            targetPosition = MIN_POSITION;
        }


        if ((Math.abs(averagePosition - targetPosition) < DEADZONE) || targetPosition <= MIN_POSITION) {
            Power = 0;
            leftHori.setPower(0);
            rightHori.setPower(0);
            return;
        }



        // Calculate PID output.
        double powerLeft = pidController.calculate(currentPosLeft, targetPosition);
        double powerRight = pidController.calculate(currentPosRight, targetPosition);

        // Clamp power to maximum limits.
        powerLeft = Math.max(-MAX_POWER, Math.min(powerLeft, MAX_POWER));
        powerRight = Math.max(-MAX_POWER, Math.min(powerRight, MAX_POWER));

        Power = powerLeft;

        leftHori.setPower(-powerLeft);
        rightHori.setPower(-powerRight);
    }
    public double currentPower() {
     return Power;
    }
    public boolean magResult(){
        if (leftMag.isPressed() || rightMag.isPressed()){
            return true;
        }
        else {
            return false;
        }
    }
    public double getCurrentPosition() {
        return (leftHori.getCurrentPosition() + rightHori.getCurrentPosition()) / 2.0;
    }
}
