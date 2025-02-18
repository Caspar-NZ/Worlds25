package org.firstinspires.ftc.teamcode.functions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class vertSlide {

    // Encoder limits: MIN_POSITION updates dynamically based on the mag sensor "vertMag"
    public static double MIN_POSITION = 0;
    public static double MAX_POSITION = MIN_POSITION + 1170;
    private static final double DEADZONE = 2;         // Encoder counts deadzone
    private static final double MAX_POWER = 1.0;        // Maximum motor power
    private static final double MAX_CURRENT = 3.0;      // Current limit in amps

    private DcMotorEx vert0;
    private DcMotorEx vert1;
    private double targetPosition = 0;
    double powerReq;
    TouchSensor slideMag;  // Use the sensor name "vertMag" per your instructions

    private BasicPID pidController;

    public vertSlide(HardwareMap hardwareMap) {
        vert0 = hardwareMap.get(DcMotorEx.class, "vert0");
        vert1 = hardwareMap.get(DcMotorEx.class, "vert1");
        slideMag = hardwareMap.get(TouchSensor.class, "vertMag");

        // Reverse one motor so that both move the slide in unison.
        vert1.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset encoders and use RUN_WITHOUT_ENCODER mode.
        vert0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vert1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        vert0.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        vert1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // PID tuning (for example, kP = 0.022, kI = 0.0, kD = 0.015)
        double kP = 0.022;
        double kI = 0.0;
        double kD = 0.015;
        PIDCoefficients pidCoefficients = new PIDCoefficients(kP, kI, kD);
        pidController = new BasicPID(pidCoefficients);
    }

    // Set target position; only clamp the upper limit (target can retract freely)
    public void setPosition(double position) {
        targetPosition = Math.min(position, MAX_POSITION);
    }

    public void update() {
        double currentPosition = vert0.getCurrentPosition();

        // Update MIN_POSITION:
        // If the sensor "vertMag" is pressed, update MIN_POSITION to the current reading.
        // Otherwise, if current reading is lower than MIN_POSITION, update it.
        if (slideMag.isPressed()) {
            MIN_POSITION = currentPosition;
        }
        if ((targetPosition < currentPosition) && (targetPosition < MIN_POSITION) && (!slideMag.isPressed())) { //this means we're requesting to go down
            MIN_POSITION -= 10;
            targetPosition = MIN_POSITION + 1;
        }

        MAX_POSITION = MIN_POSITION + 1170;

        if (targetPosition > MAX_POSITION){
            targetPosition = MAX_POSITION;
        }

        if (targetPosition < MIN_POSITION){
            targetPosition = MIN_POSITION;
        }

        if ((Math.abs(currentPosition - targetPosition) < DEADZONE) || targetPosition <= MIN_POSITION) {
            powerReq = 0;
            vert0.setPower(0);
            vert1.setPower(0);
            return;
        }





        powerReq = pidController.calculate(currentPosition, targetPosition);
        powerReq = Math.max(-MAX_POWER, Math.min(powerReq, MAX_POWER));
        powerReq = limitCurrent(vert0, powerReq);
        powerReq = limitCurrent(vert1, powerReq);

        // Invert output so that a positive manual command raises the slide.
        vert0.setPower(-powerReq);
        vert1.setPower(-powerReq);
    }

    public double getCurrentPosition() {
        return vert0.getCurrentPosition();
    }

    private double limitCurrent(DcMotorEx motor, double power) {
        double current = motor.getCurrent(CurrentUnit.AMPS);
        if (current > MAX_CURRENT) {
            double scaleFactor = MAX_CURRENT / current;
            power *= scaleFactor;
        }
        return power;
    }

    public double currentPower() {
        return powerReq;
    }
}
