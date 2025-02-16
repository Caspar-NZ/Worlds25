package org.firstinspires.ftc.teamcode.teleop;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

import java.util.List;

@TeleOp
public class teleOp extends LinearOpMode {

    private static final double MAX_INPUT_SCALING = 200;
    private boolean horiSlideIsRunningToPos = false;
    private double horiSlidesTarget;

    private boolean vertSlideIsRunningToPos = false;

    private double vertSlidesTarget;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    public String Alliance = "Red";
    public String rejectAlliance = "Blue";
    public String otherReject = "Yellow";
    public boolean ejecting = false;
    public double blockDelay;
    public boolean runDelay = false;
    public boolean runningHomeHori = false;
    public double dropInBasketTimer;
    public boolean dropInBasketDelay = false;
    public boolean slightDelayToRotate = false;
    public double roationDelayTimer;
    public boolean intakeRotationUp = true;
    public boolean specimenInClaw;
    public boolean sampleInBasket;
    public boolean deliveringSpecimen = false;
    public double specimenDelivTimer;
    public boolean specimenRunTimer = false;
    public double sampleDelivTimer;
    public boolean specimenHooked = false;
    public boolean justDroppedSample = false;
    public boolean readyToGoToDelivery = false;
    public boolean readyToDeliver = false;
    public boolean blocking = true;
    double previousStartTime = 0;
    double loopTime = 0;

    public String previosIntakeResult;

    public String target ="";
    int i =0;
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();
        Gamepad.RumbleEffect effect2 = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 0.0, 100)
                .build();


        // Initialize drive motors
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize outtake
        /*outtake outtake = new outtake(hardwareMap);
        outtake.clawOpen(true);
        outtake.hookAtIntake(true);
        outtake.bucketAtIntakePos(true);
        outtake.update();

        // Initialize intake
        intake intake = new intake(hardwareMap);
        intake.intakeRotatedUp(true);
        intakeRotationUp = true;
        intake.open(false);
        blocking = true;
        intake.update();
*/
        // Initialize horizontal slides
        horiSlides horizontalSlides = new horiSlides(hardwareMap);

        vertSlide verticalSlides = new vertSlide(hardwareMap);




        while (!isStarted() && !isStopRequested()) {

            if (Math.abs(currentGamepad2.left_stick_y) > 0.02) {
                vertSlideIsRunningToPos = false;
            } else {
                vertSlideIsRunningToPos = true;
            }
            if (vertSlideIsRunningToPos){
                verticalSlides.setPosition(vertSlidesTarget);
            } else{
                /////vert manual control
                double currentPositionvert = verticalSlides.getCurrentPosition();
                vertSlidesTarget = currentPositionvert + -currentGamepad2.left_stick_y * MAX_INPUT_SCALING;
                //vertSlidesTarget = Math.max(vertSlide.MIN_POSITION, Math.min(vertSlidesTarget, vertSlide.MAX_POSITION));
                verticalSlides.setPosition(vertSlidesTarget);
            }

            if ((Math.abs(currentGamepad2.right_stick_y) > 0.02) && !runningHomeHori || currentGamepad2.right_stick_button) {
                horiSlideIsRunningToPos = false;
                runningHomeHori=false;
            }
            if (horiSlideIsRunningToPos || runningHomeHori) {
                horizontalSlides.setPosition(horiSlidesTarget,1);
            } else {
                /////hori manual control
                double currentPosition = horizontalSlides.getCurrentPosition();
                double goingOutInput = Math.max(-currentGamepad2.right_stick_y, currentGamepad1.right_trigger);
                double goingInInput = Math.max(currentGamepad2.right_stick_y, currentGamepad1.left_trigger);
                double movementInput = (goingInInput > 0) ? -goingInInput : goingOutInput;
                double newTargetPosition = currentPosition + (movementInput * MAX_INPUT_SCALING);
                //newTargetPosition = Math.max(horiSlides.MIN_POSITION, Math.min(newTargetPosition, horiSlides.MAX_POSITION));
                horizontalSlides.setPosition(newTargetPosition,1);
            }


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if ((currentGamepad2.y && !previousGamepad2.y) || (currentGamepad1.y && !previousGamepad1.y)){
                if (Alliance.equals("Red")){
                    Alliance = "Blue";
                    rejectAlliance = "Red";
                } else {
                    Alliance = "Red";
                    rejectAlliance = "Blue";
                }
            }
            telemetry.addData("Alliance" , Alliance);
            telemetry.update();
        }


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            loopTime = getRuntime() - previousStartTime;
            previousStartTime = getRuntime();


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if ((currentGamepad1.y && !previousGamepad1.y) || (currentGamepad2.y && !previousGamepad2.y)){
                if (otherReject.equals("Yellow")){
                    otherReject = Alliance;
                } else {
                    otherReject = "Yellow";
                }
            }

            if (otherReject != "Yellow") { //target is yellow
                target = "Yellow";
            } else if (Alliance.equals("Red")){ //target is red
                target = "Red";
            } else { //target is blue
                target = "Blue";
            }
            if (i<100){
                if (target.equals("Red")) {
                    gamepad1.setLedColor(255,0,0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(255,0,0, LED_DURATION_CONTINUOUS);
                }
                if (target.equals("Yellow")) {
                    gamepad1.setLedColor(255,255,0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(255,255,0, LED_DURATION_CONTINUOUS);
                }
                if (target.equals("Blue")) {
                    gamepad1.setLedColor(0,0,255, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(0,0,255, LED_DURATION_CONTINUOUS);
                }

                i++;
            }else {
                if (!blocking) {
                    gamepad1.setLedColor(255,0,0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(255,0,0, LED_DURATION_CONTINUOUS);
                } else {
                    gamepad1.setLedColor(0,255,0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(0,255,0, LED_DURATION_CONTINUOUS);
                }
                i++;
                if (i >200){
                    i=0;
                }
            }






            ////////////////////////////// Drive code ///////////////////////////////////
            double y = -currentGamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = currentGamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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


            ////////////////////////////// Intake controls ///////////////////////////////////
            if ((Math.abs(currentGamepad2.right_stick_y) > 0.02) && !runningHomeHori || currentGamepad2.right_stick_button) {
                horiSlideIsRunningToPos = false;
                runningHomeHori=false;
            }
            if (horiSlideIsRunningToPos || runningHomeHori) {
                horizontalSlides.setPosition(horiSlidesTarget,1);
            } else {
                /////hori manual control
                double currentPosition = horizontalSlides.getCurrentPosition();
                double goingOutInput = Math.max(-currentGamepad2.right_stick_y, currentGamepad1.right_trigger);
                double goingInInput = Math.max(currentGamepad2.right_stick_y, currentGamepad1.left_trigger);
                double movementInput = (goingInInput > 0) ? -goingInInput : goingOutInput;
                double newTargetPosition = currentPosition + (movementInput * MAX_INPUT_SCALING);
                newTargetPosition = Math.max(horiSlides.MIN_POSITION, Math.min(newTargetPosition, horiSlides.MAX_POSITION));
                horizontalSlides.setPosition(newTargetPosition,1);
            }

/*
            if ((currentGamepad1.a && !previousGamepad1.a) ||(currentGamepad2.left_bumper && !previousGamepad2.left_bumper)) {
                if (blocking){
                    blocking = false;
                    intake.open(true);
                } else {
                    blocking = true;
                    intake.open(false);
                }
            }

            if ((currentGamepad1.b && !previousGamepad1.b) || (currentGamepad2.b && !previousGamepad2.b)) {
                intake.setSpeed(0.0, 0.0);
                horizontalSlides.setPosition(0,1);
                intake.intakeRotatedUp(true);
                outtake.bucketAtIntakePos(true);
                intakeRotationUp = true;
                runningHomeHori = true;
            }

            if (blocking && !intake.getDetectedColor().equals("NA") && intakeRotationUp == false){
                intake.setSpeed(0,0);
                gamepad2.runRumbleEffect(effect);
                gamepad1.runRumbleEffect(effect);
            } else {
                if (currentGamepad2.right_trigger>currentGamepad2.left_trigger){
                    intake.setSpeed(-currentGamepad2.right_trigger, currentGamepad2.right_trigger);
                } else {
                    intake.setSpeed(currentGamepad2.left_trigger, -currentGamepad2.left_trigger);
                }
            }

*//*
            if (runningHomeHori && horizontalSlides.getCurrentPosition() < 50) {
                dropInBasketTimer = getRuntime();
                dropInBasketDelay = true;
                runningHomeHori=false;
                intake.setSpeed(0.7, -0.7);
            }
            if (dropInBasketDelay && (dropInBasketTimer + 0.6) < getRuntime()) {
                intake.setSpeed(0.0, 0.0);
                dropInBasketDelay = false;
            }

            if ((currentGamepad2.right_bumper) && !slightDelayToRotate){
                roationDelayTimer = getRuntime();
                slightDelayToRotate = true;
                if (intakeRotationUp){
                    intakeRotationUp = false;
                    intake.intakeRotatedUp(false);
                    intake.setSpeed(1.0, -1.0);
                } else {
                    intakeRotationUp = true;
                    intake.intakeRotatedUp(true);
                    intake.setSpeed(0.0, 0.0);
                }
            }
            if (slightDelayToRotate && !intakeRotationUp){
                intake.setSpeed(1.0, -1.0);
            }
            if (slightDelayToRotate && (roationDelayTimer + 0.4) < getRuntime()){
                slightDelayToRotate = false;
            }

*/
            ////////////////////////////// Outtake controls ///////////////////////////////////
            if (Math.abs(currentGamepad2.left_stick_y) > 0.02) {
                vertSlideIsRunningToPos = false;
            } else {
                vertSlideIsRunningToPos = true;
            }
            if (vertSlideIsRunningToPos){
                verticalSlides.setPosition(vertSlidesTarget);
            } else{
                /////vert manual control
                double currentPositionvert = verticalSlides.getCurrentPosition();
                vertSlidesTarget = currentPositionvert + -currentGamepad2.left_stick_y * MAX_INPUT_SCALING;
                vertSlidesTarget = Math.max(vertSlide.MIN_POSITION, Math.min(vertSlidesTarget, vertSlide.MAX_POSITION));
                verticalSlides.setPosition(vertSlidesTarget);
            }

            ////////////////////Outake Yellow handling Logic ////////////////

            /*
            if (otherReject != "Yellow"){
                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)){
                    outtake.hookAtIntake(true);
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MAX_POSITION;
                }
            }

            ////////////////////Outake Alliance handling Logic ////////////////
            if (otherReject == "Yellow"){
                if ((!currentGamepad1.left_bumper && previousGamepad1.left_bumper)){
                    outtake.clawOpen(false);
                    specimenInClaw = true;
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION +150;
                    readyToGoToDelivery = true;
                    outtake.bucketAtIntakePos(false);
                }
                if (currentGamepad1.right_bumper && previousGamepad1.right_bumper){
                    outtake.hookAtIntake(true);
                    outtake.clawOpen(true);
                    specimenInClaw = false;
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION;
                    readyToGoToDelivery = false;
                    outtake.bucketAtIntakePos(true);
                }

                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up)||(currentGamepad2.dpad_up && !previousGamepad2.dpad_up)){
                    if (readyToGoToDelivery){
                        outtake.hookAtIntake(false);
                        vertSlideIsRunningToPos = true;
                        vertSlidesTarget = verticalSlides.MIN_POSITION +450;
                        readyToGoToDelivery = false;
                        readyToDeliver = true;
                        sampleDelivTimer = getRuntime();
                    } else if (readyToDeliver && sampleDelivTimer + 1.0 < getRuntime()){
                        vertSlideIsRunningToPos = true;
                        vertSlidesTarget = verticalSlides.MIN_POSITION +650;
                        deliveringSpecimen = true;
                        readyToDeliver = false;
                        specimenHooked = true;
                    }
                }
                if (((currentGamepad1.dpad_down && !previousGamepad1.dpad_down) || (currentGamepad2.dpad_down && !previousGamepad2.dpad_down))&& specimenHooked){
                    outtake.clawOpen(true);
                    specimenHooked = false;
                    specimenDelivTimer = getRuntime();
                    specimenRunTimer = true;
                }
                if (specimenRunTimer && specimenDelivTimer +0.2 <getRuntime()){
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION +200;
                }
                if (specimenRunTimer && specimenDelivTimer +0.5 <getRuntime()){
                    outtake.clawOpen(false);
                }
                if (specimenRunTimer && specimenDelivTimer +0.9 <getRuntime()){
                    outtake.clawOpen(false);
                    outtake.hookAtIntake(true);
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION;
                }
                if (specimenRunTimer && specimenDelivTimer +1.8 <getRuntime()){
                    outtake.clawOpen(true);
                    specimenRunTimer = false;
                }


            }

            if (currentGamepad1.x){
                outtake.clawOpen(false);
            }



            if (currentGamepad1.dpad_right || currentGamepad2.dpad_right){
                outtake.bucketAtIntakePos(false);
            }
            if (currentGamepad1.dpad_left || currentGamepad2.dpad_left){
                outtake.bucketAtIntakePos(true);
            }



*/
            //intake.update();
            horizontalSlides.update();
            verticalSlides.update();
            //outtake.update();

            telemetry.addData("Sample in bucket", sampleInBasket);
            telemetry.addData("Vert current", verticalSlides.getCurrentPosition());
            telemetry.addData("Vert min pos", verticalSlides.MIN_POSITION);
            telemetry.addData("Vert target", vertSlidesTarget);
            //telemetry.addData("Current intake", intake.getDetectedColor());
            telemetry.addData("Target colour", target);
            telemetry.addData("Target colour", Alliance);
            telemetry.addData("Reject colour", otherReject);
            telemetry.addData("Loop time", loopTime);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
}
