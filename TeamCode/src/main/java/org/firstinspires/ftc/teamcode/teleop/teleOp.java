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
import java.util.Objects;

import org.firstinspires.ftc.teamcode.functions.horiSlides;
import org.firstinspires.ftc.teamcode.functions.outtake;
import org.firstinspires.ftc.teamcode.functions.intake;
import org.firstinspires.ftc.teamcode.functions.vertSlide;

@TeleOp
public class teleOp extends LinearOpMode {

    private static final double MAX_INPUT_SCALING = 200;
    private static final double JOYSTICK_DEADZONE = 0.05;

    // Slide target position (now fixed numbers: 400, 200, 0, etc.)
    private double horiSlidesTarget = 0;
    private double vertSlidesTarget = 0;

    // Gamepad state copies
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // Loop timing
    double loopTime = 0;

    // Rejection state variables
    private boolean rejecting = false;
    private double rejectionEndTime = 0.0;  // time when a non-target piece was last detected

    // Go Home (target delivery) state variables
    private boolean goingHome = false;
    private double goHomeWaitStart = 0; // used when slide is below 200: wait 0.4 sec before retracting

    private double specBucketRunTimer = 0.0;
    private boolean specRunTimeRunning = false;


    public boolean specimenInClaw;
    public boolean deliveringSpecimen = false;
    public double specimenDelivTimer;
    public boolean specimenRunTimer = false;
    public double sampleDelivTimer;
    public boolean specimenHooked = false;
    public boolean readyToGoToDelivery = false;
    public boolean readyToDeliver = false;
    private boolean vertSlideIsRunningToPos = false;
    private String previousIntake, thisIntake;

    public boolean sampleInBucket = false;

    double previousStartTime = getRuntime();

    double specTiming = getRuntime();
    double loopTimeSum = 0.0;
    int loopCount = 0;
    double highestLoopTime = 0.0;
    double lowestLoopTime = Double.MAX_VALUE;
    String Alliance = "";

    boolean readyToTakeADump = false;
    boolean dumpCompleted = false;

    private int i = 0;

    boolean previousWasTarget = false;
    boolean currentIsTarget = false;
    boolean doubleDoubleTarget = false;



    @Override
    public void runOpMode() throws InterruptedException {
        // Set bulk caching mode for all hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ------------------- DRIVE MOTOR INITIALIZATION -------------------
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack  = hardwareMap.dcMotor.get("leftBack");
        DcMotor rightFront= hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // ------------------- SUBSYSTEM INITIALIZATION -------------------
        horiSlides horizontalSlides = new horiSlides(hardwareMap);
        vertSlide verticalSlides = new vertSlide(hardwareMap);
        outtake outtake = new outtake(hardwareMap);
        intake intake = new intake(hardwareMap);

        // Initialize intake and outtake positions.
        // Default manual configuration (right trigger): inner blocker closed, outer blocker open.
        intake.setRotation(TUCKED);
        intake.setInnerBlockOpen(false);
        intake.setOuterBlockOpen(true);
        // Maintain target values continuously.
        intake.setTarget(1, 0, 0);

        outtake.hookAtIntake(true);
        outtake.clawOpen(true);
        outtake.specDropAtIntakePos(true);
        outtake.specDropOpen(false);

        intake.setTarget(0, 1, 0);
        Alliance = "Red";

        while (!isStarted() && !isStopRequested()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if ((currentGamepad2.touchpad && !previousGamepad2.touchpad) || (currentGamepad1.touchpad && !previousGamepad1.touchpad)){
                if (intake.target2.equals("Red")){
                    intake.setTarget(1, 0, 1);
                    Alliance = "Blue";
                } else {
                    intake.setTarget(1, 1, 0);
                    Alliance = "Red";
                }
            }
            telemetry.addData("Alliance" , intake.target2);
            telemetry.update();

            if (intake.target2.equals("Red")) {
                gamepad1.setLedColor(255,0,0, LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(255,0,0, LED_DURATION_CONTINUOUS);
            }
            if (intake.target2.equals("Blue")) {
                gamepad1.setLedColor(0,0,255, LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(0,0,255, LED_DURATION_CONTINUOUS);
            }
            previousStartTime = getRuntime();
        }

        while (opModeIsActive()) {
            double currentTime = getRuntime();
            double loopTime = currentTime - previousStartTime;
            previousStartTime = currentTime;

            // Update running totals and count
            loopCount++;
            loopTimeSum += loopTime;
            double avgLoopTime = loopTimeSum / loopCount;

            // Update highest and lowest values
            if (loopTime > highestLoopTime) {
                highestLoopTime = loopTime;
            }
            if (loopTime < lowestLoopTime) {
                lowestLoopTime = loopTime;
            }


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            previousIntake = thisIntake;
            thisIntake = intake.getDetectedColor();


            previousWasTarget = currentIsTarget;
            currentIsTarget = intake.isTarget();

            if (previousWasTarget && currentIsTarget){
                doubleDoubleTarget = true;
            } else {
                doubleDoubleTarget = false;
            }

            // setting target
            if ((currentGamepad1.left_stick_button && !previousGamepad1.left_stick_button) || (currentGamepad2.left_stick_button && !previousGamepad2.left_stick_button)){
                if (intake.target1.equals("Yellow")){ //currently a target so change to not a target
                    if (intake.target2.equals("null")){
                        intake.target2 = Alliance;
                        intake.target1 = "null";
                    } else {
                        intake.target1 = "null";
                    }
                } else {
                    intake.target1 = "Yellow";
                }
            }
            if ((currentGamepad1.right_stick_button && !previousGamepad1.right_stick_button) || (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button)){
                if (intake.target2.equals(Alliance)){ //currently a target so change to not a target
                    if (intake.target1.equals("null")){
                        intake.target1 = "Yellow";
                        intake.target2 = "null";
                    } else {
                        intake.target2 = "null";
                    }
                } else {
                    intake.target2 = Alliance;
                }
            }

            // Alternate between the two.
            if (intake.target1.equals("Yellow") && !intake.target2.equals("null") ) {
                if ( i < 50 ) {
                    // For the first half of the cycle, show target2's colour
                    if (intake.target2.equals("Blue") ) {
                        gamepad1.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
                        gamepad2.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
                    } else if (intake.target2.equals("Red") ) {
                        gamepad1.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
                        gamepad2.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
                    }
                } else {
                    // For the second half, show yellow.
                    gamepad1.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
                }
                i++;
                if ( i > 100 ) {
                    i = 0;
                }
            }
// Case 2: Only yellow is active.
            else if (intake.target1.equals("Yellow") && intake.target2.equals("null") ) {
                gamepad1.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
                gamepad2.setLedColor(255, 255, 0, LED_DURATION_CONTINUOUS);
            }
// Case 3: Only target2 is active (and it can be "Blue" or "Red").
            else if (intake.target1.equals("null") && !intake.target2.equals("null") ) {
                if (intake.target2.equals("Blue") ) {
                    gamepad1.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(0, 0, 255, LED_DURATION_CONTINUOUS);
                } else if (intake.target2.equals("Red") ) {
                    gamepad1.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
                    gamepad2.setLedColor(255, 0, 0, LED_DURATION_CONTINUOUS);
                }
            }

            // ------------------- DRIVE CONTROL -------------------
            double y  = -currentGamepad1.left_stick_y;
            double x  = currentGamepad1.left_stick_x * 1.1;
            double rx = currentGamepad1.right_stick_x;
            if (Math.abs(rx)<0.8){
                rx= rx*0.7;
            }
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower  = (y + x + rx) / denominator;
            double leftBackPower   = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower  = (y + x - rx) / denominator;
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            // ------------------- RIGHT BUMPER TO TOGGLE INTAKE DEPLOYMENT -------------------
            if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                if (horizontalSlides.getCurrentPosition()> horizontalSlides.MIN_POSITION +60) {

                    if (intake.getRotationMode().equalsIgnoreCase("TUCKED")) {
                        intake.setRotation(INTAKE);
                        intake.setTimedIntake(-1, -1, 0.4);
                        intake.setInnerBlockOpen(true);
                        intake.setOuterBlockOpen(true);
                    } else {
                        // If already deployed, then if nothing is detected, tuck the intake.
                        if (Objects.equals(intake.getDetectedColor(), "NA")) {
                            if (intake.getRotationMode().equalsIgnoreCase("TRANSFER")) {
                                intake.setRotation(INTAKE);
                                intake.setTimedIntake(1, 1, 0.4);
                                intake.setInnerBlockOpen(true);
                                intake.setOuterBlockOpen(true);
                            } else {
                                intake.setRotation(TUCKED);
                            }
                        } else {
                            // Otherwise, toggle between INTAKE and TRANSFER.
                            if (intake.getRotationMode().equalsIgnoreCase("INTAKE"))
                                intake.setRotation(TRANSFER);
                            else
                                intake.setRotation(INTAKE);
                            // Keep both blockers open while a piece is present.
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(true);
                        }
                    }
                } else {
                    horiSlidesTarget = horizontalSlides.MIN_POSITION + 60;
                }

            }

            // ------------------- MAIN INTAKE LOGIC (when not in goHome sequence) -------------------
            if (!readyToTakeADump && !goingHome && intake.getRotationMode().equalsIgnoreCase("INTAKE")) {
                if (!Objects.equals(intake.getDetectedColor(), "NA")) {
                    if (doubleDoubleTarget) {
                        // TARGET PIECE: Stop intake immediately and start goHome sequence.
                        intake.setSpeed(0, 0);
                        goingHome = true;
                        goHomeWaitStart = currentTime;
                    } else {
                        // NON-TARGET: Reject it.
                        // Force both blockers open so the piece passes through.
                        intake.setInnerBlockOpen(true);
                        intake.setOuterBlockOpen(true);
                        // Use full-speed rejection (in the same direction as the manual command).
                        double rejectionSpeed = (currentGamepad2.right_trigger > currentGamepad2.left_trigger) ? 1.0 : -1.0;
                        intake.setSpeed(rejectionSpeed, rejectionSpeed);
                        rejecting = true;
                        rejectionEndTime = currentTime;
                    }
                } else {
                    // No piece detected.
                    if (rejecting) {
                        // Wait 0.2 sec after the piece disappears before restoring manual control.
                        if (currentTime - rejectionEndTime >= 0.2) {
                            if (currentGamepad2.right_trigger > 0.1) {
                                intake.setInnerBlockOpen(false);
                                intake.setOuterBlockOpen(true);
                            } else if (currentGamepad2.left_trigger > 0.1) {
                                intake.setInnerBlockOpen(true);
                                intake.setOuterBlockOpen(false);
                            }
                            rejecting = false;
                            intake.setSpeed(0, 0);
                        } else {
                            double rejectionSpeed = (currentGamepad2.right_trigger > currentGamepad2.left_trigger) ? 1.0 : -1.0;
                            intake.setSpeed(rejectionSpeed, rejectionSpeed);
                        }
                    } else {
                        // NORMAL MANUAL INTAKE CONTROL.
                        if (currentGamepad2.right_trigger > 0.1) {
                            intake.setInnerBlockOpen(false);
                            intake.setOuterBlockOpen(true);
                            intake.setSpeed(currentGamepad2.right_trigger, currentGamepad2.right_trigger);
                        } else if (currentGamepad2.left_trigger > 0.1) {
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(false);
                            intake.setSpeed(-currentGamepad2.left_trigger, -currentGamepad2.left_trigger);
                        } else {
                            intake.setSpeed(0, 0);
                        }
                    }
                }
            }



            if (goingHome && thisIntake.equals("NA") && previousIntake.equals("NA")){
                goingHome = false;
            }


            // ------------------- GO HOME (TARGET DELIVERY) SEQUENCE -------------------
            if (!readyToTakeADump && goingHome) {
                // Stop the intake.
                intake.setSpeed(0, 0);
                outtake.specDropAtIntakePos(true);
                outtake.specDropOpen(false);

                double slidePos = horizontalSlides.MIN_POSITION + horizontalSlides.getCurrentPosition();

                // If the slide is extended beyond 400, command retraction to 400.
                if (slidePos > 200) {
                    horiSlidesTarget = horizontalSlides.MIN_POSITION + 0.1;
                }

                if (slidePos<(horizontalSlides.MIN_POSITION +700)){
                    intake.setRotation(TRANSFER);
                    intake.setInnerBlockOpen(true);
                    intake.setOuterBlockOpen(true);
                }
                if (slidePos < (horizontalSlides.MIN_POSITION + 200) && currentTime - goHomeWaitStart >= 0.4){
                    horiSlidesTarget = horizontalSlides.MIN_POSITION + 0.1;
                }
                // When the slide is nearly retracted (below 25), run the intake wheels for 0.8 sec
                // to drop the piece. Do not change the intake rotation; leave it in TRANSFER.
                if (slidePos < (horizontalSlides.MIN_POSITION + 25)) {
                    if (!outtake.dumpingYellows || !intake.getDetectedColor().equals("Yellow")) {
                        intake.setTimedIntake(-1, -1, 0.8);
                        sampleInBucket=true;
                    }
                    if (outtake.dumpingYellows && intake.getDetectedColor().equals("Yellow")){
                        readyToTakeADump = true;
                    }
                    goingHome = false;
                }
            }

            if (!goingHome && !readyToTakeADump) {
                double currentHoriPos = horizontalSlides.getCurrentPosition();
                double horiInput = -currentGamepad2.right_stick_y;

                if (Math.abs(horiInput) > JOYSTICK_DEADZONE) {
                    if (Math.abs(horiInput) < 0.8){
                        horiInput = horiInput*0.18;
                    }
                    if (intake.getRotationMode().equalsIgnoreCase("INTAKE")) {
                        horiSlidesTarget = currentHoriPos + (horiInput * MAX_INPUT_SCALING);
                        horiSlidesTarget = Math.max(horiSlidesTarget, horizontalSlides.MIN_POSITION + 120);
                    } else {
                        horiSlidesTarget = currentHoriPos + (horiInput * MAX_INPUT_SCALING);
                    }

                } else {
                    horiSlidesTarget = currentHoriPos;
                }

            }

            if (outtake.dumpingYellows && readyToTakeADump){
                if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper){
                    horiSlidesTarget = horizontalSlides.MAX_POSITION;
                }
                if (horizontalSlides.MIN_POSITION + horizontalSlides.getCurrentPosition() > horizontalSlides.MIN_POSITION + 150 && !dumpCompleted){
                    intake.setInnerBlockOpen(true);
                    intake.setOuterBlockOpen(true);
                    intake.setRotation(INTAKE);
                }
                if (horizontalSlides.MIN_POSITION + horizontalSlides.getCurrentPosition() > horizontalSlides.MAX_POSITION -100 && !currentGamepad2.left_bumper && previousGamepad2.left_bumper){
                    intake.setTimedIntake(-1, -1, 0.8);
                    intake.setRotation(TUCKED);
                    horiSlidesTarget = horizontalSlides.MIN_POSITION + 1;
                    dumpCompleted = true;
                }
                if (dumpCompleted && (horizontalSlides.MIN_POSITION + horizontalSlides.getCurrentPosition()) < horizontalSlides.MIN_POSITION + 50){
                    readyToTakeADump = false;
                    dumpCompleted = false;
                }
            }

            horizontalSlides.setPosition(horiSlidesTarget);


            if (specRunTimeRunning && specBucketRunTimer +0.6 < currentTime){
                outtake.specDropAtIntakePos(true);
                outtake.specDropOpen(false);
                specRunTimeRunning = false;
                sampleInBucket = false;
            }

            if ((currentGamepad2.a && !previousGamepad2.a) && !specimenInClaw) {
                if (outtake.BucketPositionAtIntake){
                    outtake.specDropAtIntakePos(false);
                } else{
                    outtake.specDropAtIntakePos(true);
                    outtake.specDropOpen(false);
                    specRunTimeRunning = false;
                }
            }

            if (currentGamepad2.b && !previousGamepad2.b) {
                if (!outtake.BucketPositionAtIntake){
                    outtake.specDropOpen(true);
                    specBucketRunTimer = currentTime;
                    specRunTimeRunning = true;
                }
            }



            // ------------------- MANUAL VERTICAL SLIDE CONTROL -------------------




            ////////////////////////////// Outtake controls ///////////////////////////////////
            if ((Math.abs(currentGamepad2.left_stick_y) > 0.02) || (Math.abs(currentGamepad1.left_trigger)> 0.02) || (Math.abs(currentGamepad1.right_trigger)> 0.02)) {
                vertSlideIsRunningToPos = false;
            } else {
                vertSlideIsRunningToPos = true;
            }
            if (vertSlideIsRunningToPos){
                verticalSlides.setPosition(vertSlidesTarget);
            } else{
                /////vert manual control
                double currentVertPos = verticalSlides.getCurrentPosition();
                // Read the raw input values
                double leftTrigger  = currentGamepad1.left_trigger;
                double rightTrigger = currentGamepad1.right_trigger;
                double leftStick    = currentGamepad2.left_stick_y;

                double stickVal;
                if (Math.abs(leftTrigger) >= Math.abs(rightTrigger) && Math.abs(leftTrigger) >= Math.abs(leftStick)) {
                    // If left trigger is highest, invert its value
                    stickVal = -leftTrigger*0.2;
                } else if (Math.abs(rightTrigger) >= Math.abs(leftTrigger) && Math.abs(rightTrigger) >= Math.abs(leftStick)) {
                    // If right trigger is highest, use it as is
                    stickVal = rightTrigger*0.08;
                } else {
                    // Otherwise, use the left stick value
                    if (Math.abs(leftStick) < 0.6){
                        stickVal = leftStick/2;
                    } else {
                        stickVal = leftStick;
                    }
                }

                if (Math.abs(stickVal) > JOYSTICK_DEADZONE) {
                    vertSlidesTarget = currentVertPos + (-stickVal * MAX_INPUT_SCALING);
                }
                verticalSlides.setPosition(vertSlidesTarget);
            }

            ////////////////////Outake Yellow handling Logic ////////////////
            if (outtake.scoringSamples){
                if ((currentGamepad1.dpad_up && !previousGamepad1.dpad_up) || (currentGamepad2.dpad_up && !previousGamepad2.dpad_up)){
                    outtake.hookAtIntake(true);
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MAX_POSITION;
                }
            }

            ////////////////////Outake Alliance handling Logic ////////////////
            if (outtake.scoringSpecs){
                if ((currentGamepad1.right_bumper && !previousGamepad1.right_bumper && !specimenInClaw)){
                    if (sampleInBucket){
                        if (!outtake.BucketPositionAtIntake){
                            outtake.specDropOpen(true);
                            specBucketRunTimer = currentTime;
                            specRunTimeRunning = true;
                        } else {
                            outtake.specDropAtIntakePos(false);
                        }
                    }
                    outtake.clawOpen(false);
                    specimenInClaw = true;
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION +150;
                    readyToGoToDelivery = true;
                    specTiming = currentTime;
                } else if ((currentGamepad1.y && !previousGamepad1.y)||(currentGamepad2.dpad_up && !previousGamepad2.dpad_up) || (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) ){
                    if (readyToGoToDelivery){
                        outtake.hookAtIntake(false);
                        vertSlideIsRunningToPos = true;
                        vertSlidesTarget = verticalSlides.MIN_POSITION +450;
                        readyToGoToDelivery = false;
                        readyToDeliver = true;
                        sampleDelivTimer = currentTime;
                    } else if (readyToDeliver && sampleDelivTimer + 1.0 < currentTime){
                        vertSlideIsRunningToPos = true;
                        vertSlidesTarget = verticalSlides.MIN_POSITION +650;
                        deliveringSpecimen = true;
                        readyToDeliver = false;
                        specimenHooked = true;
                    } else if (specimenHooked){
                        outtake.clawOpen(true);
                        specimenHooked = false;
                        specimenInClaw = false;
                        specimenDelivTimer = currentTime;
                        specimenRunTimer = true;
                    }
                }
                if (currentGamepad1.left_bumper && previousGamepad1.left_bumper){
                    outtake.hookAtIntake(true);
                    outtake.clawOpen(true);
                    specimenInClaw = false;
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION +1;
                    readyToGoToDelivery = false;
                }


                if (((currentGamepad1.a && !previousGamepad1.a) || (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) || (currentGamepad1.left_bumper && !previousGamepad1.left_bumper))&& specimenHooked ){
                    outtake.clawOpen(true);
                    specimenHooked = false;
                    specimenDelivTimer = currentTime;
                    specimenRunTimer = true;
                    specimenInClaw = false;
                }
                if (specimenRunTimer && specimenDelivTimer +0.2 <currentTime){
                    vertSlideIsRunningToPos = true;
                    vertSlidesTarget = verticalSlides.MIN_POSITION +1;
                }
                if (specimenRunTimer && specimenDelivTimer +0.7 <currentTime){
                    outtake.hookAtIntake(true);
                    specimenRunTimer = false;
                }

            }

            if (currentGamepad1.x){
                outtake.clawOpen(true);
            }







            // ------------------- SUBSYSTEM UPDATES -------------------
            horizontalSlides.update();
            verticalSlides.update();
            intake.update();
            outtake.update();

            // ------------------- TELEMETRY -------------------
            telemetry.addData("Slide Pos", horizontalSlides.getCurrentPosition());
            telemetry.addData("Slide Target", horiSlidesTarget);
            telemetry.addData("Intake Rotation", intake.getRotationMode());
            telemetry.addData("Rejecting", rejecting);
            telemetry.addData("GoHome", goingHome);
            telemetry.addData("Loop Time", loopTime);
            telemetry.addData("Average Loop Time", avgLoopTime);
            telemetry.addData("Highest Loop Time", highestLoopTime);
            telemetry.addData("Lowest Loop Time", lowestLoopTime);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
    private boolean risingEdge(String Gamepadx){

        return true;
    }
}

