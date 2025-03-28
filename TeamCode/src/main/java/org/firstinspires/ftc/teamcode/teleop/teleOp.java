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

import org.firstinspires.ftc.teamcode.functions.TargetState;
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
    TargetState thisIntake;

    public boolean sampleInBucket = false;

    double previousStartTime = getRuntime();

    double specTiming = getRuntime();
    double loopTimeSum = 0.0;
    int loopCount = 0;
    double highestLoopTime = 0.0;
    double lowestLoopTime = 1.0;
    public enum AllianceColor {
        RED,
        BLUE
    }
    private AllianceColor alliance = AllianceColor.RED;


    private int i = 0;

    boolean doubleDoubleTarget = false;
    boolean doubleDouble = false;
    double avgLoopTime = 0;
    boolean scoringSpecs;
    boolean readyToTransferYellow = false;
    boolean transferringYellow = false;
    double yellowTransferTime;
    boolean yellowReadyToRelease = false;
    int yellowProcess = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        // Set bulk caching mode for all hubs
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // ------------------- DRIVE MOTOR INITIALIZATION -------------------
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftBack  = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightFront= hardwareMap.dcMotor.get("rightFront");
        DcMotor rightBack = hardwareMap.dcMotor.get("rightRear");

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

        outtake.hookAtIntake(true,false);
        outtake.clawOpen(true);
        outtake.specDropAtIntakePos(true);
        outtake.specDropOpen(false);

        outtake.setSampleOutOfWay();
        outtake.sampleReleaseOpen(false);

        intake.setTarget(0, 1, 0);
        scoringSpecs = true;

        while (!isStarted() && !isStopRequested()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Toggle alliance on touchpad press.
            if ((currentGamepad2.touchpad && !previousGamepad2.touchpad) ||
                    (currentGamepad1.touchpad && !previousGamepad1.touchpad)) {
                if (intake.target2 == TargetState.RED) {
                    intake.setTarget(0, 0, 1);
                    alliance = AllianceColor.BLUE;
                } else {
                    intake.setTarget(0, 1, 0);
                    alliance = AllianceColor.RED;
                }
            }

            // Set LED based on alliance.
            int[] color = getAllianceColorForEnum(intake.target2);

            setGamepadLedColor(color[0], color[1], color[2]);

            telemetry.addData("Alliance", alliance.toString());
            telemetry.update();
        }

        while (opModeIsActive()) {

            double currentHoriPos = horizontalSlides.getCurrentPosition();
            String rotationMode = intake.getRotationMode();
            TargetState detectedColor = intake.getDetectedColor();

            double slidePos = horiSlides.MIN_POSITION + currentHoriPos;
            double rejectionSpeed = (currentGamepad2.right_trigger > currentGamepad2.left_trigger) ? 1.0 : -1.0;

            boolean isTucked = rotationMode.equalsIgnoreCase("TUCKED");
            boolean isTransfer = rotationMode.equalsIgnoreCase("TRANSFER");
            boolean isIntake = rotationMode.equalsIgnoreCase("INTAKE");



            double currentTime = getRuntime();

            if (loopCount > 0) {
                double loopTime = currentTime - previousStartTime;

                //loopTimeSum += loopTime;
                //loopCount++;
                //avgLoopTime = loopTimeSum / loopCount;

                if (loopTime > highestLoopTime) {
                    highestLoopTime = loopTime;
                }
                if (loopTime < lowestLoopTime) {
                    lowestLoopTime = loopTime;
                }
            } else {
                loopCount++;
            }

            previousStartTime = currentTime;


            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            TargetState previousIntake = thisIntake;
            thisIntake = detectedColor;

            doubleDouble = Objects.equals(previousIntake, thisIntake);

            doubleDoubleTarget = doubleDouble && intake.isTarget();

            if ((currentGamepad2.touchpad && !previousGamepad2.touchpad) ||
                    (currentGamepad1.touchpad && !previousGamepad1.touchpad)) {
                if (intake.target2 == TargetState.NONE) {
                    intake.target1 = TargetState.NONE;
                    intake.target2 = allianceTarget(alliance);
                    scoringSpecs = true;
                } else {
                    intake.target1 = TargetState.YELLOW;
                    intake.target2 = TargetState.NONE;
                    scoringSpecs = false;
                }
            }

            if (scoringSpecs){
                outtake.setSampleOutOfWay();
            }


            // Alternating LED display logic based on target selection.
            if (intake.target1 == TargetState.YELLOW && intake.target2 != TargetState.NONE) {
                // Blink: first half shows alliance color, second half shows yellow.
                if (i < 50) {
                    int[] allianceColor = getAllianceColorForEnum(intake.target2);
                    setGamepadLedColor(allianceColor[0], allianceColor[1], allianceColor[2]);
                } else {
                    setGamepadLedColor(255, 255, 0);
                }
                i = (i + 1) % 100;
            } else if (intake.target1 == TargetState.YELLOW) {
                setGamepadLedColor(255, 255, 0);
            } else if (intake.target1 == TargetState.NONE && intake.target2 != TargetState.NONE) {
                int[] allianceColor = getAllianceColorForEnum(intake.target2);
                setGamepadLedColor(allianceColor[0], allianceColor[1], allianceColor[2]);
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
                if (currentHoriPos> horiSlides.MIN_POSITION +60) {

                    if (isTucked) {
                        intake.setRotation(INTAKE);
                        intake.setTimedIntake(-1, -1, 0.4);
                        intake.setInnerBlockOpen(true);
                        intake.setOuterBlockOpen(true);
                    } else {
                        // If already deployed, then if nothing is detected, tuck the intake.
                        if (detectedColor == TargetState.NONE) {
                            if (isTransfer) {
                                intake.setRotation(INTAKE);
                                intake.setTimedIntake(1, 1, 0.4);
                                intake.setInnerBlockOpen(true);
                                intake.setOuterBlockOpen(true);
                            } else {
                                intake.setRotation(TUCKED);
                            }
                        } else {
                            // Otherwise, toggle between INTAKE and TRANSFER.
                            if (isIntake) {
                                intake.setRotation(TRANSFER);
                            } else {
                                intake.setRotation(INTAKE);
                            }
                            // Keep both blockers open while a piece is present.
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(true);
                        }
                    }
                } else {
                    horiSlidesTarget = horiSlides.MIN_POSITION + 60;
                }

            }
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up){
                intake.setInnerBlockOpen(true);
                intake.setOuterBlockOpen(true);
                intake.setRotation(TRANSFER);
            }
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down){
                intake.setInnerBlockOpen(true);
                intake.setOuterBlockOpen(true);
                intake.setRotation(TUCKED);
            }

            // ------------------- MAIN INTAKE LOGIC (when not in goHome sequence) -------------------
            if (!goingHome && rotationMode.equalsIgnoreCase("INTAKE")) {
                if (detectedColor != TargetState.NONE && detectedColor != TargetState.MIXED && doubleDouble) {
                    if (doubleDoubleTarget) {
                        // TARGET PIECE: Stop intake immediately and start goHome sequence.
                        intake.setSpeed(0, 0);
                        goingHome = true;
                        goHomeWaitStart = currentTime;
                        rejecting = false;
                    } else {
                        // NON-TARGET: Reject it.
                        // Force both blockers open so the piece passes through.
                        intake.setInnerBlockOpen(true);
                        intake.setOuterBlockOpen(true);
                        // Use full-speed rejection (in the same direction as the manual command).
                        intake.setSpeed(rejectionSpeed, rejectionSpeed);
                        rejecting = true;
                        rejectionEndTime = currentTime;
                    }
                } else {
                    // No piece detected.
                    if (rejecting) {
                        // Wait 0.2 sec after the piece disappears before restoring manual control.
                        if (currentTime - rejectionEndTime >= 0.05) {
                            if (currentGamepad2.right_trigger > 0.1) {
                                intake.setInnerBlockOpen(false);
                                intake.setOuterBlockOpen(true);
                            } else if (currentGamepad2.left_trigger > 0.1) {
                                intake.setInnerBlockOpen(true);
                                intake.setOuterBlockOpen(false);
                            }
                            rejecting = false;
                        } else {
                            intake.setSpeed(rejectionSpeed, rejectionSpeed);
                        }
                    }else {
                        // NORMAL MANUAL INTAKE CONTROL.
                        double intakeSpeed;
                        if (currentGamepad2.right_trigger > 0.1) {
                            intake.setInnerBlockOpen(false);
                            intake.setOuterBlockOpen(true);
                            intakeSpeed = Math.min(currentGamepad2.right_trigger * 2, 1.0);
                            intake.setSpeed(intakeSpeed, intakeSpeed);
                        } else if (currentGamepad2.left_trigger > 0.1) {
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(false);
                            intakeSpeed = Math.min(currentGamepad2.left_trigger * 2, 1.0);
                            intake.setSpeed(-intakeSpeed, -intakeSpeed);
                        } else if (currentGamepad2.dpad_right) {
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(true);
                            intake.setSpeed(1, -1);
                        } else if (currentGamepad2.dpad_left) {
                            intake.setInnerBlockOpen(true);
                            intake.setOuterBlockOpen(true);
                            intake.setSpeed(-1, 1);
                        } else {
                            intake.setSpeed(0, 0);
                        }

                    }
                }
            }

            if ((goingHome && doubleDouble && !doubleDoubleTarget) || (currentGamepad2.y && previousGamepad2.y)) {
                goingHome = false;
            }

            if (currentGamepad2.x && previousGamepad2.x){
                intake.setTimedIntake(1, 1, 0.8);
                intake.setInnerBlockOpen(true);
                intake.setOuterBlockOpen(true);
            }
            if (currentGamepad2.left_bumper && scoringSpecs){
                goingHome = true;
            }

            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper &&!scoringSpecs){
                if (yellowReadyToRelease){
                    outtake.sampleReleaseOpen(true);
                    yellowReadyToRelease = false;
                } else {
                    outtake.specDropAtIntakePos(false);
                    transferringYellow = true;
                    yellowTransferTime = currentTime;
                    yellowProcess = 0;
                }
            }
            if (transferringYellow){
                switch(yellowProcess){
                    case 0:
                        if (yellowTransferTime + 0.2 < currentTime){
                            outtake.sampleAtIntakePos(true);

                            yellowProcess++;
                        }
                        break;
                    case 1:
                        if (yellowTransferTime + 0.5 < currentTime){
                            horiSlidesTarget = horiSlides.MIN_POSITION + 0.1;
                            yellowProcess++;
                        }
                        break;
                    case 2:
                        if (yellowTransferTime + 1.0 < currentTime){
                            intake.setTimedIntake(-1, -1, 0.8);
                            outtake.sampleReleaseOpen(false);
                            yellowProcess++;
                        }
                        break;
                    case 3:
                        if (yellowTransferTime + 1.35 < currentTime){
                            outtake.sampleAtIntakePos(false);
                            yellowProcess++;
                        }
                        break;
                    case 4:
                        if (yellowTransferTime + 1.8 < currentTime){
                            outtake.specDropAtIntakePos(true);
                            yellowProcess++;
                            transferringYellow = false;
                            yellowReadyToRelease = true;
                        }
                        break;

                }
            }



            // ------------------- GO HOME (TARGET DELIVERY) SEQUENCE -------------------
            if (goingHome) {
                // Stop the intake.
                intake.setSpeed(0, 0);
                outtake.specDropAtIntakePos(true);
                outtake.specDropOpen(false);

                // If the slide is extended beyond 400, command retraction to 400.
                if (slidePos > horiSlides.MIN_POSITION + 200) {
                    if (scoringSpecs) {
                        horiSlidesTarget = horiSlides.MIN_POSITION + 0.1;
                    } else {
                        horiSlidesTarget = horiSlides.MIN_POSITION + 200;
                    }
                }

                if (slidePos<(horiSlides.MIN_POSITION +700)){
                    intake.setRotation(TRANSFER);
                    //intake.setInnerBlockOpen(true);
                    intake.setOuterBlockOpen(true);
                }
                if (slidePos < (horiSlides.MIN_POSITION + 200) && currentTime - goHomeWaitStart >= 0.4){
                    if (scoringSpecs) {
                        horiSlidesTarget = horiSlides.MIN_POSITION + 0.1;
                    } else {
                        horiSlidesTarget = horiSlides.MIN_POSITION + 200;
                    }
                }
                // When the slide is nearly retracted (below 25), run the intake wheels for 0.8 sec
                // to drop the piece. Do not change the intake rotation; leave it in TRANSFER.

                if (slidePos < (horiSlides.MIN_POSITION + 210) && !scoringSpecs) {
                    goingHome = false;
                    readyToTransferYellow = true;
                }

                if (slidePos < (horiSlides.MIN_POSITION + 25)) {
                    if (scoringSpecs){
                        intake.setTimedIntake(-1, -1, 0.8);
                        sampleInBucket = true;
                        goingHome = false;
                    }
                }
            }

            if (!goingHome && !transferringYellow) {
                double horiInput = -currentGamepad2.right_stick_y;

                if (Math.abs(horiInput) > JOYSTICK_DEADZONE) {
                    if (Math.abs(horiInput) < 0.8){
                        horiInput = horiInput * 0.10;
                    }
                    if (rotationMode.equalsIgnoreCase("INTAKE")) {
                        horiSlidesTarget = currentHoriPos + (horiInput * MAX_INPUT_SCALING);
                        horiSlidesTarget = Math.max(horiSlidesTarget, horiSlides.MIN_POSITION + 80);
                    } else {
                        horiSlidesTarget = currentHoriPos + (horiInput * MAX_INPUT_SCALING);
                    }

                } else {
                    horiSlidesTarget = currentHoriPos;
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
            boolean vertSlideIsRunningToPos = (!(Math.abs(currentGamepad2.left_stick_y) > 0.02)) && (!(Math.abs(currentGamepad1.left_trigger) > 0.02)) && (!(Math.abs(currentGamepad1.right_trigger) > 0.02));
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
                        stickVal = leftStick * 0.5;
                    } else {
                        stickVal = leftStick;
                    }
                }

                if (Math.abs(stickVal) > JOYSTICK_DEADZONE) {
                    vertSlidesTarget = currentVertPos + (-stickVal * MAX_INPUT_SCALING);
                }
                verticalSlides.setPosition(vertSlidesTarget);
            }



            ////////////////////Outake Alliance handling Logic ////////////////
            if (scoringSpecs){
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
                    vertSlidesTarget = vertSlide.MIN_POSITION +150;
                    readyToGoToDelivery = true;
                    specTiming = currentTime;
                } else if ((currentGamepad1.y && !previousGamepad1.y)||(currentGamepad2.dpad_up && !previousGamepad2.dpad_up) || (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) ){
                    if (readyToGoToDelivery){
                        outtake.hookAtIntake(false,false);
                        vertSlidesTarget = vertSlide.MIN_POSITION +435;
                        readyToGoToDelivery = false;
                        readyToDeliver = true;
                        sampleDelivTimer = currentTime;
                    } else if (readyToDeliver && sampleDelivTimer + 1.0 < currentTime){
                        vertSlidesTarget = vertSlide.MIN_POSITION +650;
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
                if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
                    outtake.hookAtIntake(true,false);
                    outtake.clawOpen(true);
                    specimenInClaw = false;
                    vertSlidesTarget = vertSlide.MIN_POSITION +1;
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
                    vertSlidesTarget = vertSlide.MIN_POSITION +1;
                }
                if (specimenRunTimer && specimenDelivTimer +0.7 <currentTime){
                    outtake.hookAtIntake(true,false);
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
            telemetry.addData("Slide Pos", currentHoriPos);
            telemetry.addData("Slide Target", horiSlidesTarget);
            telemetry.addData("Intake Rotation", rotationMode);
            telemetry.addData("Rejecting", rejecting);
            telemetry.addData("GoHome", goingHome);
            telemetry.addData("Loop Time", loopTime);
            telemetry.addData("Highest Loop Time", highestLoopTime);
            telemetry.addData("Lowest Loop Time", lowestLoopTime);
            telemetry.update();

            for (LynxModule hub : allHubs) {
                hub.clearBulkCache();
            }
        }
    }
    // Helper method to get RGB values based on alliance color.
    private int[] getAllianceColorForEnum(TargetState state) {
        if (state == TargetState.RED) {
            return new int[] {255, 0, 0};
        } else if (state == TargetState.BLUE) {
            return new int[] {0, 0, 255};
        }
        return new int[] {255, 255, 255};  // Default/fallback color.
    }


    // Helper method to set LED colors on both gamepads.
    private void setGamepadLedColor(int r, int g, int b) {
        gamepad1.setLedColor(r, g, b, LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(r, g, b, LED_DURATION_CONTINUOUS);
    }

    private TargetState allianceTarget(AllianceColor alliance) {
        return alliance == AllianceColor.RED ? TargetState.RED : TargetState.BLUE;
    }




}

