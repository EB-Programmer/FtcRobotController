package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ClawConstants.*;
import static org.firstinspires.ftc.teamcode.GamePieceType.*;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.*;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.*;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.IntakeDirection.*;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.*;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.ArrayList;

@TeleOp(name = "EbTeleOp", group = "Iterative Opmode")

public class EbTeleOp extends OpMode {

    //Robot components
    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor slideVertRightMotor = null;
    private DcMotor slideVertLeftMotor = null;
    private DcMotor intakeSlideMotor = null;
    private Servo intakePivotArmServo = null;
    private CRServo intakeFrameServo = null;
    private Servo clawPivotArmServo = null;
    private Servo hardStopServo = null;
    private Servo clawServo = null;
    private DigitalChannel intakeSwitch = null;
    private TouchSensor intakeSlideSwitch = null;
    private DigitalChannel vertSlideSwitch = null;

    //Class variables
    private GamePieceType gamePiece = NONE;
    private ClawArmPresetPosition preset = null;
    private ArrayList<Boolean> buttonValues;
    private int buttonIncrement = 0;
    private int releaseCounter = 0;
    private int grabCounter = 0;

    //Button switch toggles
    private boolean left_bumper_is_pressed = false;
    private boolean left_stick_button_pressed = false;
    private boolean dpad_left_is_pressed = false;
    private boolean dpad_right_is_pressed = false;
    private boolean right_bumper_is_pressed = false;
    private boolean g1_dpad_down_is_pressed = false;
    private boolean g1_right_stick_button_is_pressed = false;
    private boolean g1_back_is_pressed = false;
    private boolean g1_a_is_pressed = false;
    private boolean g1_b_is_pressed = false;

    private double vert_slide_power = 0;
    private double intake_slide_power = 0;

    //Robot component statuses
    private boolean intakeIsActive = false;  //DONE
    private boolean loadedIntakeIsRetracting = false;
    private boolean intakeIsLoaded = false;
    private boolean robotReadyForExchange = false;
    private boolean exchangeInProgress = false;
    private boolean specimenHangInProgress = false;
    private boolean overrideIntakeSwitch = false;
    private boolean isResetSlideMode = false;
    private boolean loadedIntakeArmAtExchange = false;
    private boolean loadedIntakeSlideAtExchange = false;
    private boolean clawReadyForExchange = false;


    private double specimenHangFinalPosition = 0;
    private double clawValue = CLAW_OPEN;
    private double clawArmValue = CLAW_ARM_EXCHANGE_POSITION;
    private double intakeArmValue = INTAKE_ARM_RESTING_POSITION;
    private double hardStopValue = HARD_STOP_INACTIVE;

    private String codeSection;
    //Drive component statuses
    final double DRIVE_POWER_NORMAL = 1;
    final double DRIVE_POWER_SLOW = .4;
    final int INTAKE_TICK_COUNT = 10;

    private double drivePowerPercent = DRIVE_POWER_NORMAL;

    @Override
    public void init() {
        buttonValues = new ArrayList<Boolean>();
        //Instantiate all hardware components
        //Front Left Mechanum Motor
        frontLeftMotor = hardwareMap.get(DcMotor.class, "Front_left");
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Front Right Mechanum Motor
        frontRightMotor = hardwareMap.get(DcMotor.class, "Front_right");

        //Back Left Mechanum Motor
        backLeftMotor = hardwareMap.get(DcMotor.class, "Back_left");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Back Right Mechanum Motor
        backRightMotor = hardwareMap.get(DcMotor.class, "Back_right");

        //Left Vertical Slide Motor
        slideVertLeftMotor = hardwareMap.get(DcMotor.class, "Slide_left");
        slideVertLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        slideVertLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right Vertical Slide Motor
        slideVertRightMotor = hardwareMap.get(DcMotor.class, "Slide_right");
        slideVertRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Horizontal Intake Slide Motor
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Claw_slide");
        intakeSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Intake Pivot Arm Servo
        intakePivotArmServo = hardwareMap.get(Servo.class, "Intake_arm_servo");

        //Intake Frame Servo
        intakeFrameServo = hardwareMap.get(CRServo.class, "Intake_servo");
        intakeFrameServo.setDirection(DcMotorSimple.Direction.REVERSE);

        //Claw Pivot Arm Servo
        clawPivotArmServo = hardwareMap.get(Servo.class, "Claw_arm_servo");

        //Claw Servo
        clawServo = hardwareMap.get(Servo.class, "Claw_servo");

        //Intake Roller Limit Switch
        intakeSwitch = hardwareMap.get(DigitalChannel.class, "Intake_switch");

        //Intake Slide Bump Switch
        intakeSlideSwitch = hardwareMap.get(TouchSensor.class, "Intake_slide_switch");

        //Vert Slide Magnetic Switch
        vertSlideSwitch = hardwareMap.get(DigitalChannel.class, "Vert_slide_switch");

        hardStopServo = hardwareMap.get(Servo.class, "Hard_stop_servo");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {

        codeSection = "START";
        gamePiece = GamePieceType.NONE;
        drivePowerPercent = DRIVE_POWER_NORMAL;
        clawServo.setPosition(CLAW_OPEN);
        clawPivotArmServo.setPosition(CLAW_ARM_VERTICAL_POSITION);
        hardStopServo.setPosition(HARD_STOP_INACTIVE);
        clawArmValue = CLAW_ARM_VERTICAL_POSITION;
        intakeArmValue = INTAKE_ARM_RESTING_POSITION;
        clawValue = CLAW_OPEN;
        hardStopValue = HARD_STOP_INACTIVE;
    }

        /*
        Gamepad Control Assignments:
        Gamepad 1:
            Right Stick:
            Right Stick Button:
            Left Stick: Drive (Y position) and Strafe (X position)
            Left Stick Button:
            D-Pad Up:
            D-Pad Right:
            D-Pad Down:     Manually toggle gamepiece type
            D-Pad Left:
            A:              Drive Speed Normal
            B:              Drive Speed Slow
            X:
            Y:
            Right Trigger:  Turn Right
            Left Trigger:   Turn left
            Right Bumper:
            Left Bumper:
            Back (Select):

        Gamepad 2:
            Right Stick:    Vertical Slide
            Right Stick Button:
            Left Stick:     Intake Operations (y position)
            Left Stick Button:  Toggle Active/Clearance on Intake
            D-Pad Up:
            D-Pad Right:    Cycle through Claw Arm Positions Right
            D-Pad Down:     Eject Sample from Intake
            D-Pad Left:     Cycle through Claw Arm Positions Left
            A:              PreExchange (NONE) or Low Back Drop (SAMPLE)
            B:              Wall (NONE) or High Back Drop (SAMPLE or SPECIMEN)
            X:              Low Front Drop (SAMPLE or SPECIMEN) Preset
            Y:              High Front Drop (SAMPLE or SPECIMEN) Preset
            Right Trigger:
            Left Trigger:
            Right Bumper:
            Left Bumper:    Toggle Exchange/Clearance on Intake
            Back (Select):
         */

    @Override
    public void loop() {

        readControllerToggleButtonPresses();
        checkForResetSlideMode();
        if (isResetSlideMode) {
            resetSlidesToZero();
            showResetTelemetry();
        } else {
            executeDriveActions();
            executeRobotAction();
            showTelemetry();
        }
    }

    private void readControllerToggleButtonPresses() {
        /*
        Toggle button presses.  These all control servo actions,
        so they should only run once when button is pressed.  Button maintains TRUE
        while it is held down.
         */
        buttonIncrement = 0;
        left_stick_button_pressed = isPressed(gamepad2.left_stick_button);
        left_bumper_is_pressed = isPressed(gamepad2.left_bumper);
        dpad_left_is_pressed = isPressed(gamepad2.dpad_left);
        dpad_right_is_pressed = isPressed(gamepad2.dpad_right);
        right_bumper_is_pressed = isPressed(gamepad2.right_bumper);
        g1_dpad_down_is_pressed = isPressed(gamepad1.dpad_down);
        g1_right_stick_button_is_pressed = isPressed(gamepad1.right_stick_button);
        g1_back_is_pressed = isPressed(gamepad1.back);
        g1_a_is_pressed = isPressed(gamepad1.a);
        g1_b_is_pressed = isPressed(gamepad1.b);
    }

    private void checkForResetSlideMode() {
        if (g1_back_is_pressed) {
            isResetSlideMode = !isResetSlideMode;
        }
    }

    private void executeDriveActions() {

        if (gamepad1.a) {
            drivePowerPercent = DRIVE_POWER_NORMAL;
        } else if (gamepad1.b) {
            drivePowerPercent = DRIVE_POWER_SLOW;
        }
        driveRobot(gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.left_trigger,
                gamepad1.right_trigger);
    }

    private void driveRobot(double drivePower, double strafePower, double turnLeftPower, double turnRightPower) {

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //drive controls forward power based on Left Drive Stick's Y position
        double drive = -drivePower * drivePowerPercent;

        //turn controls left/right power based on Right and Left Triggers
        double turn;
        if (turnLeftPower > 0.0) {
            turn = -turnLeftPower * drivePowerPercent;
        } else if (turnRightPower > 0.0) {
            turn = turnRightPower * drivePowerPercent;
        } else
            turn = 0;

        //strafe controls side movement power based on Left Drive Stick's X position
        double strafe = strafePower * drivePowerPercent;

        //calculate motor power
        frontLeftPower = Range.clip((drive + turn + strafe), -1.0, 1.0);
        frontRightPower = Range.clip((drive - turn - strafe), -1.0, 1.0);
        backLeftPower = Range.clip((drive + turn - strafe), -1.0, 1.0);
        backRightPower = Range.clip((drive - turn + strafe), -1.0, 1.0);

        // Send calculated power to motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

    }

    private void executeRobotAction() {

        checkSlideSwitches();

        readControllerJoystickValues();  //sets the power values for motors
        checkForOverrides();
        setLogicalValues();  //manages the logical flow of the loop
        setPresetClawPositions();  //sets the preset positions for claw arm servo and motor

        if (gamePiece == SAMPLE) {
            setHardStop(HARD_STOP_INACTIVE);
            if (intakeIsLoaded) {
                codeSection = "SAMPLE - INTAKE LOADED";
                unloadSampleFromIntake();
                exchangeInProgress = false;
            } else {
                codeSection = "SAMPLE IN CLAW";
                placeIntakeArmInRestingPosition();
                runVerticalSlides();
                rotateClawArm(getClawArmPosition());
                if (right_bumper_is_pressed) {
                    releaseSample();
                }
            }
        } else if (gamePiece == GamePieceType.SPECIMEN) {
            codeSection = "SPECIMEN IN CLAW";
            setHardStop(HARD_STOP_ACTIVE);
            if (clawArmValue == CLAW_ARM_CHAMBER_HANG_POSITION || clawArmValue == CLAW_ARM_BACK_CHAMBER_HANG_POSITION) {
                if (right_bumper_is_pressed) {
                    runSpecimenRelease();
                } else if (specimenHangInProgress) {
                    codeSection = "SPECIMEN HANG IN PROGRESS";
                    runSpecimenRelease();
                }
            } else {
                specimenHangInProgress = false;
                rotateClawArm(getClawArmPosition());
            }
            runVerticalSlides();
        } else {
            setHardStop(HARD_STOP_INACTIVE);
            if (exchangeInProgress) {

                codeSection = "EXCHANGE IN PROGRESS";
                closeClawToGrabSample();

            } else {
                if (robotReadyForExchange) {

                    codeSection = "ROBOT READY FOR EXCHANGE";
                    if (right_bumper_is_pressed) {
                        preset = ClawArmPresetPosition.MakeExchangePreset();
                        exchangeInProgress = true;
                    }
                } else {
                    codeSection = "DEFAULT - loadedIntakeArmAtExchange "
                            + loadedIntakeArmAtExchange
                            + ", loadedIntakeSlideAtExchange " + loadedIntakeSlideAtExchange
                            + ", clawReadyForExchange " + clawReadyForExchange;

                    if (loadedIntakeIsRetracting) {

                        preset = ClawArmPresetPosition.PreExchangePreset();


                    }

                    if (right_bumper_is_pressed) {
                        if (clawValue == CLAW_CLOSE) {
                            openEmptyClaw();
                        } else {
                            closeClawToGrabSpecimen();
                        }


                    }
                }
                runIntakeSlide(intake_slide_power);
                runIntakeMotor(getIntakeMotorDirection());
                rotateIntakeArm(getIntakeArmPosition());
                runVerticalSlides();
                rotateClawArm(getClawArmPosition());
            }
        }

    }

    private void closeClawToGrabSpecimen() {
        if (clawArmValue == CLAW_ARM_WALL_SPECIMEN_POSITION) {
            setClawPosition(CLAW_CLOSE);
            preset = ClawArmPresetPosition.HighChamberPreset();
        }
    }

    private void checkSlideSwitches() {
        if (intakeSlideSwitch.isPressed()) {
            intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (!vertSlideSwitch.getState()) {
            slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

    }

    private void closeClawToGrabSample() {
        if (clawArmValue == CLAW_ARM_EXCHANGE_POSITION) {
            if (slideVertRightMotor.getCurrentPosition() <= VERT_MAKE_EXCHANGE_HEIGHT + VERT_MARGIN_OF_ERROR) {
                setClawPosition(CLAW_CLOSE);
                preset = null;
            }

        }
    }

    private void setHardStop(double position) {
        if (hardStopValue != position) {
            hardStopServo.setPosition(position);
            hardStopValue = position;
        }
    }

    private void openEmptyClaw() {
        if (clawValue == CLAW_CLOSE) {
            setClawPosition(CLAW_OPEN);
        }
    }

    private void showTelemetry() {
/*
        telemetry.addData("Vert Position:",

                slideVertLeftMotor.getCurrentPosition() +
                        ":" + slideVertRightMotor.getCurrentPosition());*/
        telemetry.addData("Intake Position:", intakeSlideMotor.getCurrentPosition());
        telemetry.addData("Game Piece", gamePiece.name());
        telemetry.addData("Intake Switch", intakeSwitch.getState());
        telemetry.addData("overrideIntakeSwitch", overrideIntakeSwitch);
        telemetry.addData("Code Section", codeSection);
        telemetry.addData("Preset", preset == null ? "NONE" : preset.getName());
        telemetry.addData("Slide Mode", slideVertRightMotor.getMode());
        telemetry.addData("Slide Switch", intakeSlideSwitch.isPressed());
        telemetry.addData("Vert Switch", vertSlideSwitch.getState());

    }

    private void resetSlidesToZero() {
        double vPow = 0;
        double inPow = 0;

        if (g1_a_is_pressed) {
            rotateIntakeArm(INTAKE_ARM_RESTING_POSITION);
        }

        if (g1_b_is_pressed) {
            rotateClawArm(CLAW_ARM_VERTICAL_POSITION);
        }

        if (gamepad1.dpad_down) {
            vPow = VERT_SLOW_DROP;
        }

        if (gamepad1.dpad_left) {
            inPow = INTAKE_SLIDE_SLOW_RETRACT_POWER;
        }

        slideVertLeftMotor.setPower(vPow);
        slideVertRightMotor.setPower(vPow);
        intakeSlideMotor.setPower(inPow);

        if (g1_right_stick_button_is_pressed) {
            slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isResetSlideMode = false;
        }

    }

    private void showResetTelemetry() {
        telemetry.addData("RESET MOTORS",
                "VL:" + slideVertLeftMotor.getCurrentPosition()
                        + ", VR:" + slideVertRightMotor.getCurrentPosition()
                        + ", INTAKE:" + intakeSlideMotor.getCurrentPosition());
    }

    private void readControllerJoystickValues() {
        vert_slide_power = -gamepad2.right_stick_y;
        intake_slide_power = -gamepad2.left_stick_y;
    }

    private void checkForOverrides() {
        if (g1_dpad_down_is_pressed) {
            manuallyOverrideGamePiece();
        }
        if (g1_right_stick_button_is_pressed) {
            manuallyOverrideIntakeSwitch();
        }
    }

    private void setLogicalValues() {

        intakeIsActive = (intakeArmValue == INTAKE_ARM_ACTIVE_POSITION);
        if (intakeIsActive) {
            intakeIsLoaded = intakeSwitch.getState() && !overrideIntakeSwitch && grabCounter > INTAKE_TICK_COUNT;
        } else {
            intakeIsLoaded = intakeSwitch.getState() && !overrideIntakeSwitch;
        }
        loadedIntakeIsRetracting = (intakeIsLoaded && intakeSlideMotor.getPower() < 0);
        loadedIntakeSlideAtExchange = (intakeIsLoaded && intakeSlideMotor.getCurrentPosition() <= INTAKE_SLIDE_EXCHANGE_POSITION);
        loadedIntakeArmAtExchange = (intakeIsLoaded && intakeArmValue == INTAKE_ARM_EXCHANGE_POSITION);
        clawReadyForExchange = (clawArmValue == CLAW_ARM_EXCHANGE_POSITION
                && (slideVertRightMotor.getCurrentPosition() <= (VERT_PRE_EXCHANGE_HEIGHT + VERT_MARGIN_OF_ERROR)
                && slideVertRightMotor.getCurrentPosition() > VERT_PRE_EXCHANGE_HEIGHT - VERT_MARGIN_OF_ERROR))
                && clawValue == CLAW_OPEN;
        robotReadyForExchange = intakeIsLoaded
                && loadedIntakeArmAtExchange
                && loadedIntakeSlideAtExchange
                && clawReadyForExchange;
    }

    private void setPresetClawPositions() {
        preset = getPresetFromController();
    }

    private void manuallyOverrideGamePiece() {
        if (g1_dpad_down_is_pressed) {
            if (gamePiece == SAMPLE) {
                gamePiece = SPECIMEN;
            } else if (gamePiece == SPECIMEN) {
                gamePiece = NONE;
            } else {
                gamePiece = SAMPLE;
            }
        }
    }

    private void manuallyOverrideIntakeSwitch() {
        if (g1_right_stick_button_is_pressed) {
            overrideIntakeSwitch = !overrideIntakeSwitch;
        }
    }

    private double getClawArmPosition() {
        if (preset != null) {
            return preset.getArmPosition();
        } else {
            return getClawArmPositionFromController();
        }
    }

    private void runVerticalSlides() {
        if (vert_slide_power != 0) {
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runClawSlide(vert_slide_power);
            preset = null;
        } else if (preset != null) {
            slideVertLeftMotor.setTargetPosition((int) preset.getSlideHeight());
            slideVertRightMotor.setTargetPosition((int) preset.getSlideHeight());
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideVertLeftMotor.setPower(VERT_FAST_RAISE);
            slideVertRightMotor.setPower(VERT_FAST_RAISE);
        } else {
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runClawSlide(0);
        }
    }

    private void runSpecimenRelease() {
        if (!specimenHangInProgress) {
            specimenHangInProgress = true;
            if (clawArmValue == CLAW_ARM_CHAMBER_HANG_POSITION
                    || clawArmValue == CLAW_ARM_CHAMBER_HANG_POSITION + CLAW_ARM_LOW_CHAMBER_OFFSET) {
                specimenHangFinalPosition = VERT_HIGH_CHAMBER_MAX_HEIGHT;
                vert_slide_power = VERT_FAST_RAISE;
            } else {
                specimenHangFinalPosition = slideVertRightMotor.getCurrentPosition()
                        - VERT_SPECIMEN_HANG_OFFSET_DISTANCE;
                vert_slide_power = VERT_FAST_DROP;
            }
        } else {
            if (clawArmValue == CLAW_ARM_CHAMBER_HANG_POSITION
                    || clawArmValue == CLAW_ARM_CHAMBER_HANG_POSITION + CLAW_ARM_LOW_CHAMBER_OFFSET) {
                if (slideVertRightMotor.getCurrentPosition() < specimenHangFinalPosition) {
                    vert_slide_power = VERT_FAST_RAISE;
                } else {
                    vert_slide_power = 0;
                    setClawPosition(CLAW_OPEN);
                    specimenHangInProgress = false;
                }
            } else {
                if (slideVertRightMotor.getCurrentPosition() > specimenHangFinalPosition) {
                    vert_slide_power = VERT_FAST_DROP;
                } else {
                    vert_slide_power = 0;
                    setClawPosition(CLAW_OPEN);
                    specimenHangInProgress = false;
                }
            }
        }
    }

    private void releaseSample() {
        if (right_bumper_is_pressed) {
            setClawPosition(CLAW_OPEN);
            preset = null;
        }
    }

    private void unloadSampleFromIntake() {
        final int TICKS_TO_RELEASE = 15;
        if (releaseCounter == 0) {
            releaseCounter++;
            runIntakeMotor(EJECT);
        } else {
            releaseCounter++;
            if (releaseCounter > TICKS_TO_RELEASE) {
                runIntakeMotor(IDLE);
                placeIntakeArmInRestingPosition();
            }
        }
    }

    private void placeIntakeArmInRestingPosition() {
        if (intakeArmValue != INTAKE_ARM_RESTING_POSITION) {
            rotateIntakeArm(INTAKE_ARM_RESTING_POSITION);
            runIntakeMotor(IDLE);
            releaseCounter = 0;
        }
    }

    private boolean isPressed(boolean button) {
        boolean res = false;
        if (buttonValues.size() == buttonIncrement) {
            buttonValues.add(false);
        }
        if (button != buttonValues.get(buttonIncrement) && button) {
            res = true;
        }
        buttonValues.set(buttonIncrement, button);
        buttonIncrement++;
        return res;
    }

    private ClawArmPresetPosition getPresetFromController() {
        if (gamepad2.y) {
            if (gamePiece == SAMPLE) {
                return ClawArmPresetPosition.HighBasketPreset();
            } else if (gamePiece == GamePieceType.SPECIMEN) {
                return ClawArmPresetPosition.HighChamberPreset();
            } else {
                return null;
            }
        } else if (gamepad2.x) {
            if (gamePiece == SAMPLE) {
                return ClawArmPresetPosition.HighBasketBackDropPreset();
            } else if (gamePiece == GamePieceType.SPECIMEN) {
                return ClawArmPresetPosition.LowChamberPreset();
            } else {
                return null;
            }
        } else if (gamepad2.a) {
            if (gamePiece == GamePieceType.NONE) {
                return ClawArmPresetPosition.PreExchangePreset();
            } else if (gamePiece == SAMPLE) {
                return ClawArmPresetPosition.LowBasketBackDropPreset();
            } else {
                return null;
            }
        } else if (gamepad2.b) {
            if (gamePiece == GamePieceType.NONE) {
                return ClawArmPresetPosition.WallGrabPreset();
            } else if (gamePiece == SAMPLE) {
                return ClawArmPresetPosition.LowBasketPreset();
            } else if (gamePiece == SPECIMEN) {
                return ClawArmPresetPosition.HighChamberBackHangPreset();
            } else {
                return null;
            }
        }
        return preset;
    }

    private IntakeMotorConstants.IntakeDirection getIntakeMotorDirection() {
        if (intakeIsActive) {
            return INTAKE;
        } else if (gamepad2.dpad_down) {
            return EJECT;
        } else {
            return IDLE;
        }
    }

    private double getClawArmPositionFromController() {
        double[] positions;
        double pos = clawArmValue;
        positions = new double[]{
                CLAW_ARM_WALL_SPECIMEN_POSITION,
                CLAW_ARM_BACK_CHAMBER_HANG_POSITION,
                CLAW_ARM_BACK_DROP_POSITION,
//                CLAW_BACK_POUNCE_POSITION,
//                CLAW_BASKET_POUNCE_POSITION,
                CLAW_ARM_BASKET_DROP_POSITION,
                CLAW_ARM_CHAMBER_HANG_POSITION,
                CLAW_ARM_EXCHANGE_POSITION};

        if (dpad_left_is_pressed) {
            for (int i = 0; i < positions.length - 1; i++) {
                if (positions[i] == clawArmValue) {
                    pos = positions[i + 1];
                }
            }
        } else if (dpad_right_is_pressed) {
            for (int i = positions.length - 1; i > 0; i--) {
                if (positions[i] == clawArmValue) {
                    pos = positions[i - 1];
                }
            }
        }
        return pos;
    }

    private void runIntakeSlide(double power) {
        double pos = intakeSlideMotor.getCurrentPosition();
        double pow = 0;
        if (intakeSlideSwitch.isPressed() && power < 0) {
            pow = 0;
            intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
/*        } else if (pos <= INTAKE_SLIDE_MIN_EXTENSION && power < 0) {
            //STOP retracting if slide position reaches MIN_EXTENSION;
            pow = 0;*/
        } else if (pos >= INTAKE_SLIDE_MAX_EXTENSION && power > 0) {
            //STOP extending if slide position reaches MAX_EXTENSION
            pow = 0;
        } else if (pos <= INTAKE_SLIDE_CLEARANCE_EXTENSION && power < 0
                && intakeArmValue == INTAKE_ARM_ACTIVE_POSITION) {
            //STOP retracting slide if position reaches CLEARANCE and INTAKE is ACTIVE
            pow = 0;
        } else if (power < 0 && pos < INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD) {
            pow = INTAKE_SLIDE_SLOW_RETRACT_POWER;
            //SLOW the retracting speed if intake is near the MIN point
        } else {
            pow = power;
        }
        intakeSlideMotor.setPower(pow);
    }

    private void runClawSlide(double power) {

        double rPow = getSlidePower(slideVertRightMotor.getCurrentPosition(), power);
        double lPow = getSlidePower(slideVertLeftMotor.getCurrentPosition(), power);

        slideVertRightMotor.setPower(rPow);
        slideVertLeftMotor.setPower(lPow);
    }

    private double getSlidePower(double position, double power) {

        double pow = 0;
        if (!vertSlideSwitch.getState() && power < 0) {
            pow = 0;
            slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        } else if (position <= VERT_MIN_HEIGHT && power < 0) {
            //STOP retracting if position is less than the MIN_HEIGHT
            pow = 0;
            //STOP extending if position is greater than MAX_HEIGHT
        } else if (position >= VERT_MAX_HEIGHT && power > 0) {
            pow = 0;
            //SLOW retracting if position is less than SLOW_THRESHOLD
        } else if (position < VERT_SLOW_MOTOR_THRESHOLD && power < 0) {
            pow = VERT_SLOW_DROP;
        } else {
            pow = power;
        }
        return pow;
    }

    private void runIntakeMotor(IntakeMotorConstants.IntakeDirection intakeDirection) {

        if (intakeDirection == IDLE) {
            intakeFrameServo.setPower(0);
            return;
        }

        if (intakeDirection == INTAKE) {
            if (!(intakeSwitch.getState() && !overrideIntakeSwitch)) {
                grabCounter = 0;
            }
            if (grabCounter > INTAKE_TICK_COUNT) {
                intakeFrameServo.setPower(0);
            } else {
                intakeFrameServo.setPower(INTAKE_MOTOR_POWER);
                grabCounter++;
            }
        } else {
            if (intakeSwitch.getState() && !overrideIntakeSwitch) {
                intakeFrameServo.setPower(INTAKE_MOTOR_EJECT_POWER);
            } else {
                intakeFrameServo.setPower(0);
            }
        }
        if (intakeArmValue==INTAKE_ARM_RESTING_POSITION) {
            intakeFrameServo.setPower(0);
        }
    }

    private void setClawPosition(double position) {
        double pos = clawValue;
        if (pos != position) {
            clawServo.setPosition(position);
            clawValue = position;
            if (position == CLAW_OPEN) {
                gamePiece = GamePieceType.NONE;
            } else {
                //specimen can be grabbed only in CLAW_WALL_SPECIMEN_POSITION
                if (clawArmValue == CLAW_ARM_WALL_SPECIMEN_POSITION) {
                    gamePiece = GamePieceType.SPECIMEN;
                } else {
                    gamePiece = SAMPLE;
                }
            }
        }
    }

    private void rotateClawArm(double position) {
        if (position != clawArmValue) {
            clawPivotArmServo.setPosition(position);
            clawArmValue = position;
        }
    }

    private void rotateIntakeArm(double position) {
        if (position != intakeArmValue) {
            intakePivotArmServo.setPosition(position);
            intakeArmValue = position;
        }
    }

    private double getIntakeArmPosition() {

        double startPos = intakeArmValue;
        double endPos = intakeArmValue;

        if (intakeIsLoaded) {
            if (startPos == INTAKE_ARM_ACTIVE_POSITION) {
                endPos = INTAKE_ARM_CLEARANCE_POSITION;
            } else {
                if (left_bumper_is_pressed || left_stick_button_pressed) {
                    if (startPos != INTAKE_ARM_EXCHANGE_POSITION) {
                        endPos = INTAKE_ARM_EXCHANGE_POSITION;
                    } else {
                        endPos = INTAKE_ARM_CLEARANCE_POSITION;
                    }
                }
            }
        } else {
            if (left_stick_button_pressed) {
                if (startPos == INTAKE_ARM_ACTIVE_POSITION
                        || startPos == INTAKE_ARM_EXCHANGE_POSITION
                        || startPos == INTAKE_ARM_RESTING_POSITION) {
                    endPos = INTAKE_ARM_CLEARANCE_POSITION;
                } else {
                    endPos = INTAKE_ARM_ACTIVE_POSITION;
                }
            } else if (left_bumper_is_pressed) {
                if (startPos == INTAKE_ARM_ACTIVE_POSITION
                        || startPos == INTAKE_ARM_EXCHANGE_POSITION) {
                    endPos = INTAKE_ARM_CLEARANCE_POSITION;
                } else {
                    endPos = INTAKE_ARM_EXCHANGE_POSITION;
                }
            }
        }
        return endPos;
    }
}