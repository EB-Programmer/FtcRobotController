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
    private Servo clawServo = null;
    private DigitalChannel intakeSwitch = null;

    //Class variables
    private GamePieceType gamePiece = NONE;
    private ClawArmPresetPosition preset = null;
    private ArrayList<Boolean> buttonValues = new ArrayList<Boolean>();
    private int buttonIncrement = 0;
    private int rCounter = 0;
    private int counter = 0;
    private int hangCounter = 0;
    private int dropCounter = 0;

    //Button switch toggles
    private boolean left_bumper_is_pressed = false;
    private boolean left_stick_button_pressed = false;
    private boolean dpad_left_is_pressed = false;
    private boolean dpad_right_is_pressed = false;
    private boolean right_bumper_is_pressed = false;
    private boolean g1_dpad_down_is_pressed = false;
    private boolean g1_dpad_up_is_pressed = false;
    private boolean g1_right_stick_button_is_pressed = false;

    private double vert_slide_power = 0;
    private double intake_slide_power = 0;

    //Robot component statuses
    private boolean intakeIsActive = false;  //DONE
    private boolean intakeIsRetracting = false;
    private boolean intakeIsLoaded = false;
    private boolean intakeAtExchange = false;
    private boolean intakeSlideAtExchange = false;
    private boolean clawReadyForExchange = false;
    private boolean readyForExchange = false;
    private boolean clawIsOpen = true;
    private boolean specimenHangInProgress = false;
    private boolean dropInProgress = false;
    private boolean toggleSwitchOverride = false;
    private boolean ignoreSwitchOverride = true;
    private double clawValue = CLAW_OPEN;
    private double clawArmValue = CLAW_EXCHANGE_POSITION;
    private double intakeArmValue = INTAKE_ARM_RESTING_POSITION;

    //Drive component statuses
    private double drivePowerPercent = 1;

    @Override
    public void init() {
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

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {

        gamePiece = GamePieceType.NONE;
        drivePowerPercent = 1;
        clawServo.setPosition(CLAW_OPEN);
        clawPivotArmServo.setPosition(CLAW_BASKET_POUNCE_POSITION);
        intakePivotArmServo.setPosition(INTAKE_ARM_RESTING_POSITION);
        preset = ClawArmPresetPosition.WallGrabPreset();
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

        executeDriveActions();
        executeRobotAction();
        buttonIncrement = 0;

        telemetry.addData("Vert Position:",
                slideVertLeftMotor.getCurrentPosition() +
                        ":" + slideVertRightMotor.getCurrentPosition());
        telemetry.addData("Intake Position:", intakeSlideMotor.getCurrentPosition());
        telemetry.addData("Game Piece", gamePiece.name());
        telemetry.addData("Intake Switch", intakeSwitch.getState() && ignoreSwitchOverride);
        telemetry.addData("toggleSwitchOverride", toggleSwitchOverride);
        telemetry.addData("ignoreSwitchOverride", ignoreSwitchOverride);
        /*
        telemetry.addData("Variables",
                "intakeIsActive " + intakeIsActive +
                        ", intakeIsLoaded " + intakeIsLoaded +
                        ", intakeIsRetracting " + intakeIsRetracting +
                        ", intakeAtExchange " + intakeAtExchange +
                        ", clawIsOpen " + clawIsOpen +
                        ", intakeArmValue " + intakeArmValue +
                        ", clawArmValue" + clawArmValue + " " + CLAW_EXCHANGE_POSITION +
                        ", clawReadyForExchange " + clawReadyForExchange +
                        ", readyForExchange " + readyForExchange +
                        ", switch engaged " + intakeSwitch.getState() +
                        ", gamePiece " + gamePiece.name() +
                        "counter " + counter);*/

    }

    private void executeDriveActions() {

        final double DRIVE_POWER_NORMAL = 1;
        final double DRIVE_POWER_SLOW = .4;

        if (gamepad1.a) {
            drivePowerPercent = DRIVE_POWER_NORMAL;
        } else if (gamepad1.b) {
            drivePowerPercent = DRIVE_POWER_SLOW;
        } else if (intakeSlideMotor.getCurrentPosition() > INTAKE_SLIDE_SLOW_DRIVE_POSITION) {
            drivePowerPercent = DRIVE_POWER_SLOW;
        }
        driveRobot(gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.left_trigger,
                gamepad1.right_trigger);

        if (gamepad1.back) {
            slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void checkForOverrides() {
        manuallyOverrideGamePiece();
        manuallyOverrideIntakeSwitch();
    }

    private void setLogicalValues() {

        final int INTAKE_TICK_COUNT = 15;

        intakeIsActive = (intakeArmValue == INTAKE_ARM_ACTIVE_POSITION);
        if (intakeIsActive) {
            intakeIsLoaded = intakeSwitch.getState() && ignoreSwitchOverride && counter > INTAKE_TICK_COUNT;
        } else {
            intakeIsLoaded = intakeSwitch.getState() && ignoreSwitchOverride;
        }
        intakeIsRetracting = (intakeSlideMotor.getPower() < 0 && intakeIsLoaded);
        intakeSlideAtExchange = intakeSlideMotor.getCurrentPosition() <= INTAKE_SLIDE_EXCHANGE_POSITION;
        intakeAtExchange = (intakeArmValue == INTAKE_ARM_EXCHANGE_POSITION);
        clawIsOpen = clawValue == CLAW_OPEN;
        clawReadyForExchange = (clawArmValue == CLAW_EXCHANGE_POSITION
                && (slideVertRightMotor.getCurrentPosition() <= (VERT_PRE_EXCHANGE_HEIGHT + VERT_MARGIN_OF_ERROR)
                && slideVertRightMotor.getCurrentPosition() > VERT_MAKE_EXCHANGE_HEIGHT))
                && clawIsOpen;
        readyForExchange = intakeIsLoaded
                && intakeAtExchange
                && intakeSlideAtExchange
                && clawReadyForExchange;
        dropInProgress = (dropCounter > 0);
        specimenHangInProgress = (clawArmValue == CLAW_CHAMBER_HANG_POSITION) && (hangCounter > 0);
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
        if (g1_dpad_up_is_pressed) {
            toggleSwitchOverride = !toggleSwitchOverride;
        }
        if (toggleSwitchOverride) {
            if (g1_right_stick_button_is_pressed) {
                ignoreSwitchOverride = !ignoreSwitchOverride;
            }
        }
    }

    private void executeRobotAction() {

        readControllerToggleButtonPresses();  //sets whether a button was pushed for buttons related to servos
        readControllerJoystickValues();  //sets the power values for motors
        checkForOverrides();
        setLogicalValues();  //manages the logical flow of the loop
        setPresetClawPositions();  //sets the preset positions for claw arm servo and motor

        if (gamePiece == SAMPLE) {
            if (intakeIsLoaded) {
                ejectSampleFromIntake();
            } else {
                ejectSampleFromIntake();
                setPowerVerticalSlides();
                setClawArmPosition();
                releaseSample();
            }
        } else if (gamePiece == GamePieceType.SPECIMEN) {
            setPowerVerticalSlides();
            setClawArmPosition();
            releaseSpecimen();
        } else {
            if (readyForExchange) {
                if (right_bumper_is_pressed) {
                    preset = ClawArmPresetPosition.MakeExchangePreset();
                }
                setPowerVerticalSlides();
                runIntakeSlide(intake_slide_power);
                runIntakeMotor(getIntakeMotorDirectionFromController());
            } else {
                runIntakeSlide(intake_slide_power);
                runIntakeMotor(getIntakeMotorDirectionFromController());
                setIntakeArmPosition();
                if (intakeIsRetracting) {
                    preset = ClawArmPresetPosition.PreExchangePreset();
                }
                setPowerVerticalSlides();
                setClawArmPosition();
                toggleClawPosition();
            }
        }
    }

    private void toggleClawPosition() {
        if (preset != null) {
            if (preset.getSlideHeight() == VERT_MAKE_EXCHANGE_HEIGHT
                    && slideVertRightMotor.getCurrentPosition() <= VERT_MAKE_EXCHANGE_HEIGHT + VERT_MARGIN_OF_ERROR) {

                setClawPosition(CLAW_CLOSE);
                preset = null;

            } else if (right_bumper_is_pressed) {
                if (clawIsOpen) {
                    setClawPosition(CLAW_CLOSE);
                } else {
                    setClawPosition(CLAW_OPEN);
                }
            }
        } else if (right_bumper_is_pressed) {
            if (clawIsOpen) {
                setClawPosition(CLAW_CLOSE);
            } else {
                setClawPosition(CLAW_OPEN);
            }
        }
    }

    private void setClawArmPosition() {
        if (preset != null) {
            rotateClawArm(preset.getArmPosition());
        } else {
            rotateClawArm(getClawArmPositionFromController());
        }
    }

    private void setPowerVerticalSlides() {
        if (vert_slide_power != 0) {
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runClawSlide(vert_slide_power);
            preset = null;
        } else if (preset != null) {
            if (!slideVertRightMotor.isBusy() && !slideVertLeftMotor.isBusy()) {
                slideVertLeftMotor.setTargetPosition((int) preset.getSlideHeight());
                slideVertRightMotor.setTargetPosition((int) preset.getSlideHeight());
                slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideVertRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideVertLeftMotor.setPower(VERT_FAST_RAISE);
                slideVertRightMotor.setPower(VERT_FAST_RAISE);
            } else if (preset.getName() == "Wall Grab") {
                preset = null;
            }

        } else {
            slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            runClawSlide(0);
        }
    }

    private void releaseSpecimen() {
        final int CLICKS_TO_HANG_SPECIMEN = 200;
        if (right_bumper_is_pressed) {
            setClawPosition(CLAW_OPEN);
        }
        /*
        if (clawArmValue == CLAW_CHAMBER_HANG_POSITION || clawArmValue == CLAW_BACK_CHAMBER_HANG_POSITION) {
            if (right_bumper_is_pressed) {
                hangCounter = 1;
                vert_slide_power = clawArmValue == CLAW_CHAMBER_HANG_POSITION ? VERT_FAST_RAISE : VERT_FAST_DROP;
            } else if (specimenHangInProgress) {
                hangCounter++;
                vert_slide_power = clawArmValue == CLAW_CHAMBER_HANG_POSITION ? VERT_FAST_RAISE : VERT_FAST_DROP;
                if (hangCounter > CLICKS_TO_HANG_SPECIMEN) {
                    setClawPosition(CLAW_OPEN);
                    hangCounter = 0;
                    preset = null;
                }
            }
        } else {
            if (right_bumper_is_pressed) {
                setClawPosition(CLAW_OPEN);
            }
        }*/
    }

    private void releaseSample() {
        if (right_bumper_is_pressed) {
            setClawPosition(CLAW_OPEN);
            preset = null;
        }
    }

    private void ejectSampleFromIntake() {
        if (intakeIsLoaded) {
            runIntakeMotor(EJECT);
        } else {
            if (intakeAtExchange) {
                intakePivotArmServo.setPosition(INTAKE_ARM_RESTING_POSITION);
            }
            runIntakeMotor(IDLE);
        }
    }

    private void readControllerToggleButtonPresses() {
        /*
        Toggle button presses.  These all control servo actions,
        so they should only run once when button is pressed.  Button maintains TRUE
        while it is held down.
         */
        left_stick_button_pressed = isPressed(gamepad2.left_stick_button);
        left_bumper_is_pressed = isPressed(gamepad2.left_bumper);
        dpad_left_is_pressed = isPressed(gamepad2.dpad_left);
        dpad_right_is_pressed = isPressed(gamepad2.dpad_right);
        right_bumper_is_pressed = isPressed(gamepad2.right_bumper);
        g1_dpad_down_is_pressed = isPressed(gamepad1.dpad_down);
        g1_dpad_up_is_pressed = isPressed(gamepad1.dpad_up);
        g1_right_stick_button_is_pressed = isPressed(gamepad1.right_stick_button);
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

    private void setPresetClawPositions() {
        preset = getPresetFromController();
    }

    private void readControllerJoystickValues() {
        vert_slide_power = -gamepad2.right_stick_y;
        intake_slide_power = -gamepad2.left_stick_y;
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

    private IntakeMotorConstants.IntakeDirection getIntakeMotorDirectionFromController() {
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
        positions = new double[]{
                CLAW_WALL_SPECIMEN_POSITION,
                CLAW_BACK_DROP_POSITION,
                CLAW_BACK_POUNCE_POSITION,
                CLAW_BASKET_POUNCE_POSITION,
                CLAW_BASKET_DROP_POSITION,
                CLAW_CHAMBER_HANG_POSITION,
                CLAW_EXCHANGE_POSITION};

        if (dpad_left_is_pressed) {
            for (int i = 0; i < positions.length - 1; i++) {
                if (positions[i] == clawArmValue) {
                    return positions[i + 1];
                }
            }
            return clawArmValue;
        } else if (dpad_right_is_pressed) {
            for (int i = positions.length - 1; i > 0; i--) {
                if (positions[i] == clawArmValue) {
                    return positions[i - 1];
                }
            }
            return clawArmValue;
        }
        return clawArmValue;
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

    private void runIntakeSlide(double power) {
        double pos = intakeSlideMotor.getCurrentPosition();
        double pow = 0;
        if (pos <= INTAKE_SLIDE_MIN_EXTENSION && power < 0) {
            //STOP retracting if slide position reaches MIN_EXTENSION;
            pow = 0;
        } else if (pos >= INTAKE_SLIDE_MAX_EXTENSION && power > 0) {
            //STOP extending if slide position reaches MAX_EXTENSION
            pow = 0;
        } else if (pos <= INTAKE_SLIDE_CLEARANCE_EXTENSION && power < 0
                && intakePivotArmServo.getPosition() == INTAKE_ARM_ACTIVE_POSITION) {
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
        //STOP retracting if position is less than the MIN_HEIGHT
        if (position <= VERT_MIN_HEIGHT && power < 0) {
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
            if (!(intakeSwitch.getState() && ignoreSwitchOverride)) {
                counter = 0;
            }
            int CLICKS_TO_INTAKE_SAMPLE = 20;
            if (counter > CLICKS_TO_INTAKE_SAMPLE) {
                intakeFrameServo.setPower(0);
            } else {
                intakeFrameServo.setPower(INTAKE_MOTOR_POWER);
                counter++;
            }
        } else {
            if (intakeSwitch.getState() && ignoreSwitchOverride) {
                intakeFrameServo.setPower(INTAKE_MOTOR_EJECT_POWER);
            } else {
                intakeFrameServo.setPower(0);
            }
        }
    }

    private void setClawPosition(double position) {
        double pos = clawServo.getPosition();
        if (pos != position) {
            clawServo.setPosition(position);
            clawValue = position;
            if (position == CLAW_OPEN) {
                gamePiece = GamePieceType.NONE;
            } else {
                //specimen can be grabbed only in CLAW_WALL_SPECIMEN_POSITION
                if (clawArmValue == CLAW_WALL_SPECIMEN_POSITION) {
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

    private void setIntakeArmPosition() {

        if (intakeIsLoaded) {
            if (intakeIsActive) {
                intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);
                intakeArmValue = INTAKE_ARM_CLEARANCE_POSITION;
            } else {
                if (left_bumper_is_pressed) {
                    if (!intakeAtExchange) {
                        intakePivotArmServo.setPosition(INTAKE_ARM_EXCHANGE_POSITION);
                        intakeArmValue = INTAKE_ARM_EXCHANGE_POSITION;
                    } else {
                        intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);
                        intakeArmValue = INTAKE_ARM_CLEARANCE_POSITION;
                    }
                }
            }
        } else {
            if (left_stick_button_pressed) {
                if (intakeIsActive || intakeAtExchange) {
                    intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);
                    intakeArmValue = INTAKE_ARM_CLEARANCE_POSITION;
                } else {
                    intakePivotArmServo.setPosition(INTAKE_ARM_ACTIVE_POSITION);
                    intakeArmValue = INTAKE_ARM_ACTIVE_POSITION;
                }
            } else if (left_bumper_is_pressed) {
                if (intakeIsActive || intakeAtExchange) {
                    intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);
                    intakeArmValue = INTAKE_ARM_CLEARANCE_POSITION;
                } else {
                    intakePivotArmServo.setPosition(INTAKE_ARM_EXCHANGE_POSITION);
                    intakeArmValue = INTAKE_ARM_EXCHANGE_POSITION;
                }
            }
        }
    }
}