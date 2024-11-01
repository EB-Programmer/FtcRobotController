package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_ARM_CHANGE_TIME;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_BASKET_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CHANGE_TIME;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CHAMBER_HANG_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_WALL_SPECIMEN_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_ACTIVE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_CLEARANCE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_EXCHANGE_TIME;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_CLEARANCE_TIME;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_REVOLUTION_TIME;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.IntakeDirection.*;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_EXCHANGE_MARGIN_OF_ERROR;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_FAST_EXTEND_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_FAST_RETRACT_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_RECALIBRATE_SPEED;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_ADJUST_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_RETRACT_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_TRAWL_SPEED;
import static org.firstinspires.ftc.teamcode.TiltServoConstants.TILT_DOWN;
import static org.firstinspires.ftc.teamcode.TiltServoConstants.TILT_UP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_FAST_RAISE;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_HIGH_BASKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_HIGH_CHAMBER_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_LOW_BASKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_LOW_CHAMBER_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MAKE_EXCHANGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_PRE_EXCHANGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_FAST_DROP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MARGIN_OF_ERROR;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLIDE_RECALIBRATE_SPEED;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_DROP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_MOTOR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_RAISE;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SPECIMEN_DROP_DISTANCE;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_WALL_HEIGHT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
    private Servo climbLiftServo = null;
    private DigitalChannel intakeSwitch = null;
    //    private DigitalChannel clawSwitch = null;
    //    private ColorSensor colorSensor = null;

    //Timers for servo actions
    private ElapsedTime intakeMotorTimer;
    private ElapsedTime intakeArmTimer;
    private ElapsedTime clawArmTimer;
    private ElapsedTime clawMotorTimer;

    //Class variables
    private RobotAction robotAction;
    private GamePieceType gamePiece = GamePieceType.NONE;

    private double drivePowerPercent;
    private double specimenHangHeight;

    //action indicators
    private boolean isRetractedIntake = false;
    private boolean isActiveIntake = false;
    private boolean isReadyForExchangeIntake = false;
    private boolean isReadyForExchangeVert = false;
    private boolean isLoadedClaw = false;
    private boolean isReadyForReleaseSample = false;
    private boolean isReadyToHangSpecimen = false;
    private boolean isRecalibrationMode = false;
    private boolean isReadyToGrabSpecimen = false;
    private boolean isManualMode = false;
    private boolean isEndGame = false;
    private boolean isInitiatedClimb = false;
    private boolean isPositionedClawArm = true;
    private boolean isInUseIntake = false;
    private boolean isInitiatedSpecimenGrab = false;
    private boolean isRunningIntake = false;
    private boolean isSetClawGrip = true;

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
        slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right Vertical Slide Motor
        slideVertRightMotor = hardwareMap.get(DcMotor.class, "Slide_right");
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Horizontal Intake Slide Motor
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Claw_slide");
        intakeSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Intake Pivot Arm Servo
        intakePivotArmServo = hardwareMap.get(Servo.class, "Intake_arm_servo");
        intakePivotArmServo.setPosition(.6);

        //Intake Frame Servo
        intakeFrameServo = hardwareMap.get(CRServo.class, "Intake_servo");

        //Claw Pivot Arm Servo
        clawPivotArmServo = hardwareMap.get(Servo.class, "Claw_arm_servo");

        //Claw Servo
        clawServo = hardwareMap.get(Servo.class, "Claw_servo");

        //Climb Lift Servo
        climbLiftServo = hardwareMap.get(Servo.class, "Climb_lift_servo");

        //Intake Color Sensor
        /*
        Not needed in teleop
         */
//        colorSensor = hardwareMap.get(ColorSensor.class, "Color_sensor");

        //Intake Roller Limit Switch
        intakeSwitch = hardwareMap.get(DigitalChannel.class, "Intake_switch");

        //Claw Bump Switch
        /*
        Not going to have this on the robot at this time, so need to change code in
        appropriate sections.
         */
//        clawSwitch = hardwareMap.get(DigitalChannel.class, "Claw_switch");

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void start() {
        robotAction = RobotAction.IDLE;
        gamePiece = GamePieceType.NONE;

        //Set timers for Servo components
        intakeArmTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeMotorTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        clawArmTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        clawMotorTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void loop() {
    /*
        Gamepad Control Assignments:
        Gamepad 1:
            Right Stick:
            Right Stick Button: STOP_AND_RESET_ENCODER
            Left Stick:     Drive (Y position) and Strafe (X position)
            Left Stick Button:
            D-Pad Up:       Cancel recalibration
            D-Pad Right:
            D-Pad Down:     Manual Slide for vertical recalibration
            D-Pad Left:     Manual Slide for intake recalibration
            A:              Drive Speed Normal
            B:              Drive Speed Slow
            X:              (end game mode) Tilt Up
            Y:              (end game mode) Tilt Down
            Right Trigger:  Turn Right
            Left Trigger:   Turn left
            Right Bumper:
            Left Bumper:
            Back (Select):  Toggle End Game

        Gamepad 2:
            Right Stick:    Vertical Slide (manual mode)
            Right Stick Button:  Wall preset
            Left Stick:     Intake Operations (y position)
            Left Stick Button:  Run Intake
            D-Pad Up:
            D-Pad Right:
            D-Pad Down:     Eject Sample from Intake
            D-Pad Left:     Retract Intake Slide
            A:              Lower Basket Preset Operation
            B:              Lower Chamber Preset Operation
            X:              Upper Basket Preset Operation
            Y:              Upper Chamber Preset Operation
            Right Trigger:  Manual Claw Arm Right
            Left Trigger:   Manual Claw Arm Left
            Right Bumper:
            Left Bumper:    Claw operations
            Back (Select):  Toggle Manual Mode
         */

        executeDriveAction();
        setRobotAction();
        executeRobotAction();
    }

    private void executeDriveAction() {

        final double DRIVE_POWER_NORMAL = 1;
        final double DRIVE_POWER_SLOW = .4;

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

    private void setRobotAction() {

        /*
        Order of Operation for setting robot actions:
        1. End Game mode
        2. Manual mode - allows driver 2 to operate all slides manually
        3. Recalibration mode - allows driver 1 to reset the slides to 0
        4. Release Sample - sample loaded and ready to release
        5. Hang specimen - specimen loaded and ready to hang
        6. Align vertical slides for baskets/chambers - sample or specimen loaded
        7. Exchange sample - sample in intake and claw and intake at exchange position
        8. Position claw and intake for exchange - sample in intake
        9. Use intake - intake slide activated by driver 2
        10. Align for wall grab -
        11. Idle - no action
         */
        if (isEndGame) {
            robotAction = RobotAction.RUN_END_GAME_MODE;
        } else if (isManualMode) {
            robotAction = RobotAction.RUN_MANUAL_MODE;
        } else if (isRecalibrationMode) {
            robotAction = RobotAction.RUN_SLIDE_RECALIBRATION;
        } else if (isReadyForReleaseSample) {
            robotAction = RobotAction.RUN_RELEASE_SAMPLE;
        } else if (isReadyToHangSpecimen) {
            robotAction = RobotAction.RUN_HANG_SPECIMEN;
        } else if (isLoadedClaw) {
            robotAction = RobotAction.ALIGN_SLIDE_FOR_POINTS;
        } else if (isReadyForExchangeIntake && isReadyForExchangeVert) {
            robotAction = RobotAction.EXCHANGE_SAMPLE;
        } else if (isReadyForExchangeIntake) {
            robotAction = RobotAction.POSITION_CLAW_FOR_EXCHANGE;
        } else if (isInUseIntake) {
            robotAction = RobotAction.USE_INTAKE;
        } else if (isReadyToGrabSpecimen) {
            robotAction = RobotAction.GET_SPECIMEN_FROM_WALL;
        } else if (isInitiatedSpecimenGrab) {
            robotAction = RobotAction.ALIGN_SLIDE_FOR_WALL;
        } else {
            robotAction = RobotAction.IDLE;
        }
    }

    private void executeRobotAction() {

        double vertSlidePower = 0;
        double intakeSlidePower = 0;
        IntakeMotorConstants.IntakeDirection intakeDirecion = isActiveIntake ? INTAKE : IDLE;

        switch (robotAction) {
            case IDLE:
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                } else if (gamepad1.dpad_up) {
                    isRecalibrationMode = true;
                } else if (gamepad2.left_stick_y != 0) {
                    intakeSlidePower = gamepad2.left_stick_y;
                    isInUseIntake = true;
                } else if (gamepad2.right_stick_button) {
                    vertSlidePower = getVerticalSlidePower(VERT_WALL_HEIGHT);
                    isReadyToGrabSpecimen =
                            hasSetVertPosition(VERT_WALL_HEIGHT)
                                    && hasSetClawArmPosition(CLAW_WALL_SPECIMEN_POSITION)
                                    && hasOpenedClaw();
                    isInitiatedSpecimenGrab = !isReadyToGrabSpecimen;
                }
                if (!isEndGame && !isManualMode && !isRecalibrationMode
                        && !isInitiatedSpecimenGrab && !isReadyToGrabSpecimen) {
                    vertSlidePower = getVerticalSlidePower(VERT_PRE_EXCHANGE_HEIGHT);
                    isReadyForExchangeVert = hasSetClawArmPosition(VERT_PRE_EXCHANGE_HEIGHT)
                            && hasSetVertPosition(VERT_PRE_EXCHANGE_HEIGHT);
                }
                if (!isInUseIntake && !isManualMode && !isRecalibrationMode) {
                    intakeSlidePower = getIntakeSlidePower(INTAKE_SLIDE_EXCHANGE_POSITION);
                }
                break;
            case ALIGN_SLIDE_FOR_WALL:
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                    isInitiatedSpecimenGrab = false;
                } else if (gamepad2.left_stick_y != 0) {
                    intakeSlidePower = gamepad2.left_stick_y;
                    isInUseIntake = true;
                    isInitiatedSpecimenGrab = false;
                } else {
                    vertSlidePower = getVerticalSlidePower(VERT_WALL_HEIGHT);
                    isReadyToGrabSpecimen =
                            hasSetVertPosition(VERT_WALL_HEIGHT)
                                    && hasSetClawArmPosition(CLAW_WALL_SPECIMEN_POSITION)
                                    && hasOpenedClaw();
                    isInitiatedSpecimenGrab = !isReadyToGrabSpecimen;
                }
                break;
            case GET_SPECIMEN_FROM_WALL:
                if (clawMotorTimer.time() > CLAW_CHANGE_TIME) {
                    if (gamepad1.back) {
                        isEndGame = true;
                    } else if (gamepad2.back) {
                        isManualMode = true;
                        isReadyToGrabSpecimen = false;
                    } else if (gamepad2.left_stick_y != 0) {
                        intakeSlidePower = gamepad2.left_stick_y;
                        isReadyToGrabSpecimen = false;
                        isInUseIntake = true;
                    } else if (gamepad2.left_bumper) {
                        isLoadedClaw = hasClosedClaw();
                        isReadyToGrabSpecimen = !isLoadedClaw;
                    } else {
                        isLoadedClaw = (clawServo.getPosition() == CLAW_CLOSE);
                        if (isLoadedClaw) {
                            gamePiece = GamePieceType.SPECIMEN;
                            isReadyToGrabSpecimen = false;
                        }
                    }
                }
                break;
            case USE_INTAKE:
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                    isInUseIntake = false;
                } else {
                    if (intakeSwitch.getState()) {  //SAMPLE in intake
                        if (isRunningIntake) {
                            intakeDirecion = INTAKE;
                        } else if (!isRetractedIntake) {
                            intakeSlidePower = getIntakeSlidePower(INTAKE_SLIDE_EXCHANGE_POSITION);
                        } else {
                            isReadyForExchangeIntake = hasSetIntakeArmPosition(INTAKE_ARM_EXCHANGE_POSITION, INTAKE_ARM_EXCHANGE_TIME);
                            isInUseIntake = !isReadyForExchangeIntake;
                        }
                        if (gamepad2.dpad_down) {
                            intakeDirecion = EJECT;
                        }
                    } else {  //No SAMPLE in intake
                        if (gamepad2.left_stick_y != 0) {
                            intakeSlidePower = gamepad2.left_stick_y;
                        }
                        if (gamepad2.dpad_left) {
                            intakeSlidePower = getIntakeSlidePower(INTAKE_SLIDE_EXCHANGE_POSITION);
                        }
                        if (gamepad2.left_stick_button) {
                            isActiveIntake = hasActivatedIntake(!isActiveIntake);
                        }
                    }

                    vertSlidePower = getVerticalSlidePower(VERT_PRE_EXCHANGE_HEIGHT);
                    isReadyForExchangeVert = hasSetClawArmPosition(VERT_PRE_EXCHANGE_HEIGHT)
                            && hasSetVertPosition(VERT_PRE_EXCHANGE_HEIGHT);
                }
                break;
            case POSITION_CLAW_FOR_EXCHANGE:
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                    isReadyForExchangeIntake = false;
                }
                isReadyForExchangeVert =
                        hasSetVertPosition(VERT_PRE_EXCHANGE_HEIGHT)
                                && hasSetClawArmPosition(CLAW_EXCHANGE_POSITION);
                break;
            case EXCHANGE_SAMPLE:
                isLoadedClaw = hasGrippedSample();
                if (isLoadedClaw) {
                    gamePiece = GamePieceType.SAMPLE;
                    isReadyForExchangeVert = false;
                    isReadyForExchangeIntake = false;
                }
                break;
            case ALIGN_SLIDE_FOR_POINTS:
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = true;
                    isLoadedClaw = false;
                } else if (gamePiece == GamePieceType.SAMPLE) {
                    if (gamepad2.a) {
                        vertSlidePower = getVerticalSlidePower(VERT_LOW_BASKET_HEIGHT);
                        isReadyForReleaseSample =
                                hasSetVertPosition(VERT_LOW_BASKET_HEIGHT)
                                        && hasSetClawArmPosition(CLAW_BASKET_DROP_POSITION);
                    } else if (gamepad2.x) {
                        vertSlidePower = getVerticalSlidePower(VERT_HIGH_BASKET_HEIGHT);
                        isReadyForReleaseSample =
                                hasSetVertPosition(VERT_HIGH_BASKET_HEIGHT)
                                        && hasSetClawArmPosition(CLAW_BASKET_DROP_POSITION);
                    }
                } else if (gamePiece == GamePieceType.SPECIMEN) {
                    if (gamepad2.b) {
                        specimenHangHeight = VERT_LOW_CHAMBER_HEIGHT;
                    } else if (gamepad2.y) {
                        specimenHangHeight = VERT_HIGH_CHAMBER_HEIGHT;
                    }
                    vertSlidePower = getVerticalSlidePower(specimenHangHeight);
                    isReadyToHangSpecimen =
                            hasSetVertPosition(specimenHangHeight)
                                    && hasSetClawArmPosition(CLAW_CHAMBER_HANG_POSITION);
                }
                break;
            case RUN_RELEASE_SAMPLE:
                if (gamepad2.left_bumper) {
                    if (hasOpenedClaw()) {
                        isReadyForReleaseSample = false;
                        gamePiece = GamePieceType.NONE;
                    }
                }
                break;
            case RUN_HANG_SPECIMEN:
                if (gamepad2.left_bumper) {
                    vertSlidePower = getVerticalSlidePower(specimenHangHeight - VERT_SPECIMEN_DROP_DISTANCE);
                    if (vertSlidePower == 0) {
                        isReadyToHangSpecimen = hasOpenedClaw();
                        if (!isReadyToHangSpecimen) {
                            gamePiece = GamePieceType.NONE;
                        }
                    }
                }
                break;
            case RUN_SLIDE_RECALIBRATION:
                if (gamepad1.dpad_down || gamepad1.dpad_right) {
                    recalibrateSlides(gamepad1.dpad_right, gamepad2.dpad_down);
                }
                if (gamepad1.right_stick_button) {
                    resetEncoders();
                    isRecalibrationMode = false;
                }
                if (gamepad1.dpad_up) {
                    isRecalibrationMode = false;
                }
                break;
            case RUN_MANUAL_MODE:
                if (gamepad1.back) {
                    isEndGame = true;
                } else if (gamepad2.back) {
                    isManualMode = false;
                } else {
                    intakeSlidePower = gamepad2.left_stick_y;
                    vertSlidePower = gamepad2.right_stick_y;
                    if (gamepad2.a) {
                        intakeDirecion = INTAKE;
                    } else if (gamepad2.b) {
                        intakeDirecion = EJECT;
                    } else {
                        intakeDirecion = IDLE;
                    }
                    if (gamepad2.right_trigger != 0) {
                        isPositionedClawArm = hasRotatedClawForward();
                    } else if (gamepad2.left_trigger != 0) {
                        isPositionedClawArm = hasRotatedClawBackward();
                    }
                    if (gamepad2.left_bumper) {
                        if (isSetClawGrip) {
                            if (clawServo.getPosition() == CLAW_OPEN) {
                                isSetClawGrip = hasClosedClaw();
                            } else {
                                isSetClawGrip = hasOpenedClaw();
                            }
                        }
                    }
                }
                break;
            case RUN_END_GAME_MODE:
                double tilt = climbLiftServo.getPosition();
                if (gamepad1.y) {
                    tilt = TILT_UP;
                }
                if (gamepad1.x) {
                    tilt = TILT_DOWN;
                }

                boolean isTilted = hasTiltedRobot(tilt);
                if (gamepad2.left_stick_y != 0) {
                    vertSlidePower = gamepad2.left_stick_y;
                    isInitiatedClimb = true;
                }
                if (gamepad1.back && !isInitiatedClimb) {
                    isEndGame = false;
                    if (isTilted) {
                        isTilted = hasTiltedRobot(TILT_DOWN);
                    }
                }
        }
        useIntakeArm(intakeSlidePower);
        setVertSlidesByPower(vertSlidePower);
        runIntakeMotor(intakeDirecion);

        telemetry.addData("Action", robotAction);
        telemetry.addData("Intake Slide Power", intakeSlidePower);
        telemetry.addData("Vertical Slide Power", vertSlidePower);
        telemetry.addData("Intake Motor Direction", intakeDirecion);

    }

    private void driveRobot(double drivePower, double strafePower, double turnLeftPower, double turnRightPower) {

        final double TURN_POWER_PERCENT = 1;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //drive controls forward power based on Left Drive Stick's Y position
        double drive = -drivePower * drivePowerPercent;

        //turn controls left/right power based on Right and Left Triggers
        double turn;
        if (turnLeftPower > 0.0) {
            turn = -turnLeftPower * TURN_POWER_PERCENT;
        } else if (turnRightPower > 0.0) {
            turn = turnRightPower * TURN_POWER_PERCENT;
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

        telemetry.addData("Motors", "left (%.2f), right (%.2f)",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private boolean hasTiltedRobot(double position) {
        boolean b = (position == TILT_UP);
        if (climbLiftServo.getPosition() != position) {
            climbLiftServo.setPosition(position);
        }
        return b;
    }

    private boolean hasGrippedSample() {
        double motorPower = 0;
        double pos = Math.min(slideVertLeftMotor.getCurrentPosition(), slideVertRightMotor.getCurrentPosition());

        if (pos > VERT_MAKE_EXCHANGE_HEIGHT) {
            motorPower = .15;
        } else {
            return hasClosedClaw();
        }

        slideVertLeftMotor.setPower(motorPower);
        slideVertRightMotor.setPower(motorPower);
        return false;
    }

    private void useIntakeArm(double power) {

        if (power == 0 && !isInUseIntake) {
            return;
        }

        double motorPower = 0;
        double slidePosition = intakeSlideMotor.getCurrentPosition();
        double armPosition = intakePivotArmServo.getPosition();

        if (isActiveIntake) {
            if (power > 0) {
                if (slidePosition < INTAKE_SLIDE_MAX_EXTENSION) {
                    motorPower = power;
                }
            } else if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE) {
                motorPower = INTAKE_SLIDE_TRAWL_SPEED;
            }
        } else {
            if (power > 0) {
                if (slidePosition < INTAKE_SLIDE_MAX_EXTENSION) { //If MAX_POSITION not reached
                    motorPower = power; //Set power based on y position
                }
                if (slidePosition > INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE
                        && armPosition != INTAKE_ARM_CLEARANCE_POSITION
                        && armPosition != INTAKE_ARM_ACTIVE_POSITION) { // if pivot arm is not in EXTEND or ACTIVE position and slide is beyond MIN_EXT
                    intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);  //set pivot arm to EXTEND
                }
            } else if (power < 0) {  //retract arm when pulling left stick down
                if (slidePosition > INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD) {
                    motorPower = power;
                } else if (slidePosition > INTAKE_SLIDE_EXCHANGE_POSITION) {
                    motorPower = INTAKE_SLIDE_SLOW_RETRACT_POWER;
                } else {
                    isInUseIntake = hasSetIntakeArmPosition(INTAKE_ARM_EXCHANGE_POSITION, INTAKE_ARM_EXCHANGE_TIME);
                }
            }
        }
        intakeSlideMotor.setPower(motorPower);  //set intakeSlide power to motorPower
    }

    private void runIntakeMotor(IntakeMotorConstants.IntakeDirection intakeDirection) {

        if (intakeDirection == IDLE) {
            return;
        }

        double motorPower = 0;
        double multiplier = (intakeDirection == INTAKE ? 1 : -1);
        if (isRunningIntake) {
            if (intakeMotorTimer.time() < INTAKE_MOTOR_REVOLUTION_TIME) {
                motorPower = INTAKE_MOTOR_POWER;
            } else {
                isRunningIntake = false;
                intakeMotorTimer.reset();
            }
        } else {
            if (intakeMotorTimer.time() > INTAKE_MOTOR_REVOLUTION_TIME) {
                motorPower = INTAKE_MOTOR_POWER;
                intakeMotorTimer.reset();
            }
        }
        intakeFrameServo.setPower(motorPower*multiplier);
    }

    private boolean hasSetVertPosition(double position) {
        return ((Math.abs(position - slideVertLeftMotor.getCurrentPosition()) <= VERT_MARGIN_OF_ERROR
                && Math.abs(position - slideVertRightMotor.getCurrentPosition()) <= VERT_MARGIN_OF_ERROR));
    }

    private boolean hasSetClawArmPosition(double position) {
        if (clawPivotArmServo.getPosition() != position) {
            clawPivotArmServo.setPosition(position);
            clawArmTimer.reset();
            return false;
        }
        return (clawArmTimer.time() > CLAW_ARM_CHANGE_TIME);
    }

    private boolean hasOpenedClaw() {
        if (clawServo.getPosition() != CLAW_OPEN) {
            clawServo.setPosition(CLAW_OPEN);
            clawMotorTimer.reset();
            return false;
        } else {
            return clawMotorTimer.time() > CLAW_CHANGE_TIME;
        }
    }

    private boolean hasClosedClaw() {
        if (clawServo.getPosition() != CLAW_CLOSE) {
            clawServo.setPosition(CLAW_CLOSE);
            clawMotorTimer.reset();
            return false;
        } else {
            return clawMotorTimer.time() > CLAW_CHANGE_TIME;
        }
    }

    private boolean hasSetIntakeArmPosition(double position, double time) {
        if (intakePivotArmServo.getPosition() != position) {
            intakePivotArmServo.setPosition(position);
            intakeArmTimer.reset();
            return false;
        }
        return intakeArmTimer.time() > time;
    }

    private boolean hasActivatedIntake(boolean activate) {
        if (activate) {
            if (intakeSlideMotor.getCurrentPosition() < INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE) {
                return false;
            }
            intakePivotArmServo.setPosition(INTAKE_ARM_ACTIVE_POSITION);
            return true;
        }
        intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);
        return false;

    }

    private boolean hasRotatedClawForward() {
        double pos = clawPivotArmServo.getPosition();
        if (isPositionedClawArm) {
            if (pos == CLAW_BASKET_DROP_POSITION) {
                clawPivotArmServo.setPosition(CLAW_EXCHANGE_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_CHAMBER_HANG_POSITION) {
                clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_WALL_SPECIMEN_POSITION) {
                clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
                clawArmTimer.reset();
                return false;
            } else {
                return true;
            }
        } else if (clawArmTimer.time() > CLAW_ARM_CHANGE_TIME) {
            if (pos == CLAW_BASKET_DROP_POSITION) {
                clawPivotArmServo.setPosition(CLAW_EXCHANGE_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_CHAMBER_HANG_POSITION) {
                clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_WALL_SPECIMEN_POSITION) {
                clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
                clawArmTimer.reset();
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    private boolean hasRotatedClawBackward() {
        double pos = clawPivotArmServo.getPosition();
        if (isPositionedClawArm) {
            if (pos == CLAW_CHAMBER_HANG_POSITION) {
                clawPivotArmServo.setPosition(CLAW_WALL_SPECIMEN_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_BASKET_DROP_POSITION) {
                clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_EXCHANGE_POSITION) {
                clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
                clawArmTimer.reset();
                return false;
            } else {
                return true;
            }
        } else if (clawArmTimer.time() > CLAW_ARM_CHANGE_TIME) {
            if (pos == CLAW_CHAMBER_HANG_POSITION) {
                clawPivotArmServo.setPosition(CLAW_WALL_SPECIMEN_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_BASKET_DROP_POSITION) {
                clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
                clawArmTimer.reset();
                return false;
            } else if (pos == CLAW_EXCHANGE_POSITION) {
                clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
                clawArmTimer.reset();
                return false;
            } else {
                return true;
            }
        }
        return false;
    }

    private void recalibrateSlides(boolean recalIntake, boolean recalVert) {
        isRecalibrationMode = true;
        double intakeMotorPower = 0;
        double vertMotorPower = 0;
        if (recalIntake) {
            intakeMotorPower = INTAKE_SLIDE_RECALIBRATE_SPEED;
        }
        intakeSlideMotor.setPower(intakeMotorPower);
        if (recalVert) {
            vertMotorPower = VERT_SLIDE_RECALIBRATE_SPEED;
        }
        slideVertLeftMotor.setPower(vertMotorPower);
        slideVertRightMotor.setPower(vertMotorPower);
    }

    private double getVerticalSlidePower(double target) {

        double slideLeft = slideVertLeftMotor.getCurrentPosition();
        double slideRight = slideVertRightMotor.getCurrentPosition();

        double leftOffset = slideLeft - target;
        double rightOffset = slideRight - target;

        double leftMotorPower = 0;
        double rightMotorPower = 0;

        if (Math.abs(leftOffset) > VERT_SLOW_MOTOR_THRESHOLD) {
            leftMotorPower = (leftOffset > 0) ? VERT_FAST_DROP : VERT_FAST_RAISE;
        } else if (Math.abs(leftOffset) > VERT_MARGIN_OF_ERROR) {
            leftMotorPower = (leftOffset > 0) ? VERT_SLOW_DROP : VERT_SLOW_RAISE;
        }

        if (Math.abs(rightOffset) > VERT_SLOW_MOTOR_THRESHOLD) {
            rightMotorPower = (rightOffset > 0) ? VERT_FAST_DROP : VERT_FAST_RAISE;
        } else if (Math.abs(rightOffset) > VERT_MARGIN_OF_ERROR) {
            rightMotorPower = (rightOffset > 0) ? VERT_SLOW_DROP : VERT_SLOW_RAISE;
        }

        return Math.max(leftMotorPower, rightMotorPower);
    }

    private double getIntakeSlidePower(double target) {

        double position = intakeSlideMotor.getCurrentPosition();
        double pow = 0;
        if (target > position) {
            if (position < INTAKE_SLIDE_MAX_EXTENSION) {
                pow = INTAKE_SLIDE_FAST_EXTEND_POWER;
            }
        } else {
            boolean isAtClearanceHeight = hasSetIntakeArmPosition(INTAKE_ARM_CLEARANCE_POSITION, INTAKE_ARM_CLEARANCE_TIME);

            if (position > INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE) {
                pow = INTAKE_SLIDE_FAST_RETRACT_POWER;
            } else if (isAtClearanceHeight && position > INTAKE_SLIDE_EXCHANGE_POSITION + INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD) {
                pow = INTAKE_SLIDE_FAST_RETRACT_POWER;
            } else if (isAtClearanceHeight && position > INTAKE_SLIDE_EXCHANGE_POSITION + INTAKE_SLIDE_EXCHANGE_MARGIN_OF_ERROR) {
                pow = INTAKE_SLIDE_SLOW_RETRACT_POWER;
            } else if (position < INTAKE_SLIDE_EXCHANGE_POSITION - INTAKE_SLIDE_EXCHANGE_MARGIN_OF_ERROR) {
                pow = INTAKE_SLIDE_SLOW_ADJUST_POWER;
            } else if (Math.abs(position - INTAKE_SLIDE_MIN_EXTENSION_FOR_CLEARANCE) < INTAKE_SLIDE_EXCHANGE_MARGIN_OF_ERROR) {
                isRetractedIntake = true;
            }
        }
        return pow;
    }

    private void setVertSlidesByPower(double power) {

        if (power == 0) {
            return;
        }

        double slideLeftPower = 0;
        double slideRightPower = 0;

        double leftPos = slideVertLeftMotor.getCurrentPosition();
        double rightPos = slideVertRightMotor.getCurrentPosition();

        if (leftPos > VERT_MIN_HEIGHT && leftPos < VERT_MAX_HEIGHT) {
            slideLeftPower = power;
        }
        if (rightPos > VERT_MIN_HEIGHT && rightPos < VERT_MAX_HEIGHT) {
            slideRightPower = power;
        }

        slideVertRightMotor.setPower(slideRightPower);
        slideVertLeftMotor.setPower(slideLeftPower);
    }

    private void resetEncoders() {
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
