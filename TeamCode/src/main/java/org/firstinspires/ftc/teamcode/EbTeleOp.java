package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_BASKET_DROP_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_BASKET_POUNCE_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CHAMBER_HANG_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_CLOSE;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_OPEN;
import static org.firstinspires.ftc.teamcode.ClawConstants.CLAW_WALL_SPECIMEN_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_ACTIVE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_CLEARANCE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeArmConstants.INTAKE_ARM_EXCHANGE_POSITION;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_EJECT_POWER;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.INTAKE_MOTOR_POWER;
import static org.firstinspires.ftc.teamcode.IntakeMotorConstants.IntakeDirection.*;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MAX_EXTENSION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_CLEARANCE_EXTENSION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_MIN_EXTENSION;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_RETRACT_POWER;
import static org.firstinspires.ftc.teamcode.IntakeSlideConstants.INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_FAST_RAISE;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_HIGH_BASKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_HIGH_CHAMBER_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_LOW_BASKET_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_LOW_CHAMBER_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MAX_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MIN_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_PRE_EXCHANGE_HEIGHT;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_MARGIN_OF_ERROR;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_DROP;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_MOTOR_THRESHOLD;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_SLOW_RAISE;
import static org.firstinspires.ftc.teamcode.VerticalSlideConstants.VERT_WALL_HEIGHT;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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
    private DigitalChannel intakeSwitch = null;

    //Class variables
    private RobotAction robotAction;
    private GamePieceType gamePiece = GamePieceType.NONE;

    private double drivePowerPercent;


    //New variables
    private boolean intakeIsRetracting = false;
    private boolean intakeIsLoaded = false;
    private double presetHeight = 0;

    private int counter = 0;
    private boolean toggleIntakeArmPosition = false;
//    private int clicks = 0;

    //Click variables to limit actions on buttons to one
    private int leftButtonClicks;
    private int leftBumperClicks;
    private int dpadLClicks;
    private int dpadRClicks;
    private int rightBumperClicks;

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
        toggleIntakeArmPosition = true;
    }

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
            D-Pad Right:    Cycle through Claw Arm Positions Right
            D-Pad Down:     Eject Sample from Intake
            D-Pad Left:     Cycle through Claw Arm Positions Left
            A:              Lower Drop (Basket or Chamber) Preset
            B:              Exchange preset (Claw and Vert)
            X:              Upper Drop (Basket or Chamber) Preset
            Y:              Wall Preset (Claw and Vert)
            Right Trigger:  Manual Claw Arm Right
            Left Trigger:   Manual Claw Arm Left
            Right Bumper:
            Left Bumper:    Claw operations
            Back (Select):  Toggle Manual Mode
         */

    @Override
    public void loop() {

        /*
        TEST THESE INDEPENDENTLY FIRST

        executeDriveAction();
        setRobotAction();
        executeRobotAction();

         */

        executeDriveActions();
        executeRobotActions();
        //testRobotActions();
        telemetry.addData("VERT POSITION", slideVertRightMotor.getCurrentPosition());
    }

    private double getPresetHeightsFromController() {
        double h = 0;
        if (gamepad2.a) {
            switch (gamePiece) {
                case SAMPLE:
                    h = VERT_LOW_BASKET_HEIGHT;
                    break;
                case SPECIMEN:
                    h = VERT_LOW_CHAMBER_HEIGHT;
                    break;
                default:
                    h = 0;
            }
        } else if (gamepad2.x) {
            switch (gamePiece) {
                case SAMPLE:
                    h = VERT_HIGH_BASKET_HEIGHT;
                    break;
                case SPECIMEN:
                    h = VERT_HIGH_CHAMBER_HEIGHT;
                    break;
                default:
                    h = 0;
            }
        } else if (gamepad2.b) {
            h = VERT_PRE_EXCHANGE_HEIGHT;
        } else if (gamepad2.y) {
            h = VERT_WALL_HEIGHT;
        }
        return h;
    }

    private double getIntakeArmPositionFromController(double position) {
        //use the Left Joystick push to toggle between Active and Clearance position on intake arm
        if (gamepad2.left_stick_button) {
            leftButtonClicks++;
            leftBumperClicks = 0;
            if (leftButtonClicks == 1) {
                toggleIntakeArmPosition = !toggleIntakeArmPosition;
            }
        } else if (gamepad2.left_bumper) {
            leftBumperClicks++;
            leftButtonClicks = 0;
            if (leftBumperClicks == 1) {
                toggleIntakeArmPosition = !toggleIntakeArmPosition;
            }
        } else {
            leftButtonClicks = 0;
            leftBumperClicks = 0;
        }
        if (!toggleIntakeArmPosition) {
            return INTAKE_ARM_CLEARANCE_POSITION;
        } else if (leftButtonClicks == 1) {
            return INTAKE_ARM_ACTIVE_POSITION;
        } else if (leftBumperClicks == 1) {
            return INTAKE_ARM_EXCHANGE_POSITION;
        } else {
            return position;
        }
    }

    private IntakeMotorConstants.IntakeDirection getIntakeMotorDirectionFromController() {
        if (intakePivotArmServo.getPosition() == INTAKE_ARM_ACTIVE_POSITION) {
            return INTAKE;
        } else {
            if (gamepad2.dpad_down) {
                return EJECT;
            } else {
                return IDLE;
            }
        }
    }

    private double getClawArmPositionFromController() {
        double pos = clawPivotArmServo.getPosition();
        double[] positions;
        positions = new double[]{
                CLAW_WALL_SPECIMEN_POSITION,
                CLAW_BASKET_POUNCE_POSITION,
                CLAW_BASKET_DROP_POSITION,
                CLAW_CHAMBER_HANG_POSITION,
                CLAW_EXCHANGE_POSITION};

        if (gamepad2.dpad_left) {
            dpadLClicks++;
            dpadRClicks = 0;
            if (dpadLClicks == 1) {
                for (int i = 0; i < positions.length - 1; i++) {
                    if (positions[i] == pos) {
                        return positions[i + 1];
                    }
                }
                return pos;
            }
        } else if (gamepad2.dpad_right) {
            dpadRClicks++;
            dpadLClicks = 0;
            if (dpadRClicks == 1) {
                for (int i = positions.length - 1; i > 0; i--) {
                    if (positions[i] == pos) {
                        return positions[i - 1];
                    }
                }
                return pos;
            }
        }
        return pos;
    }

    private double getClawPositionFromController() {

        if (gamepad2.right_bumper) {
            rightBumperClicks++;
            if (rightBumperClicks == 1) {
                if (clawServo.getPosition() == CLAW_OPEN) {
                    return CLAW_CLOSE;
                }
                return CLAW_OPEN;
            }
        }else {
            rightBumperClicks=0;
        }
        return clawServo.getPosition();
    }


    private void executeRobotActions() {

        if (presetHeight == 0) {
            presetHeight = getPresetHeightsFromController();
        }

        //INTAKE SLIDE
        runIntakeSlide(-gamepad2.left_stick_y);

        double clawSlideHeight = slideVertRightMotor.getCurrentPosition();

        //CLAW SLIDE (VERTICAL)
        if (gamepad2.right_stick_y != 0) {
            runClawSlide(-gamepad2.right_stick_y);
            presetHeight = 0;
        } else if (presetHeight > 0) {
            if (presetHeight == VERT_WALL_HEIGHT) {
                if (clawSlideHeight > presetHeight + VERT_MARGIN_OF_ERROR) {
                    runClawSlide(VERT_SLOW_DROP);
                } else if (clawSlideHeight < presetHeight - VERT_MARGIN_OF_ERROR) {
                    runClawSlide(VERT_SLOW_RAISE);
                } else {
                    presetHeight = 0;
                }
            }
            if (clawSlideHeight < presetHeight) {
                runClawSlide(VERT_FAST_RAISE);
            } else {
                runClawSlide(0);
                presetHeight = 0;
            }
        } else {
            runClawSlide(0);
        }

        //INTAKE ARM
        double intakeArmPosition = 0;
        if (gamepad2.left_bumper || gamepad2.left_stick_button) {
            intakeArmPosition = getIntakeArmPositionFromController(intakePivotArmServo.getPosition());
        }
        rotateIntakeArm(intakeArmPosition);

        //INTAKE MOTOR
        runIntakeMotor(getIntakeMotorDirectionFromController());

        //CLAW MOTOR
        if (presetHeight == 0) {
            if (clawSlideHeight > VERT_HIGH_BASKET_HEIGHT && clawPivotArmServo.getPosition() == CLAW_BASKET_POUNCE_POSITION) {
                rotateClawArm(CLAW_BASKET_DROP_POSITION);
            } else {
                rotateClawArm(getClawArmPositionFromController());
            }
        } else {
            if (presetHeight == VERT_PRE_EXCHANGE_HEIGHT) {
                rotateClawArm(CLAW_EXCHANGE_POSITION);
            } else if (presetHeight == VERT_WALL_HEIGHT) {
                rotateClawArm(CLAW_WALL_SPECIMEN_POSITION);
            } else if (presetHeight == VERT_HIGH_BASKET_HEIGHT ||
                    presetHeight == VERT_LOW_BASKET_HEIGHT) {
                rotateClawArm(CLAW_BASKET_POUNCE_POSITION);
            } else if (presetHeight == VERT_HIGH_CHAMBER_HEIGHT ||
                    presetHeight == VERT_LOW_CHAMBER_HEIGHT) {
                rotateClawArm(CLAW_CHAMBER_HANG_POSITION);
            }
        }

        //CLAW
        double clawPosition = getClawPositionFromController();
        if (clawPosition!=clawServo.getPosition()) {
            if (clawPosition==CLAW_OPEN) {
                if (clawPivotArmServo.getPosition()==CLAW_BASKET_POUNCE_POSITION) {
                    rotateClawArm(CLAW_BASKET_DROP_POSITION);
                }
            }
        }
        setClawPosition(clawPosition);

    }

    private void executeDriveActions() {

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

    private void driveRobot(double drivePower,
                            double strafePower,
                            double turnLeftPower,
                            double turnRightPower) {

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

//        telemetry.addData("Motors", "left (%.2f), right (%.2f)",
//                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void runIntakeSlide(double power) {
        double pos = intakeSlideMotor.getCurrentPosition();
        double pow = 0;
        //STOP retracting if slide position reaches MIN_EXTENSION;
        if (pos <= 0 && power < INTAKE_SLIDE_MIN_EXTENSION) {
            pow = 0;
            //STOP extending if slide position reaches MAX_EXTENSION
        } else if (pos >= INTAKE_SLIDE_MAX_EXTENSION && power > 0) {
            pow = 0;
            //STOP retracting slide if position reaches CLEARANCE and INTAKE is ACTIVE
        } else if (pos <= INTAKE_SLIDE_CLEARANCE_EXTENSION && power < 0
                && intakePivotArmServo.getPosition() == INTAKE_ARM_ACTIVE_POSITION) {
            pow = 0;
            //SLOW the retracting speed if intake is near the MIN point
        } else if (power < 0 && pos < INTAKE_SLIDE_SLOW_RETRACT_THRESHOLD) {
            pow = INTAKE_SLIDE_SLOW_RETRACT_POWER;
        } else {
            pow = power;
        }
        intakeSlideMotor.setPower(pow);
        intakeIsRetracting = (pow < 0);
    }

    private void runClawSlide(double power) {

        double pos = slideVertRightMotor.getCurrentPosition();
        double pow = 0;

        // if intake is retracting, then get the vert slide to the exchange height
        if (intakeIsRetracting) {
            if (pos < VERT_PRE_EXCHANGE_HEIGHT - VERT_MARGIN_OF_ERROR) {
                pow = VERT_SLOW_RAISE;
            } else if (pos > VERT_PRE_EXCHANGE_HEIGHT + VERT_MARGIN_OF_ERROR) {
                pow = VERT_SLOW_DROP;
            }
        } else {
            //STOP retracting if position is less than the MIN_HEIGHT
            if (pos <= VERT_MIN_HEIGHT && power < 0) {
                pow = 0;
                //STOP extending if position is greater than MAX_HEIGHT
            } else if (pos >= VERT_MAX_HEIGHT && power > 0) {
                pow = 0;
                //SLOW retracting if position is less than SLOW_THRESHOLD
            } else if (pos < VERT_SLOW_MOTOR_THRESHOLD && power < 0) {
                pow = VERT_SLOW_DROP;
            } else {
                pow = power;
            }
        }
        slideVertRightMotor.setPower(pow);
    }

    private void rotateIntakeArm(double position) {

        double pos = intakePivotArmServo.getPosition();

        if (intakeIsLoaded) {
            if (pos == INTAKE_ARM_ACTIVE_POSITION) {
                pos = INTAKE_ARM_CLEARANCE_POSITION;
                toggleIntakeArmPosition = false;
            } else {
                if (position == INTAKE_ARM_EXCHANGE_POSITION) {
                    pos = position;
                }
            }
        } else {
            if (pos == INTAKE_ARM_CLEARANCE_POSITION) {
                pos = position;
            } else {
                if (position == INTAKE_ARM_CLEARANCE_POSITION) {
                    pos = position;
                }
            }
        }
        if ((intakePivotArmServo.getPosition() != pos) && (pos != 0)) {
            intakePivotArmServo.setPosition(pos);
        }
    }

    private void runIntakeMotor(IntakeMotorConstants.IntakeDirection intakeDirection) {

        if (intakeDirection == IDLE) {
            intakeFrameServo.setPower(0);
            intakeIsLoaded = intakeSwitch.getState();
            return;
        }

        if (!intakeSwitch.getState()) {
            counter = 0;
        }

        double pow = 0;

        switch (intakeDirection) {
            case INTAKE:
                pow = INTAKE_MOTOR_POWER;
                break;
            case EJECT:
                pow = INTAKE_MOTOR_EJECT_POWER;
                break;
            case IDLE:
                pow = 0;
        }
        if (counter > 60) {
            intakeFrameServo.setPower(0);
            intakeIsLoaded = true;
        } else {
            intakeFrameServo.setPower(pow);
            intakeIsLoaded = false;
            counter++;
        }
    }

    private void setClawPosition(double position) {
        double pos = clawServo.getPosition();
        if (pos != position) {
            clawServo.setPosition(position);
            if (position == CLAW_OPEN) {
                gamePiece = GamePieceType.NONE;
            } else {
                if (clawPivotArmServo.getPosition() == CLAW_WALL_SPECIMEN_POSITION) {
                    gamePiece = GamePieceType.SPECIMEN;
                } else {
                    gamePiece = GamePieceType.SAMPLE;
                }
            }
        }
    }

    private void rotateClawArm(double position) {
        double pos = clawPivotArmServo.getPosition();
        if (pos != position) {
            clawPivotArmServo.setPosition(position);
        }
    }

/*

    private void grabSampleWithIntake() {

        //use the Left Joystick push to toggle between Active and Clearance position on intake arm
        if (gamepad2.left_stick_button) {
            clicks++;
            if (clicks == 1) {
                toggleIntakeArmPosition = !toggleIntakeArmPosition;
            }
        } else {
            clicks = 0;
        }

        // if toggled to TRUE and position isn't already Active, set to Active
        if (toggleIntakeArmPosition && intakePivotArmServo.getPosition() != INTAKE_ARM_ACTIVE_POSITION) {
            intakePivotArmServo.setPosition(INTAKE_ARM_ACTIVE_POSITION);
        }

        // if toggled to false and not
        if (!toggleIntakeArmPosition && intakePivotArmServo.getPosition() != INTAKE_ARM_CLEARANCE_POSITION) {
            intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);
        }

        if (!toggleIntakeArmPosition && gamepad2.left_bumper) {
            counter = 0;
            intakePivotArmServo.setPosition(INTAKE_ARM_EXCHANGE_POSITION);
        }

        if (gamepad2.dpad_down && !intakeSwitch.getState()) {
            counter = 0;
            intakeFrameServo.setPower(INTAKE_MOTOR_POWER);
        }

        if (intakeSwitch.getState() && counter > 60) {
            intakeFrameServo.setPower(0);
            intakePivotArmServo.setPosition(INTAKE_ARM_CLEARANCE_POSITION);
            toggleIntakeArmPosition = false;
            telemetry.addData("Arm Position", intakePivotArmServo.getPosition());
        }

        counter++;
    }

    private void setServos() {
        if (gamepad1.x) {
            clawPivotArmServo.setPosition(CLAW_EXCHANGE_POSITION);
        }
        if (gamepad1.y) {
            clawPivotArmServo.setPosition(CLAW_CHAMBER_HANG_POSITION);
        }
        if (gamepad1.b) {
            clawPivotArmServo.setPosition(CLAW_BASKET_DROP_POSITION);
        }
        if (gamepad1.a) {
            clawPivotArmServo.setPosition(CLAW_BASKET_POUNCE_POSITION);
        }
        if (gamepad1.right_stick_button) {
            clawPivotArmServo.setPosition(CLAW_WALL_SPECIMEN_POSITION);
        }
        if (gamepad1.left_stick_button) {
            intakePivotArmServo.setPosition(INTAKE_ARM_EXCHANGE_POSITION);
        }
        if (gamepad1.dpad_right) {
            clawServo.setPosition(CLAW_CLOSE);
        }
        if (gamepad1.dpad_left) {
            clawServo.setPosition(CLAW_OPEN);
        }

    }

    private void getSlidePositions() {
        double p = 0;
        if (gamepad1.dpad_up) {
            p = VERT_SLOW_RAISE;
        } else if (gamepad1.dpad_down) {
            p = VERT_SLOW_DROP;
        }
        slideVertRightMotor.setPower(p);
        telemetry.addData("Right Vert Slide:", slideVertRightMotor.getCurrentPosition());
    }

*/
}
