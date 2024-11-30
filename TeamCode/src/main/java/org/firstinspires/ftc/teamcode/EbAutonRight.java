package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "EbAutonRight", group = "Robot")
public class EbAutonRight extends LinearOpMode {

    enum DriveMethod {
        DEFAULT_FORWARD,
        STRAFE,
        TURN
    }

    enum DriveDirection {
        FORWARD,
        BACKWARD,
        STRAFE_LEFT,
        STRAFE_RIGHT,
        TURN_LEFT,
        TURN_RIGHT
    }

    private DcMotor frontLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor backRightMotor = null;
    private DcMotor slideVertRightMotor = null;
    private DcMotor slideVertLeftMotor = null;
    private Servo clawPivotArmServo = null;
    private Servo clawServo = null;
    private DcMotor intakeSlideMotor = null;

    static final double TICKS_PER_MOTOR_REV = 384.5;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.127;     // For figuring circumference
    static final double TICKS_PER_INCH = 37.2; //(TICKS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //(WHEEL_DIAMETER_INCHES * Math.PI);

    final double DRIVE_POWER = .4;
    final double TURN_POWER = .6;




    @Override
    public void runOpMode() throws InterruptedException {

        initComponents();
        waitForStart();
        driveToChamberPosition();
        setClawArmPositionForSpecimenHang();
        hangSpecimen();
        driveToYellowSamples();
        pushYellowSamplesToScoringArea();
        driveToParkPosition();
    }

    private void setClawArmPositionForSpecimenHang() {
        runVerticalSlideToPosition(VerticalSlideConstants.VERT_HIGH_CHAMBER_HEIGHT);
        rotateClawArmToPosition(ClawConstants.CLAW_ARM_CHAMBER_HANG_POSITION);
    }

    private void runVerticalSlideToPosition(double position) {
        slideVertLeftMotor.setTargetPosition((int) position);
        slideVertRightMotor.setTargetPosition((int) position);
        slideVertLeftMotor.setPower(VerticalSlideConstants.VERT_FAST_RAISE);
        slideVertRightMotor.setPower(VerticalSlideConstants.VERT_FAST_RAISE);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slideVertRightMotor.isBusy() || slideVertLeftMotor.isBusy()) {

        }
        slideVertRightMotor.setPower(0);
        slideVertLeftMotor.setPower(0);

    }

    private void rotateClawArmToPosition(double position) {
        clawPivotArmServo.setPosition(position);
    }

    private void driveToChamberPosition() {
        drive(EbAutonLeft.DriveDirection.FORWARD, 37.5, DRIVE_POWER);
    }

    private void hangSpecimen() {
        double pos = slideVertLeftMotor.getCurrentPosition() + VerticalSlideConstants.VERT_SPECIMEN_HANG_OFFSET_DISTANCE;
        slideVertLeftMotor.setTargetPosition((int) pos);
        slideVertRightMotor.setTargetPosition((int) pos);
        slideVertLeftMotor.setPower(VerticalSlideConstants.VERT_FAST_RAISE);
        slideVertRightMotor.setPower(VerticalSlideConstants.VERT_FAST_RAISE);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slideVertLeftMotor.isBusy()) {
        }
        slideVertRightMotor.setPower(0);
        slideVertLeftMotor.setPower(0);
        clawServo.setPosition(ClawConstants.CLAW_OPEN);
    }

    private void driveToYellowSamples() {
        drive(EbAutonLeft.DriveDirection.BACKWARD,8,DRIVE_POWER);
        drive(EbAutonLeft.DriveDirection.STRAFE_RIGHT, 35, DRIVE_POWER);
        drive(EbAutonLeft.DriveDirection.FORWARD, 28, DRIVE_POWER);
    }

    private void pushYellowSamplesToScoringArea() {
        drive(EbAutonLeft.DriveDirection.STRAFE_RIGHT, 15, DRIVE_POWER);
        drive(EbAutonLeft.DriveDirection.BACKWARD, 55, DRIVE_POWER);
        drive(EbAutonLeft.DriveDirection.STRAFE_LEFT, 4, DRIVE_POWER);
        drive(EbAutonLeft.DriveDirection.FORWARD, 58, DRIVE_POWER);
        drive(EbAutonLeft.DriveDirection.STRAFE_RIGHT, 16, DRIVE_POWER);
        drive(EbAutonLeft.DriveDirection.BACKWARD, 55, DRIVE_POWER);
        drive(EbAutonLeft.DriveDirection.FORWARD, 55, DRIVE_POWER);
    }

    private void driveToParkPosition() {
        drive(EbAutonLeft.DriveDirection.STRAFE_LEFT, 37, DRIVE_POWER);
    }

    private void initComponents() {
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
        slideVertLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right Vertical Slide Motor
        slideVertRightMotor = hardwareMap.get(DcMotor.class, "Slide_right");
        slideVertRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Claw Pivot Arm Servo
        clawPivotArmServo = hardwareMap.get(Servo.class, "Claw_arm_servo");
        clawPivotArmServo.setPosition(ClawConstants.CLAW_ARM_CHAMBER_HANG_POSITION);

        //Claw Servo
        clawServo = hardwareMap.get(Servo.class, "Claw_servo");
        clawServo.setPosition(ClawConstants.CLAW_CLOSE);

        //Horizontal Intake Slide Motor
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Claw_slide");
        intakeSlideMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void drive(EbAutonLeft.DriveDirection direction, double inches, double power) {
        switch(direction) {
            case FORWARD:
                drive(inches, EbAutonLeft.DriveMethod.DEFAULT_FORWARD, power);
                break;
            case BACKWARD:
                drive(-inches, EbAutonLeft.DriveMethod.DEFAULT_FORWARD, power);
                break;
            case STRAFE_LEFT:
                drive(-inches, EbAutonLeft.DriveMethod.STRAFE, power);
                break;
            case STRAFE_RIGHT:
                drive(inches, EbAutonLeft.DriveMethod.STRAFE, power);
                break;
        }
    }

    private void turn(EbAutonLeft.DriveDirection direction, double degrees, double power) {
        final double TICKS_TO_DEGREES = 100;

        switch(direction) {
            case TURN_LEFT:
                drive(-degrees*TICKS_TO_DEGREES, EbAutonLeft.DriveMethod.TURN, power);
                break;
            case TURN_RIGHT:
                drive(degrees*TICKS_TO_DEGREES, EbAutonLeft.DriveMethod.TURN, power);
                break;
        }
    }

    public void drive(double inches, EbAutonLeft.DriveMethod method, double power) {

        int sMx = method == EbAutonLeft.DriveMethod.STRAFE ? -1 : 1;
        int tMx = method == EbAutonLeft.DriveMethod.TURN ? -1 : 1;

        double d = inches * TICKS_PER_INCH;

        double fr = frontRightMotor.getCurrentPosition() + d * sMx * tMx;
        double fl = frontLeftMotor.getCurrentPosition() + d;
        double br = backRightMotor.getCurrentPosition() + d * tMx;
        double bl = backLeftMotor.getCurrentPosition() + d * sMx;

        frontLeftMotor.setTargetPosition((int) fl);
        frontRightMotor.setTargetPosition((int) fr);
        backLeftMotor.setTargetPosition((int) bl);
        backRightMotor.setTargetPosition((int) br);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //       backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        double p = frontLeftMotor.getPower();
//        frontRightMotor.setPower(p * sMx * tMx);
//        backLeftMotor.setPower(p * sMx);
//        backRightMotor.setPower(p * tMx);


        while (opModeIsActive() && frontLeftMotor.isBusy()
        ) {
//            p = frontLeftMotor.getPower();
//            frontRightMotor.setPower(p * sMx * tMx);
//            backLeftMotor.setPower(p * sMx);
//            backRightMotor.setPower(p * tMx);

        }

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
