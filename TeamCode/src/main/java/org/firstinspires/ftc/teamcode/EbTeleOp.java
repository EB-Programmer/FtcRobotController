package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="EbTeleOp", group="Iterative Opmode")

public class EbTeleOp extends OpMode
{

    /*
    Set IS_TEST_MODE value to TRUE to enter test mode.  Use test mode to verify
    that components are working properly
     */
    private final boolean IS_TEST_MODE = true;

    //Robot components
    private ElapsedTime runtime = new ElapsedTime();
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

    //Class variables

    //Class constants
    private final double DRIVE_POWER_PERCENT = 0.75;
    private final double TURN_POWER_PERCENT = 1.0;
    private final double STRAFE_POWER_PERCENT = 1.0;
    private final double MAX_SLIDE_HEIGHT = 3175;
    private final double MIN_SLIDE_HEIGHT = 15;
    private final double MID_SLIDE_HEIGHT = 2500;
    private final double MAX_HORIZ_SLIDE_EXTENSION = 1600;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

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
        slideVertLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideVertLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Right Vertical Slide Motor
        slideVertRightMotor = hardwareMap.get(DcMotor.class, "Slide_right");
        slideVertRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVertRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideVertRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Horizontal Intake Slide Motor
        intakeSlideMotor = hardwareMap.get(DcMotor.class, "Claw_slide");
        intakeSlideMotor.setDirection(DcMotor.Direction.REVERSE);

        //Intake Pivot Arm Servo
        intakePivotArmServo = hardwareMap.get(Servo.class, "Intake_arm_servo");
        intakePivotArmServo.setPosition(.6);

        //Intake Frame Servo
        intakeFrameServo = hardwareMap.get(CRServo.class, "Intake_servo");

        //Claw Pivot Arm Servo

        //Claw Servo

        //Climb Lift Servo

        //Intake Color Sensor

        //Intake Roller Limit Switch

        //Gyro
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (IS_TEST_MODE) {
            runTestMode();
        } else {
            runTeleOpLoop();
        }
    }

    private void runTeleOpLoop() {

        setDrive();
        setVerticalSlides();
        setIntake();
        setClaw();
        runEndGameLift();

    }

    private void setDrive() {

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //drive controls forward power based on Left Drive Stick's Y position
        double drive = -gamepad1.left_stick_y*DRIVE_POWER_PERCENT;

        //turn controls left/right power based on Right and Left Triggers
        double turn;
        if(gamepad1.left_trigger > 0.0){
            turn = -gamepad1.left_trigger*TURN_POWER_PERCENT;
        }
        else if(gamepad1.right_trigger > 0.0){
            turn = gamepad1.right_trigger*TURN_POWER_PERCENT;
        }
        else
            turn = 0;

        //strafe controls side movement power based on Left Drive Stick's X position
        double strafe = gamepad1.left_stick_x*STRAFE_POWER_PERCENT;

        //calculate motor power
        frontLeftPower = Range.clip((drive + turn + strafe), -1.0, 1.0) ;
        frontRightPower = Range.clip((drive - turn - strafe), -1.0, 1.0) ;
        backLeftPower = Range.clip((drive + turn - strafe), -1.0, 1.0) ;
        backRightPower = Range.clip((drive - turn + strafe), -1.0, 1.0) ;

        // Send calculated power to motors
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("Motors", "left (%.2f), right (%.2f)",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    private void setVerticalSlides() {
        double slideLeftPower;
        double slideRightPower;
        slideLeftPower=0;
        slideRightPower=0;

        if (gamepad2.dpad_up && slideVertLeftMotor.getCurrentPosition() < 100){
            slideLeftPower = 0.62;
            slideRightPower = 0.62;
        }


        if (gamepad2.y && slideVertLeftMotor.getCurrentPosition()<MAX_SLIDE_HEIGHT){
            slideLeftPower = .4;
            slideRightPower = .4;
        }

        if (gamepad2.x && slideVertLeftMotor.getCurrentPosition()<MID_SLIDE_HEIGHT){
            slideLeftPower = .4;
            slideRightPower = .4;
        }

        if (gamepad2.a && slideVertLeftMotor.getCurrentPosition()>MIN_SLIDE_HEIGHT){
            slideLeftPower = -.4;
            slideRightPower = -.4;
        }

        if (slideVertLeftMotor.getCurrentPosition()<MIN_SLIDE_HEIGHT) {
            slideLeftPower = .4;
        }

        slideVertRightMotor.setPower(slideLeftPower);
        slideVertLeftMotor.setPower(slideRightPower);
    }

    private void setIntake(){

    }

    private void setClaw(){

    }

    private void runEndGameLift(){

    }

    //Test robot components independently
    private void runTestMode() {

        final double POWER_PERCENT = .25;

        double leftVertSlidePower = (gamepad1.dpad_left?0:1)*POWER_PERCENT;
        double rightVertSlidePower = (gamepad1.dpad_right?0:1)*POWER_PERCENT;
        double clawSlidePower = (gamepad1.dpad_up?0:1)*POWER_PERCENT;
        double frontLeftMotorPower = (gamepad1.y?0:1)*POWER_PERCENT;
        double frontRightMotorPower = (gamepad1.b?0:1)*POWER_PERCENT;
        double backLeftMotorPower = (gamepad1.x?0:1)*POWER_PERCENT;
        double backRightMotorPower = (gamepad1.a?0:1)*POWER_PERCENT;

        if (slideVertLeftMotor.getCurrentPosition()<MAX_SLIDE_HEIGHT) {
            slideVertLeftMotor.setPower(leftVertSlidePower);
        }

        if (slideVertRightMotor.getCurrentPosition()<MAX_SLIDE_HEIGHT) {
            slideVertRightMotor.setPower(rightVertSlidePower);
        }

        if (intakeSlideMotor.getCurrentPosition()<MAX_HORIZ_SLIDE_EXTENSION) {
            intakeSlideMotor.setPower(clawSlidePower);
        }

        frontLeftMotor.setPower(frontLeftMotorPower);
        frontRightMotor.setPower(frontRightMotorPower);
        backLeftMotor.setPower(backLeftMotorPower);
        backRightMotor.setPower(backRightMotorPower);
    }


    @Override
    public void stop() {
    }

}
