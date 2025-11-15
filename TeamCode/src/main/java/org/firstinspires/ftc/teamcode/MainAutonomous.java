package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.starterbot.StarterBotAuto;

@Autonomous(name = "MainAutonomous", group = "Main Bot")
public class MainAutonomous extends OpMode
{
    private IMU imu;
    final double FEED_TIME = 0.2;
    final double DRIVE_SPEED = 0.7;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime launchTimer = new ElapsedTime();

    private DcMotor frontLeft, frontRight, backLeft, backRight, launcherMotorLeft, launcherMotorRight, intakeMotor;
    private CRServo beltLeft, beltRight, beltTopLeft, beltTopRight;

    private String alliance = "red";

    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE;
    }

    private AutonomousState autonomousState;


    @Override
    public void init()
    {
        //assign
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        launcherMotorLeft = hardwareMap.get(DcMotor.class, "launcherMotorLeft");
        launcherMotorRight = hardwareMap.get(DcMotor.class, "launcherMotorRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        beltLeft = hardwareMap.get(CRServo.class, "beltLeft");
        beltRight = hardwareMap.get(CRServo.class, "beltRight");
        beltTopLeft = hardwareMap.get(CRServo.class, "beltTop");
        beltTopRight = hardwareMap.get(CRServo.class, "beltDown");

        //set directions
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //set modes
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcherMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set zero power behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        autonomousState = AutonomousState.DRIVING_OFF_LINE;

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot
                (RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        if (gamepad1.circle) {
            alliance = "red";
        } else if (gamepad1.square) {
            alliance = "blue";
        }

        telemetry.addData("press circle", "for red");
        telemetry.addData("press square", "for blue");
        telemetry.addData("selected alliance", alliance);
    }

    @Override
    public void start()
    {
        telemetry.addData("Status", "Started");
    }
/*
    public void driveTimeBased(double speed, double time)
    {
        ElapsedTime elapsed = new ElapsedTime();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        if(elapsed.seconds() > time)
        {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        telemetry.addData("frontleft drive auto: ", frontLeft.getPower());
        telemetry.addData("frontright drive auto: ", frontRight.getPower());
        telemetry.addData("backleft drive auto: ", backLeft.getPower());
        telemetry.addData("backright drive auto: ", backRight.getPower());
    }

    //negative speed = clockwise rotation
    public void rotateTimeBased(double speed, double time)
    {
        ElapsedTime elapsed = new ElapsedTime();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        if(elapsed.seconds() > time)
        {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }
    }
*/
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds)
    {
        final double TOLERANCE_MM = 10;

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        frontLeft.setTargetPosition((int)targetPosition);
        frontRight.setTargetPosition((int)targetPosition);
        backLeft.setTargetPosition((int)targetPosition);
        backRight.setTargetPosition((int)targetPosition);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.1);
        frontRight.setPower(-0.01);
        backLeft.setPower(0.1);
        backRight.setPower(0.01);

        if(Math.abs(targetPosition - frontLeft.getCurrentPosition()) > (100)) {
            frontLeft.setTargetPosition((int)targetPosition);
            frontRight.setTargetPosition((int)targetPosition);
            backLeft.setTargetPosition((int)targetPosition);
            backRight.setTargetPosition((int)targetPosition);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.1);
            frontRight.setPower(-0.1);
            backLeft.setPower(0.1);
            backRight.setPower(0.1);

            driveTimer.reset();
        }
        return(driveTimer.seconds() > holdSeconds);
    }

    /**
     * @param speed From 0-1
     * @param angle the amount that the robot should rotate
     * @param angleUnit the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if the motors are within tolerance of the target position for more than
     *         holdSeconds. False otherwise.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds){
        final double TOLERANCE_MM = 10;

        //number of mm to get to the angle (rad)
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);

        double leftTargetPosition = (targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        frontLeft.setTargetPosition((int) leftTargetPosition);
        frontRight.setTargetPosition((int) rightTargetPosition);
        backLeft.setTargetPosition((int) leftTargetPosition);
        backRight.setTargetPosition((int) rightTargetPosition);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0.01);
        frontRight.setPower(-0.01);
        backLeft.setPower(0.01);
        backRight.setPower(0.01);

        if(Math.abs((Math.abs(leftTargetPosition) - Math.abs(frontLeft.getCurrentPosition()))) > (30)) {

            //driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    @Override
    public void loop()
    {
        telemetry.addLine("looping");

        //drive(DRIVE_SPEED, 24, DistanceUnit.INCH, 1);

        switch(autonomousState)
        {

            case DRIVING_OFF_LINE:
                telemetry.addLine("in auto switch");
                if(drive(0.1,  -10, DistanceUnit.INCH, 0.5)){
                    telemetry.addLine("drive complete");
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.ROTATING;
                    break;

                }
            case ROTATING:
                telemetry.addLine("in Rotate");
                if(rotate(.5,90,AngleUnit.DEGREES, 1)) {
                    telemetry.addLine("drive complete");
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.COMPLETE;
                    break;
                }
        }

/*
        switch(alliance)
        {
            case "red":
                frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                telemetry.addLine("Inside Red");
                break;
            case "blue":
                telemetry.addLine("bwg");
                break;
            default:
                telemetry.addLine("how did you do this");
                break;
        }
*/
        telemetry.addData("Motor Current Positions", "left (%d), right (%d)",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "left (%d), right (%d)",
                frontLeft.getTargetPosition(), frontRight.getTargetPosition());
        telemetry.addData("Motor Current Positions", "left (%d), right (%d)",
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "left (%d), right (%d)",
                backLeft.getTargetPosition(), backRight.getTargetPosition());
    }

    @Override
    public void stop()
    {
        telemetry.addData("Status", "Stopped");
    }
}
