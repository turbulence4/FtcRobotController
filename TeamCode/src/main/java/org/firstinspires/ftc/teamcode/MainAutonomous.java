package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

    private DcMotorEx frontLeft, frontRight, backLeft, backRight, launcherMotorLeft, launcherMotorRight, intakeMotor;
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
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        launcherMotorLeft = hardwareMap.get(DcMotorEx.class, "launcherMotorLeft");
        launcherMotorRight = hardwareMap.get(DcMotorEx.class, "launcherMotorRight");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        beltLeft = hardwareMap.get(CRServo.class, "beltLeft");
        beltRight = hardwareMap.get(CRServo.class, "beltRight");
        beltTopLeft = hardwareMap.get(CRServo.class, "beltTop");
        beltTopRight = hardwareMap.get(CRServo.class, "beltDown");

        //set directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
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

    boolean launch(double speed, double time) {


        launcherMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        beltLeft.setPower(-speed);
        beltRight.setPower(speed);
        beltTopLeft.setPower(-speed);
        beltTopRight.setPower(-speed);
        launcherMotorRight.setPower(speed);
        launcherMotorLeft.setPower(-speed);
        intakeMotor.setPower(speed);

        return(launchTimer.seconds() > time);
    }

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

        launcherMotorRight.setPower(.44);
        launcherMotorLeft.setPower(-.44);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        if(Math.abs(frontLeft.getCurrentPosition()) > Math.abs(targetPosition))
        {
            return(true);
        }
        if(Math.abs(targetPosition - frontLeft.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
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
        double targetMm = angleUnit.toRadians(angle) * TRACK_WIDTH_MM;

        double leftTargetPosition = (targetMm * TICKS_PER_MM);
        double rightTargetPosition = targetMm * TICKS_PER_MM;

        frontLeft.setTargetPosition((int) -leftTargetPosition);
        frontRight.setTargetPosition((int) rightTargetPosition);
        backLeft.setTargetPosition((int) -leftTargetPosition);
        backRight.setTargetPosition((int) rightTargetPosition);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        if(Math.abs(frontLeft.getCurrentPosition()) > Math.abs(leftTargetPosition))
        {
            //return(true);
        }

        if(Math.abs((Math.abs(leftTargetPosition) - Math.abs(frontLeft.getCurrentPosition()))) > (30)) {

            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    @Override
    public void loop()
    {
        telemetry.addLine("looping");

        switch(autonomousState)
        {
            case DRIVING_OFF_LINE:
                telemetry.addLine("in auto switch");
                if(drive(.3,  -44, DistanceUnit.INCH, 1))
                {
                    telemetry.addLine("drive complete");
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.LAUNCH;

                    launchTimer.reset();
                    driveTimer.reset();
                }
                break;
            case LAUNCH:
                telemetry.addLine("In Launch");

                if(launch(.44,4)) {
                    telemetry.addLine("drive complete");
                    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    launcherMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    launcherMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    beltTopRight.setPower(0);
                    beltTopLeft.setPower(0);
                    beltRight.setPower(0);
                    beltLeft.setPower(0);

                    autonomousState = AutonomousState.COMPLETE;
                }

        }

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
