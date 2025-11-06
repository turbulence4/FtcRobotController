package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@Autonomous(name = "MainAutonomous", group = "Main Bot")
public class MainAutonomousRed extends OpMode
{
    final double FEED_TIME = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    private ElapsedTime driveTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime launchTimer = new ElapsedTime();

    private DcMotor frontLeft, frontRight, backLeft, backRight, launcherMotorLeft, launcherMotorRight, intakeMotor;
    private CRServo beltLeft, beltRight, beltTopLeft, beltTopRight;

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
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //set zero power behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop()
    {

    }
}
