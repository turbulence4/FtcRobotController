package org.firstinspires.ftc.teamcode.starterbot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp()

public class StarterRobotTeleOp extends OpMode {

    Mecanum s_driveTrain;
    Intake s_intake;
    LauncherStarter s_launcher;

    private ElapsedTime runTime = new ElapsedTime();

    boolean slomo = false;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        s_driveTrain = new Mecanum(hardwareMap, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD, DcMotor.Direction.REVERSE);

        Mecanum.alt = true;

        try {
            s_launcher = new LauncherStarter(hardwareMap);
        } catch (Exception e) {

        }

        runTime.reset();
    }

    public void loop()
    {
        slomo = gamepad1.square;

        s_driveTrain.teleop(gamepad1, slomo);
        try
        {


            s_launcher.teleop(gamepad1);

        } catch (Exception e) {

        }

        s_driveTrain.periodic(telemetry);
        try
        {
        } catch(Exception e) {

        }
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();
    }
}
