package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@TeleOp(name = "Teleop For Two", group = "Linear Opmode")
public class RobotTeleOpAlt extends OpMode
{
    Mecanum s_driveTrain;
    Intake s_intake;
    Launcher s_launcher;

    private ElapsedTime runTime = new ElapsedTime();

    boolean slomo = false;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        s_driveTrain = new Mecanum(hardwareMap);
        Mecanum.alt = false;

        try
        {
            s_intake = new Intake(hardwareMap);
            s_launcher = new Launcher(hardwareMap);
        } catch(Exception e) {

        }

        runTime.reset();
    }

    public void loop()
    {
        slomo = gamepad1.square;

        s_driveTrain.teleop(gamepad1, slomo);
        try
        {
            s_intake.teleop(gamepad2);
            s_launcher.teleop(gamepad2);
        } catch (Exception e) {

        }

        s_driveTrain.periodic(telemetry);
        try
        {
            s_intake.periodic(telemetry);
            s_launcher.periodic(telemetry);
        } catch(Exception e) {

        }
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();
    }
}
