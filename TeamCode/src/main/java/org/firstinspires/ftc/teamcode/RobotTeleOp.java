package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "Teleop", group = "Linear Opmode")
public class RobotTeleOp extends OpMode
{
    Mecanum s_driveTrain;
    Intake s_intake;

    private ElapsedTime runTime = new ElapsedTime();

    boolean slomo = false;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        s_driveTrain = new Mecanum(hardwareMap);

        try
        {
            s_intake = new Intake(hardwareMap);
        } catch(Exception e) {
            System.out.println("what");
        }

        runTime.reset();
    }

    public void loop()
    {
        if(gamepad1.y)
        {
            //if(slomo) {slomo = false;} else {slomo = true;}
            slomo = !slomo;
        }

        s_driveTrain.teleop(gamepad1, slomo);
        try
        {
            s_intake.teleop(gamepad1);
        } catch (Exception e) {
            System.out.println("what");
        }


        s_driveTrain.periodic(telemetry);
        try
        {
            s_intake.periodic(telemetry);
        } catch(Exception e) {
            System.out.println("what");
        }
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();
    }
}
