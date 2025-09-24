package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Teleop", group = "Linear Opmode")
public class RobotTeleOp extends OpMode
{
    Mecanum driveTrain;
    //Intake s_Intake;

    private ElapsedTime runTime = new ElapsedTime();

    boolean slomo = false;

    @Override
    public void init()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        driveTrain = new Mecanum(hardwareMap);

        runTime.reset();
    }

    public void loop()
    {
        if(gamepad1.y)
        {
            //if(slomo) {slomo = false;} else {slomo = true;}
            slomo = !slomo;
        }

        driveTrain.teleop(gamepad1, slomo);
        driveTrain.periodic(telemetry);
        telemetry.addData("Status", "Run Time: " + runTime.toString());
        telemetry.update();
    }
}
