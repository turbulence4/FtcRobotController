package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher
{
    //temporary names
    private DcMotor launcherMotor1, launcherMotor2;
    private boolean isLauncherActive;

    public Launcher(HardwareMap hardwareMap)
    {
        launcherMotor1 = hardwareMap.get(DcMotor.class, "launcherMotor1");
        launcherMotor2 = hardwareMap.get(DcMotor.class, "launcherMotor2");
    }

    public void teleop(Gamepad gamepad1)
    {
        if(gamepad1.y)
        {
            isLauncherActive = !isLauncherActive;

            if(isLauncherActive)
            {
                launcherMotor1.setPower(0.7);
                launcherMotor2.setPower(-0.7);
            } else {
                launcherMotor1.setPower(0);
                launcherMotor2.setPower(0);
            }
        }
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("Launcher: ");
    }
}
