package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher
{
    private DcMotor launcherMotorLeft, launcherMotorRight;
    private String what = "Fuck You John";
    private boolean isLauncherActive;

    public Launcher(HardwareMap hardwareMap)
    {
        launcherMotorLeft = hardwareMap.get(DcMotor.class, "launcherMotor1");
        launcherMotorRight = hardwareMap.get(DcMotor.class, "launcherMotor2");
    }

    public void teleop(Gamepad gamepad1)
    {
        if(gamepad1.y)
        {
            isLauncherActive = !isLauncherActive;

            if(isLauncherActive)
            {
                launcherMotorLeft.setPower(0.7);
                launcherMotorRight.setPower(-0.7);
            } else {
                launcherMotorLeft.setPower(0);
                launcherMotorRight.setPower(0);
            }
        }
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("Launcher: ");
        telemetry.addLine(what);
    }
}
