package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mecanum;

public class Launcher
{
    private DcMotor launcherMotorLeft, launcherMotorRight;
    private String what = "Fuck You John";

    public Launcher(HardwareMap hardwareMap)
    {
        launcherMotorLeft = hardwareMap.get(DcMotor.class, "launcherMotorLeft");
        launcherMotorRight = hardwareMap.get(DcMotor.class, "launcherMotorRight");

        launcherMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void teleop(Gamepad gamepad1, Gamepad gamepad2)
    {
        if(!Mecanum.alt) {
            if (gamepad1.right_trigger > 0.0) {
                launcherMotorLeft.setPower(-1);
                launcherMotorRight.setPower(1);
            } else {
                launcherMotorLeft.setPower(0);
                launcherMotorRight.setPower(0);
            }
        } else {
            if (gamepad2.right_trigger > 0.0) {
                launcherMotorLeft.setPower(-1);
                launcherMotorRight.setPower(1);
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
