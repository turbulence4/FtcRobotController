package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Launcher
{
    private DcMotor launcherMotorLeft, launcherMotorRight;
    private String what = "sorry noah";

    public Launcher(HardwareMap hardwareMap)
    {
        launcherMotorLeft = hardwareMap.get(DcMotor.class, "launcherMotorLeft");
        launcherMotorRight = hardwareMap.get(DcMotor.class, "launcherMotorRight");

        launcherMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void teleop(Gamepad gamepad)
    {
        if (gamepad.triangle) {
            launcherMotorLeft.setPower(-0.55);
            launcherMotorRight.setPower(0.55);
        } else if(gamepad.right_trigger > 0.0) {
            launcherMotorLeft.setPower(-0.44);
            launcherMotorRight.setPower(0.44);
        } else if(gamepad.circle) {
            launcherMotorLeft.setPower(-1);
            launcherMotorRight.setPower(1);
        } else {
            launcherMotorLeft.setPower(0);
            launcherMotorRight.setPower(0);
        }
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("launcherMotorLeft: " + launcherMotorLeft.getPower());
        telemetry.addLine("launcherMotorRight: " + launcherMotorRight.getPower());
        telemetry.addLine(what);
    }
}
