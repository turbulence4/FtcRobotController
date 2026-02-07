package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Launcher
{
    private DcMotor launcherMotorRight;
    private DcMotor launcherMotorLeft;
    private String what = "Fuck You John";
    private double powerDifference = 0;
    private double baseHighPower = 0.94;
    private double highPower = baseHighPower;
    public static final double idealVoltage = 13;
    private double currentVoltage = 12.8;
    private VoltageSensor voltageSensor;
    private double powerRatio = 1;

    public Launcher(HardwareMap hardwareMap)
    {
        launcherMotorRight = hardwareMap.get(DcMotor.class, "launcherMotorRight");
        launcherMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        launcherMotorLeft = hardwareMap.get(DcMotor.class, "launcherMotorLeft");
        launcherMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void teleop(Gamepad gamepad)
    {
        currentVoltage = voltageSensor.getVoltage();
        powerRatio = Math.abs((idealVoltage - currentVoltage) / currentVoltage) + 1;

        if(gamepad.triangle) {
            launcherMotorRight.setPower((baseHighPower + powerDifference) * powerRatio);
            launcherMotorLeft.setPower((baseHighPower + powerDifference) * -powerRatio);
        } else if(gamepad.right_trigger > 0.0) {
            launcherMotorRight.setPower(0.68 * powerRatio);
            launcherMotorLeft.setPower(-0.68 * powerRatio);
        } else if(gamepad.circle) {
            launcherMotorRight.setPower(1);
            launcherMotorLeft.setPower(-1);
        } else {
            launcherMotorRight.setPower(0);
            launcherMotorLeft.setPower(0);
        }

        if(gamepad.dpadUpWasPressed())
        {
            powerDifference += 0.01;
        } else if(gamepad.dpadDownWasPressed()) {
            powerDifference -= 0.01;
        }

        highPower = baseHighPower + powerDifference;
    }

    public void periodic(Telemetry telemetry)
    {
        telemetry.addLine("launcherMotor: " + launcherMotorRight.getPower());
        telemetry.addLine("high power: " + highPower + "(default: " + baseHighPower + ")");
        telemetry.addLine("power ratio: " + powerRatio);
        telemetry.addLine(what);
    }
}
