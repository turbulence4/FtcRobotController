package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Launcher
{
    private DcMotor launcherMotor;
    private String what = "Fuck You John";
    private double powerDifference = 0;
    private double baseHighPower = 0.79;
    private double highPower = baseHighPower;
    public static final double idealVoltage = 13;
    private double currentVoltage = 12.8;
    private VoltageSensor voltageSensor;
    private double powerRatio = 1;

    public Launcher(HardwareMap hardwareMap)
    {
        launcherMotor = hardwareMap.get(DcMotor.class, "launcherMotor");
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void teleop(Gamepad gamepad)
    {
        currentVoltage = voltageSensor.getVoltage();
        powerRatio = Math.abs((idealVoltage - currentVoltage) / currentVoltage) + 1;

        if(gamepad.triangle) {
            launcherMotor.setPower((baseHighPower + powerDifference) * powerRatio);
        } else if(gamepad.right_trigger > 0.0) {
            launcherMotor.setPower(0.67 * powerRatio);
        } else if(gamepad.circle) {
            launcherMotor.setPower(1);
        } else {
            launcherMotor.setPower(0);
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
        telemetry.addLine("launcherMotor: " + launcherMotor.getPower());
        telemetry.addLine("high power: " + highPower + "(default: " + baseHighPower + ")");
        telemetry.addLine("power ratio: " + powerRatio);
        telemetry.addLine(what);
    }
}
