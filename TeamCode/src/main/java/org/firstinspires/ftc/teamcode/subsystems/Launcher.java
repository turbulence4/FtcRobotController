package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Launcher
{
    private DcMotor launcherMotorLeft, launcherMotorRight;
    private String what = "Fuck You Noah";
    private double powerDifference = 0;
    private double baseHighPower = 0.61;
    private double highPower = baseHighPower;
    public static final double idealVoltage = 13;
    private double currentVoltage = 12.8;
    private VoltageSensor voltageSensor;
    private double powerRatio = 1;

    public Launcher(HardwareMap hardwareMap)
    {
        launcherMotorLeft = hardwareMap.get(DcMotor.class, "launcherMotorLeft");
        launcherMotorRight = hardwareMap.get(DcMotor.class, "launcherMotorRight");

        launcherMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    public void teleop(Gamepad gamepad)
    {
        currentVoltage = voltageSensor.getVoltage();
        powerRatio = Math.abs((idealVoltage - currentVoltage) / currentVoltage) + 1;

        if(gamepad.triangle) {
            launcherMotorLeft.setPower((-baseHighPower - powerDifference) * powerRatio);
            launcherMotorRight.setPower((baseHighPower + powerDifference) * powerRatio);
        } else if(gamepad.right_trigger > 0.0) {
            launcherMotorLeft.setPower(-0.45 * powerRatio);
            launcherMotorRight.setPower(0.45 * powerRatio);
        } else if(gamepad.circle) {
            launcherMotorLeft.setPower(-1);
            launcherMotorRight.setPower(1);
        } else {
            launcherMotorLeft.setPower(0);
            launcherMotorRight.setPower(0);
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
        telemetry.addLine("launcherMotorLeft: " + launcherMotorLeft.getPower());
        telemetry.addLine("launcherMotorRight: " + launcherMotorRight.getPower());
        telemetry.addLine("high power: " + highPower + "(default: " + baseHighPower + ")");
        telemetry.addLine("power ratio: " + powerRatio);
        telemetry.addLine(what);
    }
}
