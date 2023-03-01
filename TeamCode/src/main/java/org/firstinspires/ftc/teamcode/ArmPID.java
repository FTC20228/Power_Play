package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class ArmPID extends OpMode {

    private PIDController controller;

    public static double p = 0.018, i = 0, d = 0;
    public static double f = -0.08;

    public static int target;

    private final double TIG = 537.6 / 360;

    private DcMotorEx arm;
    private DcMotorEx arm1;



    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        target = 0;

    }
    @Override
    public void loop(){
        controller.setPID(p, i, d);
        int armPos = arm.getCurrentPosition();
        int arm1Pos = arm1.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double pid1 = controller.calculate(arm1Pos, target);

        double ff = Math.cos(Math.toRadians(target/TIG)) * f;

        double Power = pid + ff;
        double Power1 = pid1 + ff;

        arm.setPower(Power);
        arm1.setPower(Power1);
        telemetry.addData("pos", armPos);
        telemetry.addData("pos1", arm1Pos);

        telemetry.addData("target", target);

        if(gamepad2.dpad_down){
            target = 0;
        }
        if(gamepad2.dpad_left){
            target = 2200;
        }


    }

}

