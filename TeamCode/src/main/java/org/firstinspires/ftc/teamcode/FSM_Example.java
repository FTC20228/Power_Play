package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.controller.PIDController;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp(name="Drive")
public class FSM_Example extends OpMode{
    public Servo servoL, servoR, servo0, servoA;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear, arm, arm1;
    public ColorSensor colorA;
    MecanumDrive drive;

    // An Enum is used to represent lift states.
    // (This is one thing enums are designed to do)
    public enum LiftState {
        Lift_Start,
        Lift_Ready,
        Lift_Stack,
        Lift_Low,
        Lift_Mid,
        Lift_High,
        Lift_Dump_Ready,
        Lift_Reset
    };

    // The liftState variable is declared out here
    // so its value persists between loop() calls
    LiftState liftState = LiftState.Lift_Start;

    // Some hardware access boilerplate; these would be initialized in init()
    // the lift motor, it's in RUN_TO_POSITION mode

    // the dump servo
    // used with the dump servo, this will get covered in a bit
    ElapsedTime liftTimer = new ElapsedTime();

    final double Grabber_Open = 0; // the idle position for the dump servo
    final double Grabber_Closed = 1; // the dumping position for the dump servo
    final double Wrist_Home = 0;
    final double Wrist_Away = 1;

    // the amount of time the dump servo takes to activate in seconds
    final double Grabber_Time = 1;

    final int Lift_Home = 0;
    final int Lift_Stack = 500;
    final int Lift_Low = 1100; // the low encoder position for the lift
    final int Lift_Mid = 2200; // the mid encoder position for the lift
    final int Lift_High = 3400; // the high encoder position for the lift

    public void init(){
        liftTimer.reset();
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        servo0  = hardwareMap.get(Servo.class, "servo0");
        servoA = hardwareMap.get(Servo.class, "servoA");
        servoR = hardwareMap.get(Servo.class, "servoR");
        servoL = hardwareMap.get(Servo.class, "servoL");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setDirection(DcMotorSimple.Direction.REVERSE);
        servoR.setDirection(Servo.Direction.REVERSE);
        servoA.setDirection(Servo.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear);


        // hardware initialization code goes here
        // this needs to correspond with the configuration used
        servoR.scaleRange(0.24, 0.9);
        servoL.scaleRange(0.24, 0.9);
        servoA.scaleRange(0.23, 0.88);//change to 0.88
        servo0.scaleRange(0.8, 0.97);

        arm.setTargetPosition(Lift_Home);
        arm1.setTargetPosition(Lift_Home);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Gripper(0,0);

    }

    public void loop() {
        arm.setPower(1.0);
        arm1.setPower(1.0);

        switch (liftState) {
            case Lift_Start:
                // Waiting for some input
                if(gamepad1.b){
                    Gripper(1,0.5);
                    liftState = LiftState.Lift_Ready;
                }
                break;
            case Lift_Ready:
                if(gamepad1.dpad_right){ // The robot waited long enough, time to start
                    // retracting the lift
                    arm.setTargetPosition(Lift_Low);
                    arm1.setTargetPosition(Lift_Low);
                    Gripper(1, 1);
                    liftState = LiftState.Lift_Low;
                    liftTimer.reset();

                }
                else if(gamepad1.dpad_left){ // The robot waited long enough, time to start
                    // retracting the lift
                    arm.setTargetPosition(Lift_Mid);
                    arm1.setTargetPosition(Lift_Mid);
                    Gripper(1, 1);
                    liftState = LiftState.Lift_Mid;
                    liftTimer.reset();

                }
                else if(gamepad1.dpad_up){ // The robot waited long enough, time to start
                    // retracting the lift
                    arm.setTargetPosition(Lift_High);
                    arm1.setTargetPosition(Lift_High);
                    Gripper(1, 1);
                    liftState = LiftState.Lift_High;
                    liftTimer.reset();
                }
                else if(gamepad1.left_trigger>0){
                    arm.setTargetPosition(Lift_Stack);
                    arm1.setTargetPosition(Lift_Stack);
                    Gripper(0,0);
                    liftState = LiftState.Lift_Stack;
                    liftTimer.reset();
                }
            case Lift_Stack:
                if (Math.abs(arm.getCurrentPosition() - Lift_Stack) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    Gripper(0, 0);

                    liftTimer.reset();
                    liftState = LiftState.Lift_Dump_Ready;
                }
                break;

            case Lift_Low:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(arm.getCurrentPosition() - Lift_Low) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    Gripper(1, 1);

                    liftTimer.reset();
                    liftState = LiftState.Lift_Dump_Ready;
                }
                break;
            case Lift_Mid:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(arm.getCurrentPosition() - Lift_Mid) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    Gripper(1, 1);

                    liftTimer.reset();
                    liftState = LiftState.Lift_Dump_Ready;
                }
            case Lift_High:
                // check if the lift has finished extending,
                // otherwise do nothing.
                if (Math.abs(arm.getCurrentPosition() - Lift_High) < 10) {
                    // our threshold is within
                    // 10 encoder ticks of our target.
                    // this is pretty arbitrary, and would have to be
                    // tweaked for each robot.

                    // set the lift dump to dump
                    Gripper(1, 1);

                    liftTimer.reset();
                    liftState = LiftState.Lift_Dump_Ready;
                }
                break;
            case Lift_Dump_Ready:
                if (gamepad1.dpad_down) {
                    // The robot waited long enough, time to start
                    // retracting the lift
                    Gripper(0, 0);
                    arm.setTargetPosition(Lift_Home);
                    arm1.setTargetPosition(Lift_Home);
                    liftState = LiftState.Lift_Reset;
                }

                break;
            case Lift_Reset:
                if (Math.abs(arm.getCurrentPosition() - Lift_Home) < 10) {
                    liftState = LiftState.Lift_Start;
                }
                break;
            default:
                // should never be reached, as liftState should never be null
                liftState = LiftState.Lift_Start;
        }

        // small optimization, instead of repeating ourselves in each
        // lift state case besides LIFT_START for the cancel action,
        // it's just handled here
        if (gamepad1.y && liftState != LiftState.Lift_Start) {
            Gripper(0,0);
            arm.setTargetPosition(Lift_Home);
            arm1.setTargetPosition(Lift_Home);
            liftState = LiftState.Lift_Start;
        }

        // mecanum drive code goes here
        // But since none of the stuff in the switch case stops
        // the robot, this will always run!
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        {
            double driveSpeed;

            if(gamepad1.right_trigger>0){
                driveSpeed = 1;
            }
            else{
                driveSpeed = 0.6;
            }

            drive.moveInTeleop(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, driveSpeed);

            if(gamepad1.a){servo0.setPosition(1);}
            else if(gamepad1.x){servo0.setPosition(0);}



        }
    }
    public void Gripper(double grabber, double wrist) {
            servoA.setPosition(grabber);
            servoR.setPosition(wrist);
            servoL.setPosition(wrist);
    }
}
