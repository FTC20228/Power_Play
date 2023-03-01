package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Park", group="Autonomous")

public class JustPark extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d());
        double open = 0;
        double closed = 1;

        drive.servoR.setDirection(Servo.Direction.REVERSE);
        drive.servoA.setDirection(Servo.Direction.REVERSE);

        drive.servoR.scaleRange(0.24, 0.9);
        drive.servoL.scaleRange(0.24, 0.9);
        drive.servoA.scaleRange(0.23, 0.88);//change to 0.88
        drive.servo0.scaleRange(0.8, 0.97);

        drive.servoL.setPosition(0);
        drive.servoR.setPosition(0);
        drive.servoA.setPosition(0);
        drive.servo0.setPosition(1);

        //drive.setPoseEstimate(new Pose2d());

        Trajectory toScan = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(26.5)
                .build();

        Trajectory toPost = drive.trajectoryBuilder(toScan.end())
                .addTemporalMarker(0.1, () -> {
                    armMove(2250); //Life Arm to Height
                })
                .strafeLeft(15)
                .build();

        TrajectorySequence parkRed = drive.trajectorySequenceBuilder(toPost.end())
                .strafeLeft(4)
                .back(30)
                .build();

        TrajectorySequence parkBlue = drive.trajectorySequenceBuilder(toPost.end())
                .strafeLeft(4)
                .forward(30)
                .build();

        TrajectorySequence parkGreen = drive.trajectorySequenceBuilder(toPost.end())
                .strafeLeft(3)
                .build();

        drive.colorR.enableLed(true);

        waitForStart();

        drive.followTrajectory(toScan);
        if (drive.colorR.red()>drive.colorR.green() && drive.colorR.red()>drive.colorR.blue()) {
            telemetry.addLine("Red");
            drive.followTrajectorySequence(parkRed);


        } else if (drive.colorR.green()>drive.colorR.blue() && drive.colorR.green()>drive.colorR.red()) {
            telemetry.addLine("Green");
            drive.followTrajectorySequence(parkGreen);
            sleep(500);


        } else if (drive.colorR.blue()>drive.colorR.green() && drive.colorR.blue()>drive.colorR.red()){
            telemetry.addLine("Blue");
            drive.followTrajectorySequence(parkBlue);
            sleep(500);
        }

        telemetry.update();
    }

    public void armMove(int target) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.arm.setTargetPosition(target);
        drive.arm1.setTargetPosition(target);
        drive.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.arm.setPower(1);
        drive.arm1.setPower(1);
    }

    public void Gripper(double grabber, double wrist) {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.servoA.setPosition(grabber);
        drive.servoR.setPosition(wrist);
        drive.servoL.setPosition(wrist);
    }
}
