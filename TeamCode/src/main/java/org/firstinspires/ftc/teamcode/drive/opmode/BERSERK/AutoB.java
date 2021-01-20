package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

//FTC Import

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//Road Runner Import

//Vision Import
//import org.firstinspires.ftc.teamcode.drive.opmode.UGContourRingDetector;
//import org.firstinspires.ftc.teamcode.drive.opmode.UGContourRingPipeline;

@Autonomous(group = "BERSERK")
public class AutoB extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareBERSERK robot    = new HardwareBERSERK();
        robot.init(hardwareMap);

        double shooter_target_velo = 1800;
        double launch_angle = 0.173;
        double kicker_out = 0.7;
        double kicker_in = 0.2;
        double wobble_close = 0.45;
        double wobble_open = 1;
        double wobble_up = 0.6;
        double wobble_down = 0.16;
        long shootWait = 400;

        waitForStart();

        robot.kicker.setPosition(kicker_out);
        Pose2d startPose = new Pose2d(-63.0,50, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);
        if (isStopRequested()) return;

        //SHOOT POSITION
        Trajectory B1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25.0, 55.0), Math.toRadians(0.0))
                .addDisplacementMarker(() -> {
                    ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                   // ((DcMotorEx) robot.shooter2).setVelocity(0);
                })
                .splineTo(new Vector2d(-3.0, 38.0), Math.toRadians(0.0))
                .build();

        //WOBBLE B POSITION
        Trajectory B2 = drive.trajectoryBuilder(B1.end())
                .splineToLinearHeading(new Pose2d(15.0, 38.0, Math.toRadians(-90.0)), Math.toRadians(0.0))
                .build();

        //MOVE TOWARDS STACK
        Trajectory B3 = drive.trajectoryBuilder(B2.end(),true)
                .splineToLinearHeading(new Pose2d(-19.0, 38.0, Math.toRadians(180.0)), Math.toRadians(180.0))
                .build();

        //SHOOT POSITION
        Trajectory B4 = drive.trajectoryBuilder(B3.end(),true)
                .splineToLinearHeading( new Pose2d(-3.0, 38.0, Math.toRadians(0)), Math.toRadians(180))
                .build();

        //PARK
        Trajectory B5 = drive.trajectoryBuilder(B4.end())
                .forward(9)
                .build();

        //SET SERVOS
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_close);
        robot.flap.setPosition(launch_angle);

        //SHOOT POSITION
        drive.followTrajectory(B1);

        //SHOOT x 3
        sleep(shootWait);
        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_out);

        //TURN OFF SHOOTER
        ((DcMotorEx) robot.shooter1).setVelocity(0);
        ((DcMotorEx) robot.shooter2).setVelocity(0);

        //WOBBLE C POSITION
        drive.followTrajectory(B2);

        //DROP WOBBLE 1
        robot.wobble_lift.setPosition(wobble_down);
        sleep(400);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(200);

        //MOVE TOWARDS STACK
        robot.intake.setPower(0.8);
        drive.followTrajectory(B3);

        robot.wobble_lift.setPosition(wobble_up);

       ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
      //  ((DcMotorEx) robot.shooter2).setVelocity(0);

        //SHOOT POSITION
        drive.followTrajectory(B4);

        //SHOOT X 3
        sleep(2500);

        robot.kicker.setPosition(kicker_out);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_in);
        sleep(shootWait);
        robot.kicker.setPosition(kicker_out);

        //TURN OFF INTAKE AND SHOOTER
        robot.intake.setPower(0);
        robot.feeder_turn.setPower(0);
        ((DcMotorEx) robot.shooter1).setVelocity(0);
        ((DcMotorEx) robot.shooter2).setVelocity(0);

        //PARK
        drive.followTrajectory(B5);
    }
}
