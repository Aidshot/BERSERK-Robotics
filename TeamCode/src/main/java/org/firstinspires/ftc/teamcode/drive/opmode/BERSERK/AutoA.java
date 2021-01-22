package org.firstinspires.ftc.teamcode.drive.opmode.BERSERK;

//FTC Import

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "BERSERK")
public class AutoA extends LinearOpMode {
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
        long shootWait = 300;

        waitForStart();

        robot.kicker.setPosition(kicker_out);
        Pose2d startPose = new Pose2d(-63.0,50, Math.toRadians(0.0));
        drive.setPoseEstimate(startPose);
        if (isStopRequested()) return;

        //SHOOT POSITION
        Trajectory A1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-25.0, 55.0), Math.toRadians(0.0))
                .addDisplacementMarker(() -> {
                    ((DcMotorEx) robot.shooter1).setVelocity(shooter_target_velo);
                   // ((DcMotorEx) robot.shooter2).setVelocity(((DcMotorEx) robot.shooter1).getVelocity());
                })
                .splineTo(new Vector2d(-3.0, 38.0), Math.toRadians(0.0))
                .build();

        //WOBBLE A POSITION
        Trajectory A2 = drive.trajectoryBuilder(A1.end())
                .splineToLinearHeading(new Pose2d(55.0, 20.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        //PICK UP WOBBLE 2
        Trajectory A3 = drive.trajectoryBuilder(A2.end(),true)
                .splineToLinearHeading(new Pose2d(-19.0, 38.0, Math.toRadians(135.0)), Math.toRadians(180.0))
                .build();

        //WOBBLE A POSITION
        Trajectory A4 = drive.trajectoryBuilder(A3.end(),true)
                .splineToLinearHeading(new Pose2d(55.0, 15.0, Math.toRadians(0.0)), Math.toRadians(0.0))
                .build();

        //PARK
        Trajectory A5 = drive.trajectoryBuilder(A4.end())
                .back(4)
                .build();

        //SET SERVOS
        robot.wobble_lift.setPosition(wobble_up);
        robot.wobble_claw.setPosition(wobble_close);
        robot.flap.setPosition(launch_angle);

        //SHOOT POSITION
        drive.followTrajectory(A1);

        //SHOOT x 3
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

        //WOBBLE A POSITION
        drive.followTrajectory(A2);

        //DROP WOBBLE 1
        robot.wobble_lift.setPosition(wobble_down);
        sleep(700);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(200);

        //MOVE TOWARDS WOBBLE 2
        drive.followTrajectory(A3);

        //GRAB WOBBLE 2
        robot.wobble_claw.setPosition(wobble_close);
        sleep(500);
        robot.wobble_lift.setPosition(wobble_up);
        sleep(700);

        //WOBBLE A POSITION
        drive.followTrajectory(A4);

        //DROP WOBBLE 2
        robot.wobble_lift.setPosition(wobble_down);
        sleep(700);
        robot.wobble_claw.setPosition(wobble_open);
        sleep(200);

        //PARK
        drive.followTrajectory(A5);


    }
}
