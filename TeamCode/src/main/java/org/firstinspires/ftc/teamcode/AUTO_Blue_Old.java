package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "Auto Blue Old", group = "Autonomous")
public class AUTO_Blue_Old extends LinearOpMode {

    HardwareRobot robot = new HardwareRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        Pose2d initialPose = new Pose2d(0, -63, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder toSub = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(0, -25));
        Pose2d toSubEnd = new Pose2d(0, -24, Math.toRadians(180));
        // Headings: 180 = Left, 0 = Right, -90 = Down, 90=UP

        TrajectoryActionBuilder toSamples = drive.actionBuilder(toSubEnd)
                //Travel Around submersible and toward samples.
                .strafeToLinearHeading(new Vector2d(35, -30),Math.toRadians(-84))
                .strafeToLinearHeading(new Vector2d(35, -18),Math.toRadians(-82))
                .strafeTo(new Vector2d(44, -14)) // move back to sample 1
              //  .waitSeconds(1)
                .strafeTo(new Vector2d(44, -52)) // push sample 1
              //  .waitSeconds(2)
                .strafeTo(new Vector2d(43, -16)) // move back to sample 2
                .strafeTo(new Vector2d(57, -16)) // move over to sample 2
               // .waitSeconds(1)
                .strafeTo(new Vector2d(55, -52)) // push sample 2

               // .strafeTo(new Vector2d(50, -16)) // move back to sample 3
              //  .waitSeconds(.1)
             //   .strafeTo(new Vector2d(62, -16)) // move over to sample 3
              //  .strafeTo(new Vector2d(62, -52)) // push sample 3
              //  .waitSeconds(2)
             //   .strafeTo(new Vector2d(50, -90)) // back up

                //Push in Sample 1
             //   .strafeTo(new Vector2d(37, -90))// push sample to wall
            //    .strafeToLinearHeading(new Vector2d(3, -71),Math.toRadians(180))
            //    .strafeToLinearHeading(new Vector2d(0, -100),Math.toRadians(180))
                //.strafeTo(new Vector2d(15, -65))// bak up

            //    .strafeToLinearHeading(new Vector2d(-7, -33),Math.toRadians(90))
             //   .waitSeconds(5)
             //   .strafeTo(new Vector2d(54-25, -10-25)) // move over to sample 2
             //   .waitSeconds(100)

                //Push in Sample 2
             //   .strafeTo(new Vector2d(54-25, -50-25))// push sample to wall
                //.strafeTo(new Vector2d(52, -10)) // backup
                //.strafeTo(new Vector2d(66, -10)) // move over
                //Push in Sample 3
                // .strafeTo(new Vector2d(66,-48))
                //Relocate to retrieve Specemine 1
             //   .strafeToLinearHeading(new Vector2d(-20, -110),Math.toRadians(10))
                .splineToSplineHeading(new Pose2d(40,-50, Math.toRadians(0)), Math.toRadians(-95))
                //Press to wall for Spec1
              //  .waitSeconds(1)
                .strafeTo(new Vector2d(40, -64));//relocate
               // .waitSeconds(1)
              //  .strafeTo(new Vector2d(40, -62)); //press to wall

        Pose2d toSamplesEnd = new Pose2d(40, -63, Math.toRadians(0));

        TrajectoryActionBuilder toSub2 = drive.actionBuilder(toSamplesEnd)
                //Move to submersible to deliver Spec1
                //.waitSeconds(.15)
                .strafeTo(new Vector2d(40, -58))
            //    .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, -46),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-6, -46),Math.toRadians(195))
              //  .waitSeconds(1)
              //  .strafeTo(new Vector2d(-90, -79.5))
                .strafeTo(new Vector2d(-6, -36));


        //.waitSeconds(.25);
        Pose2d toSub2End = new Pose2d(-6, -36.5, Math.toRadians(180));


        TrajectoryActionBuilder toWall2 = drive.actionBuilder(toSub2End)
                //Return for Spec
              //  .waitSeconds(1)
                .strafeTo(new Vector2d(-4, -43))
             //   .strafeTo(new Vector2d(20-25, -50-25))
                .strafeToLinearHeading(new Vector2d(40, -64),Math.toRadians(0))
               // .strafeToLinearHeading(new Vector2d(-55, -110),Math.toRadians(0))
              //  .strafeTo(new Vector2d(-45, -120))

              //  .waitSeconds(1)

                //Press Wall for Spec2
               // .strafeTo(new Vector2d(-45, -110))
                .strafeTo(new Vector2d(40, -78));

        Pose2d toWall2End = new Pose2d(40, -78, Math.toRadians(0));

        TrajectoryActionBuilder toSub3 = drive.actionBuilder(toWall2End)
                //Move to submersible to deliver Spec1
                //.waitSeconds(.15)
                .strafeTo(new Vector2d(40, -58))
                //    .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(2, -47),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(0, -46),Math.toRadians(195))
                //  .waitSeconds(1)
                //  .strafeTo(new Vector2d(-90, -79.5))
                .strafeTo(new Vector2d(0, -36.5));

        Pose2d toSub3End = new Pose2d(-4, -36.5, Math.toRadians(180));

        TrajectoryActionBuilder toWall3 = drive.actionBuilder(toSub3End)
                .strafeTo(new Vector2d(-4, -43))
                //   .strafeTo(new Vector2d(20-25, -50-25))
                .strafeToLinearHeading(new Vector2d(40, -64),Math.toRadians(0))
                // .strafeToLinearHeading(new Vector2d(-55, -110),Math.toRadians(0))
                //  .strafeTo(new Vector2d(-45, -120))

                //  .waitSeconds(1)

                //Press Wall for Spec2
                // .strafeTo(new Vector2d(-45, -110))
                .strafeTo(new Vector2d(40, -78));
        Pose2d toWall3End = new Pose2d(40, -78, Math.toRadians(0));

        TrajectoryActionBuilder toSub4 = drive.actionBuilder(toWall3End)
                //Deliver Spec3
                //Move to submersible to deliver Spec1
                //.waitSeconds(.15)
                .strafeTo(new Vector2d(40, -58))
                //    .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(0, -46),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(2, -46),Math.toRadians(195))
                //  .waitSeconds(1)
                //  .strafeTo(new Vector2d(-90, -79.5))
                .strafeTo(new Vector2d(2, -36));


        Action trajectoryActionCloseOut = toSub4.fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(rotate.rotateUp());




        telemetry.addData("Initialization Complete", "Awaiting Start Command");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;
        robot.servohang.setPosition(1);
        robot.specimenClamp.setPosition(0);
        robot.servorotate.setPosition(.1);
        lift (1, 18.5);
        Actions.runBlocking(
                new SequentialAction(
                        toSub.build()
                )
        );
        lift (1, -5);
        sleep(400);
        robot.specimenClamp.setPosition(.4);
        lift (.5, -14);


        Actions.runBlocking(
                new SequentialAction(
                        toSamples.build()
                )
        );
        robot.specimenClamp.setPosition(0);
        sleep(400);
        lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub2.build()
                )
        );
        lift (1, -5);
        sleep(400);
        robot.specimenClamp.setPosition(.4);
        lift (.5, -13.5);

        Actions.runBlocking(
                new SequentialAction(
                        toWall2.build()
                )
        );
        robot.specimenClamp.setPosition(0);
        sleep(400);
        lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub3.build()
                )
        );
        lift (1, -5);
        sleep(400);
        robot.specimenClamp.setPosition(.4);
        lift (.5, -13.5);

        Actions.runBlocking(
                new SequentialAction(
                        toWall3.build()
                )
        );
        robot.specimenClamp.setPosition(0);
        sleep(400);
        lift (1, 18.5);
        // lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub4.build()
                )
        );
        lift (1, -18.5);
        sleep(1000);
        robot.specimenClamp.setPosition(.4);
        robot.liftH.setPower(-.2);
        lift (.5, -13.5);
        sleep(500);
    }


    public void lift(double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;
        int newLiftTarget2;



        if (opModeIsActive()) {

            newLiftTarget = robot.liftV.getCurrentPosition() + (int) (inches * (-1140/(3.5 * 3.1415))*.717);
            newLiftTarget2 = robot.liftV2.getCurrentPosition() + (int) (inches * (-1140/(3.5 * 3.1415))*.717);

            robot.liftV.setTargetPosition(newLiftTarget);
            robot.liftV2.setTargetPosition(newLiftTarget2);

            robot.liftV.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftV2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftV.setPower(Math.abs(power));
            robot.liftV2.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftV.isBusy()) {
                       telemetry.addData("Lift", "Running at %7d",
                                       robot.liftV.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftV.setPower(0);
          robot.liftV.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

        }
    }


    public void extend(double power, double inches)
    {     ElapsedTime runtime = new ElapsedTime();

        int newLiftTarget;

        if (opModeIsActive()) {

            newLiftTarget = robot.liftH.getCurrentPosition() + (int) (inches * (1140/(3.5 * 3.1415)));

            robot.liftH.setTargetPosition(newLiftTarget);

            robot.liftH.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            robot.liftH.setPower(Math.abs(power));

           /*while (opModeIsActive() &&
                                   robot.liftV.isBusy()) {
                       telemetry.addData("Lift", "Running at %7d",
                                       robot.liftV.getCurrentPosition());
                    telemetry.update();

               }
          robot.liftV.setPower(0);
          robot.liftV.setMode(DcMotor.RunMode.RUN_USING_ENCODER); */

        }
    }

}

