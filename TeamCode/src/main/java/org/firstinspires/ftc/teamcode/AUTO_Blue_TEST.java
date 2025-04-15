package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous(name = "Auto Blue TEST", group = "Autonomous")
public class AUTO_Blue_TEST extends LinearOpMode {

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

        TrajectoryActionBuilder toSubCorrectionforw = drive.actionBuilder(toSubEnd)
               // .waitSeconds(5)
                .strafeTo(new Vector2d(0, -24));
        toSubEnd = new Pose2d(0, -24, Math.toRadians(180));
        TrajectoryActionBuilder toSubCorrectionback = drive.actionBuilder(toSubEnd)
                // .waitSeconds(5)
                .strafeTo(new Vector2d(0, -24));
        toSubEnd = new Pose2d(0, -24, Math.toRadians(180));

        TrajectoryActionBuilder toSamples = drive.actionBuilder(toSubEnd)
                //Travel Around submersible and toward samples.
                .strafeToLinearHeading(new Vector2d(20, -30),Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(20, -20),Math.toRadians(90)) //move forward to sample
                .afterDisp(5, () ->{ //open arm after 5 inches
                    robot.servosweep.setPosition(.3);
                })
                .strafeTo(new Vector2d(20, -50)) //move back to wall

                .strafeTo(new Vector2d(30, -20)) //move forward to sample 2
                .strafeTo(new Vector2d(30, -50)) //move back to wall

                .strafeTo(new Vector2d(40, -20)) //move forward to sample 3
                .strafeTo(new Vector2d(40, -50)) //move back to wall

                .strafeTo(new Vector2d(40, -40)) //back up

                .strafeToLinearHeading(new Vector2d(30, -50),Math.toRadians(0)); //manuver to wall



                /*      .strafeToLinearHeading(new Vector2d(20, -30),Math.toRadians(-90))
                //  .waitSeconds(2)
                .strafeToLinearHeading(new Vector2d(40, -22),Math.toRadians(45))
               // .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(40, -75),Math.toRadians(0)) //push in sample 1
              //  .waitSeconds(5)
                .afterTime(1, () ->{
                    robot.servosweep.setPosition(.3);
        })
                .strafeToLinearHeading(new Vector2d(45, -22),Math.toRadians(30)) // move to sample 2
                // .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(45, -75),Math.toRadians(0)) // push in sample 2
               //   .waitSeconds(5)
                .strafeToLinearHeading(new Vector2d(53, -22),Math.toRadians(10))//move to sample 3
               //  .waitSeconds(1) */
             //   .strafeToLinearHeading(new Vector2d(53, -75),Math.toRadians(-20));// push in sample 3

              //  .splineToSplineHeading(new Pose2d(40,-50, Math.toRadians(0)), Math.toRadians(-95))
                //Press to wall for Spec1
              //  .waitSeconds(1)
              //  .strafeTo(new Vector2d(40, -64));//relocate
               // .waitSeconds(1)
              //  .strafeTo(new Vector2d(40, -62)); //press to wall

        Pose2d toSamplesEnd = new Pose2d(53, -75, Math.toRadians(0));

        TrajectoryActionBuilder toSamplesCorrectionforw = drive.actionBuilder(toSamplesEnd)
                .strafeTo(new Vector2d(53, -75));
        toSamplesEnd = new Pose2d(53, -76, Math.toRadians(0));
        TrajectoryActionBuilder toSamplesCorrectionback = drive.actionBuilder(toSamplesEnd)
                .strafeTo(new Vector2d(53, -75));
        toSamplesEnd = new Pose2d(53, -76, Math.toRadians(0));

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

        TrajectoryActionBuilder toSub2Correctionforw = drive.actionBuilder(toSub2End)
                .strafeTo(new Vector2d(-6, -36.5));
        toSub2End = new Pose2d(-6, -36.5, Math.toRadians(180));
        TrajectoryActionBuilder toSub2Correctionback = drive.actionBuilder(toSub2End)
                .strafeTo(new Vector2d(-6, -36.5));
        toSub2End = new Pose2d(-6, -36.5, Math.toRadians(180));

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

        TrajectoryActionBuilder toWall2Correctionforw = drive.actionBuilder(toWall2End)
                .strafeTo(new Vector2d(40, -78));
        toWall2End = new Pose2d(40, -78, Math.toRadians(0));
        TrajectoryActionBuilder toWall2Correctionback = drive.actionBuilder(toWall2End)
                .strafeTo(new Vector2d(40, -78));
        toWall2End = new Pose2d(40, -78, Math.toRadians(0));

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

        TrajectoryActionBuilder toSub3Correctionforw = drive.actionBuilder(toSub3End)
                .strafeTo(new Vector2d(-4, -36.5));
        toSub3End = new Pose2d(-4, -36.5, Math.toRadians(180));
        TrajectoryActionBuilder toSub3Correctionback = drive.actionBuilder(toSub3End)
                .strafeTo(new Vector2d(-4, -36.5));
        toSub3End = new Pose2d(-4, -36.5, Math.toRadians(180));

        TrajectoryActionBuilder toWall3 = drive.actionBuilder(toSub3End)
                .strafeTo(new Vector2d(-4, -43))
                //   .strafeTo(new Vector2d(20-25, -50-25))
                .strafeToLinearHeading(new Vector2d(40, -64),Math.toRadians(0))
                .strafeTo(new Vector2d(40, -78));
        Pose2d toWall3End = new Pose2d(40, -78, Math.toRadians(0));

        TrajectoryActionBuilder toWall3Correctionforw = drive.actionBuilder(toWall3End)
                .strafeTo(new Vector2d(40, -78));
        toWall3End = new Pose2d(40, -78, Math.toRadians(0));
        TrajectoryActionBuilder toWall3Correctionback = drive.actionBuilder(toWall3End)
                .strafeTo(new Vector2d(40, -78));
        toWall3End = new Pose2d(40, -78, Math.toRadians(0));

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
        Pose2d toSub4End = new Pose2d(2, -36, Math.toRadians(180));

        TrajectoryActionBuilder toSub4Correction = drive.actionBuilder(toSub4End)
                .strafeTo(new Vector2d(2, -36));
       // toSub4End = new Pose2d(40, -63, Math.toRadians(180));
        Action trajectoryActionCloseOut = toSub4.fresh()
                //.strafeTo(new Vector2d(48, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(rotate.rotateUp());


        double currentDistance = robot.sensorDistance.getDistance(DistanceUnit.INCH);
        telemetry.addData("Current Distance", currentDistance);

        telemetry.addData("Initialization Complete", "Awaiting Start Command");
        telemetry.update();
        waitForStart();
        telemetry.addData("Current Distance", currentDistance);


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
       // sleep(100);
        telemetry.update();
        if (currentDistance > 4) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSubCorrectionforw.build()
                            )
            );

        }
        if (currentDistance < 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSubCorrectionback.build()
                    )
            );

        }
        lift (1, -5);
        sleep(50);
        robot.specimenClamp.setPosition(.4);
        lift (.5, -14);
       // robot.servosweep.setPosition(.3);


        Actions.runBlocking(
                new SequentialAction(
                        toSamples.build()
                )
        );
        telemetry.update();
        robot.servosweep.setPosition(0);
        if (currentDistance > 5) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSamplesCorrectionforw.build()
                    )
            );
        }
        if (currentDistance < 4) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSamplesCorrectionback.build()
                    )
            );
        }
        robot.specimenClamp.setPosition(0);
      //  sleep(400);
        lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub2.build()
                )
        );
        telemetry.update();

        if (currentDistance > 4) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSub2Correctionforw.build()
                    )
            );
        }
        if (currentDistance < 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSub2Correctionback.build()
                    )
            );
        }
        lift (1, -5);
      //  sleep(400);
        robot.specimenClamp.setPosition(.4);
        lift (.5, -13.5);

        Actions.runBlocking(
                new SequentialAction(
                        toWall2.build()
                )
        );
        telemetry.update();

        if (currentDistance > 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            toWall2Correctionforw.build()
                    )
            );
        }
        if (currentDistance > 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            toWall2Correctionback.build()
                    )
            );
        }
        robot.specimenClamp.setPosition(0);
      //  sleep(400);
        lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub3.build()
                )
        );
        telemetry.update();

        if (currentDistance > 4) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSub3Correctionforw.build()
                    )
            );
        }
        if (currentDistance < 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSub3Correctionback.build()
                    )
            );
        }
        lift (1, -5);
     //   sleep(400);
        robot.specimenClamp.setPosition(.4);
        lift (.5, -13.5);

        Actions.runBlocking(
                new SequentialAction(
                        toWall3.build()
                )
        );
        telemetry.update();

        if (currentDistance > 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            toWall3Correctionforw.build()
                    )
            );
        }
        if (currentDistance > 3) {
            Actions.runBlocking(
                    new SequentialAction(
                            toWall3Correctionback.build()
                    )
            );
        }
        robot.specimenClamp.setPosition(0);
       // sleep(400);
        lift (1, 18.5);
        // lift (1, 18.5);

        Actions.runBlocking(
                new SequentialAction(
                        toSub4.build()
                )
        );
        telemetry.update();

        if (currentDistance > 4) {
            Actions.runBlocking(
                    new SequentialAction(
                            toSub4Correction.build()
                    )
            );
        }
        lift (1, -18.5);
     //   sleep(1000);
        robot.specimenClamp.setPosition(.4);
        robot.liftH.setPower(-.2);
        lift (.5, -13.5);
     //   sleep(500);
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


            telemetry.addData("deviceName", robot.sensorDistance.getDeviceName() );
            telemetry.addData("range", String.format("%.01f in", robot.sensorDistance.getDistance(DistanceUnit.INCH)));

            telemetry.update();

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


    // Helper function to adjust pose using the distance sensor
    // Helper function to adjust Y-coordinate using the distance sensor
 //   public Pose2d adjustYWithSensor(Pose2d targetPose, double expectedDistance, double scaleFactor) {
      //  double measuredDistance = robot.getDistance(); // Get sensor reading

       // double correction = (measuredDistance - expectedDistance) / scaleFactor; // Calculate correction

        // Adjust only the Y-coordinate
    //    Pose2d correctedPose = new Pose2d(targetPose.position.x, targetPose.position.y + correction, targetPose.heading);

      //  telemetry.addData("Fixed Point Localization", "Adjusting Y");
       // telemetry.addData("Measured Distance (mm)", measuredDistance);
     //   telemetry.addData("Correction Applied (mm)", correction);
     //   telemetry.update();

     //   return correctedPose;
  //  }

}


