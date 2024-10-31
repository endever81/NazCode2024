package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, -60, Math.toRadians(180)))
                            //Move to submersible
                        .strafeTo(new Vector2d(0, -21))
                            //Travel Around submersible and toward samples.
                        .splineToSplineHeading(new Pose2d(10,-21, Math.toRadians(180)), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(35,-15, Math.toRadians(-90)), Math.toRadians(90))
                        .strafeTo(new Vector2d(45, -10))
                            //Push in Sample 1
                        .lineTo(new Vector2d(45, -50))  //.lineToY(-50)
                        .lineTo(new Vector2d(45, -10))  //.lineToY(-10)
                        .strafeTo(new Vector2d(55, -10))
                            //Push in Sample 2
                        .lineTo(new Vector2d(55, -50))  //.lineToY(-50)
                        .lineTo(new Vector2d(55, -10))  //.lineToY(-10)
                        .strafeTo(new Vector2d(65, -10))
                            //Push in Sample 3
                        .lineTo(new Vector2d(65, -50))  //.lineToY(-50)
                            //Relocate to retrieve Specemine 1
                        .splineToSplineHeading(new Pose2d(35,-50, Math.toRadians(0)), Math.toRadians(-90))
                            //Press to wall for Spec1
                        .strafeTo(new Vector2d(35, -61))
                            //Move to submersible to deliver Spec1
                        .strafeTo(new Vector2d(30, -50))
                        .lineToLinearHeading(new Pose2d(3,-21, Math.toRadians(180))) //.strafeToLinearHeading(new Vector2d(3, -21),Math.toRadians(180))
                            //Return for Spec2
                        .strafeTo(new Vector2d(35, -50))
                        .lineToLinearHeading(new Pose2d(35,-55, Math.toRadians(0)))  //.strafeToLinearHeading(new Vector2d(35, -61),Math.toRadians(0))
                            //Press Wall for Spec2
                        .strafeTo(new Vector2d(35, -61))
                            //Move to submersible to deliver Spec2
                        .strafeTo(new Vector2d(30, -50))
                        .lineToLinearHeading(new Pose2d(3,-21, Math.toRadians(180))) //.strafeToLinearHeading(new Vector2d(3, -21),Math.toRadians(180))
                            //Return to the wall
                        .strafeTo(new Vector2d(35, -50))
                        .lineToLinearHeading(new Pose2d(35,-55, Math.toRadians(0)))  //.strafeToLinearHeading(new Vector2d(35, -61),Math.toRadians(0))
                           //Pick up Spec3
                        .strafeTo(new Vector2d(35, -61)).strafeTo(new Vector2d(30, -50))
                           //Deliver Spec3
                        .lineToLinearHeading(new Pose2d(6,-21, Math.toRadians(180))) //.strafeToLinearHeading(new Vector2d(3, -21),Math.toRadians(180))

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}