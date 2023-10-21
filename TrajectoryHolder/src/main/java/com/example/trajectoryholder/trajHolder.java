package com.example.trajectoryholder;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class trajHolder {

    public trajHolder()
    {

    }

//    public void trajSeqbuilder (TrajectorySequenceBuilder traj)
    public void trajSeqbuilder (TrajectorySequenceBuilder traj)
    {
        traj.splineTo(new Vector2d(-30, -36), Math.PI );
        traj.splineTo(new Vector2d(-60, 0), Math.PI * 1/2);

    }
}