package com.example.trajectoryholder;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

// This class is set up with bounded generic class that can be called by meepmeep or roadrunner
// some good references:  https://docs.oracle.com/javase/tutorial/java/generics/bounded.html
// https://docs.oracle.com/javase/tutorial/java/generics/types.html - and next few lessons

public class trajHolder<T extends TrajectorySequenceBuilder /* & TrajectorySequenceBuilder*/> {
    private T t;  // T stands for "Type"

    public trajHolder()
    {

    }

//    public void trajSeqbuilder (TrajectorySequenceBuilder traj)
    public void trajSeqbuilder (T traj)
    {
        traj.splineTo(new Vector2d(-30, -36), Math.PI );
        traj.splineTo(new Vector2d(-60, 0), Math.PI * 1/2);

    }
}