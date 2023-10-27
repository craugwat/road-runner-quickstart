package com.example.trajectoryholder;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

// This class is set to be called by meepmeep or roadrunner
// to achieve this we needed conditional compile (pre-processor directives) that are not native to Java
// a plugin is used to achieve this functionalty.  See references below:
//
// http://manifold.systems/android.html
// https://github.com/manifold-systems/manifold/tree/master/manifold-deps-parent/manifold-preprocessor
//

#define ModeA
#if ModeA
    #warning "MeepTest defined"
#else
    #error "MeepTest not defined"
#endif
public class trajHolder<T extends TrajectorySequenceBuilder /* & TrajectorySequenceBuilder*/> {
    private T t;  // T stands for "Type"
    T parent = null;

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