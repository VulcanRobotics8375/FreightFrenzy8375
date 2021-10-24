package org.firstinspires.ftc.teamcode.robotcorelib.motion.followers;

import org.firstinspires.ftc.teamcode.robotcorelib.motion.path.Path;

public class PurePursuit extends Follower {

     boolean following;

    public PurePursuit() {}

    public void followPath(Path path) {
        following = true;

        // base follower
        while(following) {

        }

        //mitigate pose error

    }

}
