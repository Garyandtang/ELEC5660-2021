Use the script "test_trajectory.m" as the main entry point.

readonly folder: supposed to be "read only"
    quadModel_readonly.m: parameters of a 500g quadrotor
    quadEOM_readonly.m: dynamics model of quadrotor.
    run_trajectory_readonly: solve the equation of motion, receive desired trajectory, run your controller, iteratively. visualization is also included.

utils: useful functions.

test_trajectory.m: main entry.

-----------------------------------------------------------------------------------------------------------
controller.m: You have alredy had a good controller if you finished phase1. If you still have problems with your controller, please contact TAs.

trajectory_generator.m: What you need to work with. Design the trajectory for quadrotor given the path. And calculate desired state given current time.

Contact TAs with any questions you may have.

yzhangec@connect.ust.hk
xluaj@connect.ust.hk
