Use the script "test_trajectory.m" as the main entry point.

readonly folder: supposed to be "read only"
    quadModel_readonly.m: parameters of a 500g quadrotor
    quadEOM_readonly.m: dynamics model of quadrotor.
    run_trajectory_readonly: solve the equation of motion, receive desired trajectory, run your controller, iteratively. visualization is also included.

utils: useful functions.

test_trajectory.m: main entry.

-----------------------------------------------------------------------------------------------------------
controller.m, trajectory_generator.m: You have alredy done in phase1 and phase2. If you still have problems with your controller, please contact TAs.

path_from_A_star.m: What you need to work with. The code which pre-processes obstacle into a grid map has been given. You need to generate path points using A* and feed the points into trajectory_generator.m

Contact TAs with any questions you may have.

yzhangec@connect.ust.hk
xluaj@connect.ust.hk

