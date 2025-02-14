
What are problem(s) that could occur, when...

1. The lookahead distance is too large?
- In this code's implementation, the robot iterates an index to search for a pose in the global_plan_.poses array that is at least desired_lookahead_dist_ away from the closest pose to the robot. 

- If it cannot find this pose, which is one of the possibility when the look ahead distance is too large, the index would be incremented to the last index of the array, which is global_plan_.poses.size() - 1, which corresponds to the goal_pose. 

- Even when the robot can find the required pose in global_plan_.poses, due to the look ahead distance being large, only a few of the poses in the global_plan would be chosen to track. With the current navigation, the trajectory is relatively straight and so there is no change in the robot's behavior. In navigation tasks that requires the robot to move in curved trajectory, however, a large look ahead distance would suggest that the generated global_path is smoothen greatly, and so the robot cannot traverse tight areas. 

2. The lookahead distance is too small?
- If the lookahead distance is too small, when the robot searches for a pose to track in the global_plan_.poses array, the chosen pose would be very close to where the robot is currently at. At the moment, since the path is relatively straight, the robot's behavior remains the same. In navigation tasks that requires the robot to move in curved trajectory, however, it is expected that the small look ahead distance would result in oscillating movement of the robot. 

3. The linear velocity is too large?
- If the linear velocity is too large, the angular velocity is also large as <code> angular_vel = linear_vel * curvature </code>
- Because of this, the turning radius of the robot is large, and so the robot cannot navigate in tight corners. 
- Additionally, because pure pursuit relies on a curvature-based movement, a high linear velocity may cause the robot to overshoot the path, because the steering response (with look ahead point) may not be as fast to correct path overshooting.

4. The lookahead point is to the left or right of the robot, such that $y' \approx 0$?

- If the lookahead point is to the left or right of the robot, $y' \approx 0$.

- This means that $x' \approx $ $\Delta$ x * cos($\phi$<sub>r</sub>), and $y' \approx $ 
-$\Delta$ x * sin($\phi$<sub>r</sub>).

- Therefore, curvature = (2 * -$\Delta$ x * sin($\phi$<sub>r</sub>)) / (($\Delta$ x)<sup>2</sup> * cos<sup>2</sup>($\phi$<sub>r</sub>) + ($\Delta$ x)<sup>2</sup> * sin<sup>2</sup>($\phi$<sub>r</sub>)) .

- Curvature = (2 * -($\Delta$ x) * sin($\phi$<sub>r</sub>)) / ($\Delta$ x)<sup>2</sup> = (-2 * sin($\phi$<sub>r</sub>) / $\Delta$ x).

- From the above equation, the range of sin($\phi$<sub>r</sub>), and the value of $\Delta$ x (which is expected to be not too small, due to the reason in question 2), the calculated curvature will be small.

- Hence, the robot will move in a relatively straight line with little turning. 

5. The lookahead point is behind the robot, such that $x' < 0$?
- If the lookahead point is behind the robot, the robot would turn around to track that pose.
- Because of this, the robot movement may osciallate between tracking the goal pose and tracking the look ahead point at the back.