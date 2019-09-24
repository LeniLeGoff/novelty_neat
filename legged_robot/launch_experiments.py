#! /usr/bin/env python

import subprocess as sp

def launch(cmd) :
  process = sp.Popen(cmd, shell=True, stdout=sp.PIPE)
  process.wait()
  print(cmd,process.returncode)



# launch("./build/exp/novelty_neat/legged_robot_novelty_neat_three_legs_3dof_long_run")
# launch("./build/exp/novelty_neat/legged_robot_novelty_neat_three_legs_3dof_large_pop")
# launch("./build/exp/novelty_neat/legged_robot_novelty_neat_three_legs_3dof_long_eval")
# launch("./build/exp/novelty_neat/legged_robot_novelty_neat_three_legs_3dof_long_run_long_eval")
# launch("./build/exp/novelty_neat/legged_robot_novelty_neat_three_legs_3dof_long_run_large_pop")
# launch("./build/exp/novelty_neat/legged_robot_novelty_neat_three_legs_3dof_large_pop_long_eval")
# launch("./build/exp/novelty_neat/legged_robot_novelty_neat_three_legs_3dof_long_run_long_eval_large_pop")

for i in range(0,10) :
  launch("./build/exp/novelty_neat/legged_robot_novelty_neat_three_legs_4dof_long_eval")

