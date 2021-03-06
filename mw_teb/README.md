## modifications
Under the controller/config/controler_action_server_params.yaml 

under the controllers add this:    
    - {name: teb_local_planner, type: "teb_local_planner/TebLocalPlannerROS"}  
    
under the controller_executions add this:
    - {name: teb_local_planner_execution, type: "teb_local_planner/TebLocalPlannerROSExecution"}
## Installation

catkin build mw_teb
