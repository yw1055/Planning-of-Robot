# Sampling-based planning with anytime weighted search
  ## A method is proposed to solve Robots' motion planning high dimensional continous space
  ##Combing anytime weighted search, i.g. RWA*, AWA*, ARA*, with sampling-based planning techonolgies,like RRT* and BIT*.
  
  ##This method is going to quickly find the first solution, and then get better and better solutions with additional time until it converges to an optimal solution    from a start state to the goal state.
  
##ARA* is an anytime weighted heuristic search adjusting its performence bound on sufficient search time.
  
  ##AWA* is an anytime weighted heuristic search finding a sequence of improved path solutions and eventually converges to an optimal solution.
  
  ##RWA* is an anytime weighted heuristic search restarting
