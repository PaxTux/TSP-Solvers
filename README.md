# TSP Solvers
Solvers for various versions of Traveling Salesman Problem (TSP).
There is no randomness involved. If you provide the same input data, you will get the same result.

tsp_solver_points()
-------------------
Imagine traveling salesman has to visit a set of points, each exactly once.

* **routeStartPoint** is a point that specifies where roughly the route should start. You can think of this as salesmans initial location (at the beginning of the journey). This parameter is mandatory. You may leave it blank, but then solver will set it to x=0;y=0.

  <img src="/images/tsp_solver_points_SP_Eq10.png" width="600"/>
  <img src="/images/tsp_solver_points_SP_Eq45.png" width="600"/>
  <img src="/images/tsp_solver_points_SP_EqHx.png" width="600"/>
  <img src="/images/tsp_solver_points_SP_Pola.png" width="600"/>
  <img src="/images/tsp_solver_points_SP_Rand.png" width="600"/>

* **routeEndPoint** is a point that specifies where roughly the route should end. You can think of this as where salesman wants to go after the journey is completed. This parameter is optional.

* **attractor** TBD



tsp_solver_tunnels()
-------------------
Imagine traveling salesman has to go through a set of tunnels (or portals), each exactly once.
Each tunnel has an entry point where salesman enters and an exit point where salesman pops out.
Entry point and exit point can be the same.


In examples below tunnels are drawn as (blue) straight lines but in reality tunnels can be much more complex. The function doesn't care how a tunnel looks like, it just cares about entry point and exit point.

* **routeStartPoint** is a point that specifies where roughly the route should start. You can think of this as salesmans initial location (at the beginning of the journey). This parameter is mandatory. You may leave it blank, but then solver will set it to x=0;y=0.

* **routeEndPoint** is a point that specifies where roughly the route should end. You can think of this as where salesman wants to go after the journey is completed. This parameter is optional.

* **allowFlipping** defines if salesman is allowed to travel through tunnels in the opposite direction. Setting this to True usually results in much shorter route.

(note that first and second example have same tunnels generated, but second example has allowFlipping set to False. Since all tunnels are oriented from left to right, salesman will fulfill this requirement in second example.)

<img src="/images/tsp_solver_tunnels_Hori_AF=T.png" width="800"/>
<img src="/images/tsp_solver_tunnels_Hori_AF=F.png" width="800"/>
<img src="/images/tsp_solver_tunnels_Rand.png" width="800"/>
