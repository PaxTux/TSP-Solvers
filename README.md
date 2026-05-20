# TSP Solvers
Solvers for various versions of Traveling Salesman Problem (TSP).
There is no randomness involved. If you provide the same input data, you will get the same result.

tsp_solver_points()
-------------------
Imagine traveling salesman has to visit a set of points, each exactly once.

* **routeStartPoint** specifies where ROUGHLY the route should start. You can think of this as salesmans initial location (at the beginning of the journey).
This parameter is mandatory. You may leave it blank, but then solver will set it to x=0;y=0.

* **routeEndPoint** specifies where ROUGHLY the route should end. You can think of this as where salesman wants to go after the journey is completed.
This parameter is optional. If not defined (let's say salesman doesn't care where to finish the journey), solver will pick one to ensure good route efficiency.

<img src="/images/tsp_solver_points_Eq10.png" width="800"/>
<img src="/images/tsp_solver_points_Eq45.png" width="800"/>
<img src="/images/tsp_solver_points_EqHx.png" width="800"/>
<img src="/images/tsp_solver_points_Pola.png" width="800"/>
<img src="/images/tsp_solver_points_Rand.png" width="800"/>

tsp_solver_tunnels()
-------------------
Imagine traveling salesman has to go through a set of tunnels (or portals), each exactly once.
Each tunnel has an entry point where salesman enters and an exit point where salesman pops out.
Entry point and exit point can be the same.

* **routeStartPoint** specifies where ROUGHLY the route should start. You can think of this as salesmans initial location (at the beginning of the journey).
This parameter is mandatory. You may leave it blank, but then solver will set it to x=0;y=0.

* **routeEndPoint** specifies where ROUGHLY the route should end. You can think of this as where salesman wants to go after the journey is completed.
This parameter is optional. If not defined (let's say salesman doesn't care where to finish the journey), solver will pick one to ensure good route efficiency.

* **allowFlipping** defines if salesman is allowed to travel through tunnels in the opposite direction. Setting this to True usually results in much shorter route.
