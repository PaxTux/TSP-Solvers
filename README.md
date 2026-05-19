# Travelling-Salesman-Problem-Solvers
Solvers for various versions of Traveling Salesman Problem (TSP)
There is no randomness involved. If you provide the same input data, you will get the same result.

---
All solvers have an option to set routeStartPoint and routeEndPoint.

routeStartPoint specifies where ROUGHLY the route should start. You can think of this as salesmans initial location (at the beginning of the journey).
This parameter is mandatory. You may leave it blank, but then solver will set it to x=0;y=0.

routeEndPoint specifies where ROUGHLY the route should end. You can think of this as where salesman wants to go after the journey is completed.
This parameter is optional. If not defined (let's say salesman doesn't care where to finish the journey), solver will pick one to ensure good route efficiency.

tsp_solver_points()
-------------------
This is a classical one...
Imagine traveling salesman has to visit all the points on the list, each exactly once.

Typical usecase:
