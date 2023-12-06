# 3D Projection in Python
The '3d_projection.py' is an attempt at achieving a 3D projection of a 3D space in Python. 

## Projection
It uses rasterisation, i.e. for a point in 3D space, it finds its position when it is seen by a camera, facing in a direction given by a normal vector. The code uses vector algebra to do the same. If we draw a ray from the point to the camera, it will intersect at a point in the base of an imaginary projection pyramid. This projection pyramid has a rectangular base and at its apex is the camera. The direction of the vector joining the apex to the center of the rectangular base is the normal vector. The result of the math is the coordinates of the intersection point in terms of two vectors, each describing the directions of the perpendicular edges of the rectangular base. The coordinates can be used to show the 3D space in a 2D surface. The 2D surface used here is a tkinter canvas.

## Models
A model is considered as a collection of faces. Each face is taken as a set of points joining to form a polygon, has a fill colour and an outline colour.  
The points of each face is projected to the tkinter canvas and a polygon is drawn joining them. The faces of a model are given different drawing priorities according to the distance of the centroid of the face from the camera.

## What the code does
The '3d_projection.py' uses all the above and implements a projection of a 3D space with 6 cubes, one of which is controllable by keyboard, to translate, or to rotate about x,y,z axes with respect to the object. All other cubes are static but can be given velocities in the code. I have tried to provide documentations in the code, wherever possible.

## Drawbacks of the method
* The code should blow up in terms of time if the model is complex.
* The drawing priority of different models according to depth is based on a logic that sorts the models according to the distance of the model origin from camera. But this is not a proper logic since different parts of a model should have different drawing priorities. In fact, different parts of a face will itself have different drawing priorites.
* Like mentioned before, the position of a face is taken as the position of the centroid of all points in the face in dept calculation. This is also used to identify if an object lies behind the camera. So in case there is some object that partly lies in front, and partly behind, say for examply ground or a side wall, but in centroid position lies behind the cam, it won't be drawn. Such large objects will have wierd drawing priorities too compared to other objects.
* This method completely ignores light and shading.

## How to run the code
This code imports tkinter, math and keyboard modules. Tkinter and math come with Python distributions. Keyboard module can be installed with pip.
Just run the code, you should see the controllable cube right in front. The controls are as follows.

* Up and Down cursor keys to increase or decrease 'z' position of the cube
* Left and Right cursor keys to increase or decrease 'x' position of the cube
* Q and E keys to increase or decrease 'y' position of the cube
* R and F keys to rotate the cube along its X-axis
* T and G keys to rotate the cube along its Y-axis
* Y and H keys to rotate the cube along its Z-axis
* WASD keys to control the camera view direction

The cube controls are set up in the myobj_model class. The camera controls are set in the common step function which is called by tkinter's main loop.