import math
import tkinter as tk
import keyboard


# 3D vector operations
def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_scalar(a, x):
    return (x * a[0], x * a[1], x * a[2])


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_mag(a):
    return (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]) ** (0.5)


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(a, b):
    return (
        a[1] * b[2] - b[1] * a[2],
        a[2] * b[0] - b[2] * a[0],
        a[0] * b[1] - b[0] * a[1],
    )


def vec_unit(a):
    mag = vec_mag(a)
    if mag == 0:
        return (0, 0, 0)
    return vec_scalar(a, 1 / vec_mag(a))


# 3D standard unit vectors
v_i = (1, 0, 0)
v_j = (0, 1, 0)
v_k = (0, 0, 1)


# The 3D projection/camera object
# This uses rasterisation
class projection:
    def __init__(self, k, width, height, maxdist=1000):
        """
        k,width and height specifies the projection pyramid
        k is the distance of apex from the base of the rect pyramid with dimensions width and height
        """
        self.k = k
        self.width = width
        self.height = height
        self.maxdist = maxdist  # This is to be used in object drawing functions
        self.campos = [0, 0, 0]

        # The below angles(polar angles) is to specify normal in terms of view angles,for controlling
        # alpha is the angle made by normal with y axis, varies in 0,pi
        # beta is the angle made by projection of normal on XZ plane,with Z, varying between -pi and pi
        self.alpha = math.pi / 2
        self.beta = 0
        self.set_normal(self.alpha, self.beta)

    def set_normal(self, alpha, beta):
        # For setting the projection normal
        # It creates an orthogonal triad for projection
        self.alpha, self.beta = alpha, beta
        self.v_n = (
            math.sin(alpha) * math.sin(beta),
            math.cos(alpha),
            math.sin(alpha) * math.cos(beta),
        )

        self.v_a = (math.cos(beta), 0, -math.sin(beta))
        self.v_b = vec_cross(self.v_n, self.v_a)

    def proj_point(self, v_r):
        """
        Creates the projection coordinates of any point
        Projection coordinates refers to the position of the image of the point
        in the base rectange with respect to the orthonormal triad
        Returns None if point is behind
        """
        v_to_r = vec_sub(v_r, self.campos)
        mycoef = self.k / vec_dot(v_to_r, self.v_n)
        if mycoef < 0:  # Point is behind
            return None
        v_rp = vec_add(vec_scalar(v_to_r, mycoef), self.campos)
        scr_x = vec_dot(self.v_a, v_rp)
        scr_y = vec_dot(self.v_b, v_rp)
        return scr_x, scr_y


# 3D Model
class Face:
    # Face of a 3D model
    def __init__(self):
        # This will store vectors representing points(cyclic, to form a polygon)
        self.points = []
        self.initpoints = []
        # Points will also include the translational and rotational components of the model
        # initPoints will not include such components
        self.outline = "#000000"
        self.color = "#FFFFFF"


class Model:
    # The model, a collection of faces
    def __init__(self):
        self.faces = []  # list of faces
        self.pos = [0, 0, 0]
        self.ang = [0, 0, 0]

    def add_face(self, points, outline, color):
        # To add a face.Use during model creation
        face = Face()
        face.points = [list(point) for point in points]
        face.initpoints = [tuple(point) for point in points]
        face.outline = outline
        face.color = color
        self.faces.append(face)

    def translate(self, topos):
        # Translate the model to the position specified by 'topos'
        for face in self.faces:
            for point in face.points:
                point[0] = point[0] + topos[0] - self.pos[0]
                point[1] = point[1] + topos[1] - self.pos[1]
                point[2] = point[2] + topos[2] - self.pos[2]
        self.pos[0], self.pos[1], self.pos[2] = topos[0], topos[1], topos[2]

    def rotate(self, angle):
        """
        Rotate the model to the angle specified by 'angle'
        angle[0] represents rotation along x_axis wrt the model's origin
        angle[1] and angle[2] similarly for y_axis and z_axis
        """
        self.ang[0], self.ang[1], self.ang[2] = angle[0], angle[1], angle[2]
        for face in self.faces:
            for p_i in range(len(face.points)):
                initPoint = face.initpoints[p_i]
                point = face.points[p_i]
                # x rotation
                point[0] = initPoint[0]
                point[1] = initPoint[1] * math.cos(self.ang[0]) - initPoint[2] * math.sin(self.ang[0])
                point[2] = initPoint[2] * math.cos(self.ang[0]) + initPoint[1] * math.sin(self.ang[0])
                # y rotation
                x = point[0] * math.cos(self.ang[1]) + point[2] * math.sin(self.ang[1])
                z = point[2] * math.cos(self.ang[1]) - point[0] * math.sin(self.ang[1])
                point[0] = x
                point[2] = z
                # z rotation
                x = point[0] * math.cos(self.ang[2]) - point[1] * math.sin(self.ang[2])
                y = point[1] * math.cos(self.ang[2]) + point[0] * math.sin(self.ang[2])
                point[0] = x
                point[1] = y
                # translation
                point[0], point[1], point[2] = (
                    point[0] + self.pos[0],
                    point[1] + self.pos[1],
                    point[2] + self.pos[2],
                )

    def sortfromcam(self, cam):
        """
        Sorts faces according to distance from cam
        for drawing priority
        """
        global CAMPOS
        CAMPOS = cam
        self.faces.sort(key=Facedist, reverse=True)


def Facedist(face):
    # This function is used by the sort method of list to sort faces according to the its distance from camera
    # Distance is calculated by considering the average point of all face points
    global CAMPOS
    avgpoint = [0, 0, 0]
    n = len(face.points)
    for point in face.points:
        avgpoint[0] += point[0]
        avgpoint[1] += point[1]
        avgpoint[2] += point[2]
    avgpoint[0] /= n
    avgpoint[1] /= n
    avgpoint[2] /= n
    return vec_mag(vec_sub(avgpoint, CAMPOS))


def model_copy(model):
    # Returns a copy of the model
    newmodel = Model()
    for face in model.faces:
        newmodel.add_face(face.initpoints, face.outline, face.color)
    return newmodel


# Implementation using tkinter canvas


class mycanvas:
    def __init__(self, root):
        self.canvas = tk.Canvas(root, width=500, height=400, background="#87CEEB")
        self.canvas.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)


class myobj_model:
    """
    An object which can be assigned a model
    Can be controlled by keyboard by setting ctrl to True
    It can be assigned a velocity if it is not controlled. Call step function to update position.
    Step function will also call the draw function.
    """

    def __init__(self, myid="MyOBJ"):
        self.x = 0
        self.y = 0
        self.z = 0
        self.ang = [0, 0, 0]
        self.xspd = 0
        self.yspd = 0
        self.zspd = 0
        self.canvas = None
        self.ctrl = False
        self.proj = None
        self.model = None
        self.myid = myid

        self.ktspd = 2  # Keyboard press translational speed
        self.krspd = math.pi / 180  # keyboard press rotational speed in rad/s

    def convert_to_canvas(self, canpos):
        """
        The projection object will return coordinates with respect to a projection origin with (0,0) at center
        But canvas has (0,0) at top left and y increases downwards in canvas.
        This function converts projection coordinates('canpos') to canvas coordinates
        """
        return (
            int(self.canvas["width"]) / 2 - canpos[0],
            int(self.canvas["height"]) / 2 - canpos[1],
        )

    def step(self):
        # Update position,calls draw
        self.x += self.xspd
        self.y += self.yspd
        self.z += self.zspd
        if self.canvas != None:
            self.draw(self.canvas, self.proj)

    def draw(self, canvas, proj):
        # Draws the model face by face
        self.canvas = canvas
        self.proj = proj
        # To clear previous rendering
        self.canvas.delete(self.myid)
        if self.model != None:
            # Check if it should be drawn
            v_camToObj = vec_sub((self.x, self.y, self.z), self.proj.campos)
            if (
                vec_dot(v_camToObj, self.proj.v_n) < 0
                or vec_mag(v_camToObj) > self.proj.maxdist
            ):
                # if obj is behind cam or far from it,no need to draw
                return
            # To set drawing priority of faces
            self.model.sortfromcam(proj.campos)
            for face in self.model.faces:
                canpoints = []
                for point in face.points:
                    canpoint = self.proj.proj_point(point)
                    if canpoint == None:
                        break
                    canpoint = self.convert_to_canvas(canpoint)
                    canpoints.append(canpoint[0])
                    canpoints.append(canpoint[1])
                else:
                    # Draw only if it didnt break
                    self.canvas.create_polygon(
                        canpoints, fill=face.color, outline=face.outline, tag=self.myid
                    )

    def kresp(self):
        # For keyboard control
        t_trig = False
        r_trig = False
        if self.ctrl == True:
            if keyboard.is_pressed("left arrow"):
                self.x += self.ktspd
                t_trig = True
            elif keyboard.is_pressed("right arrow"):
                self.x -= self.ktspd
                t_trig = True
            elif keyboard.is_pressed("up arrow"):
                self.z += self.ktspd
                t_trig = True
            elif keyboard.is_pressed("down arrow"):
                self.z -= self.ktspd
                t_trig = True
            elif keyboard.is_pressed("q"):
                self.y += self.ktspd
                t_trig = True
            elif keyboard.is_pressed("e"):
                self.y -= self.ktspd
                t_trig = True
            elif keyboard.is_pressed("r"):
                self.ang[0] += self.krspd
                r_trig = True
            elif keyboard.is_pressed("f"):
                self.ang[0] -= self.krspd
                r_trig = True
            elif keyboard.is_pressed("t"):
                self.ang[1] += self.krspd
                r_trig = True
            elif keyboard.is_pressed("g"):
                self.ang[1] -= self.krspd
                r_trig = True
            elif keyboard.is_pressed("y"):
                self.ang[2] += self.krspd
                r_trig = True
            elif keyboard.is_pressed("h"):
                self.ang[2] -= self.krspd
                r_trig = True
            if self.model != None:
                if t_trig == True:
                    self.model.translate((self.x, self.y, self.z))
                if r_trig == True:
                    self.model.rotate(self.ang)


# Model Making
# Here a cube is made, the main cube which will be controlled
mymodel = Model()
mymodel.add_face(
    ([10, 10, 10], [10, 10, -10], [10, -10, -10], [10, -10, 10]),
    "#0000FF",
    "#FF0000"
)  # +YZ
mymodel.add_face(
    ([10, 10, 10], [10, -10, 10], [-10, -10, 10], [-10, 10, 10]),
    "#0000FF",
    "#FF0000"
)  # XY+
mymodel.add_face(
    ([10, 10, 10], [10, 10, -10], [-10, 10, -10], [-10, 10, 10]),
    "#0000FF",
    "#FF0000"
)  # X+Z
mymodel.add_face(
    ([-10, 10, 10], [-10, 10, -10], [-10, -10, -10], [-10, -10, 10]),
    "#0000FF",
    "#FF0000"
)  # -YZ
mymodel.add_face(
    ([10, 10, -10], [10, -10, -10], [-10, -10, -10], [-10, 10, -10]),
    "#0000FF",
    "#FF0000"
)  # XY-
mymodel.add_face(
    ([10, -10, 10], [10, -10, -10], [-10, -10, -10], [-10, -10, 10]),
    "#0000FF",
    "#FF0000"
)  # X-Z

# Models for more static cubes
mymodel1 = model_copy(mymodel)
mymodel2 = model_copy(mymodel)
mymodel3 = model_copy(mymodel)
mymodel4 = model_copy(mymodel)
mymodel5 = model_copy(mymodel)


# Functions to be called in the tk loop
def Objdist(obj):
    # distance of object from cam
    # used to prioritize drawing
    global CAMPOS
    return vec_mag(vec_sub((obj.x, obj.y, obj.z), CAMPOS))


def step():
    global CAMPOS
    CAMPOS = proj.campos
    dist_sorted_objs = sorted(objs, key=Objdist, reverse=True)
    for obj in dist_sorted_objs:
        root.after(1000 // FPS, obj.step)
        root.after(1000 // FPS, obj.kresp)

    coord_text.set(f"Coords: {objs[0].x},{objs[0].y},{objs[0].z}")
    ang_text.set(f"Angles: {objs[0].ang[0]},{objs[0].ang[1]},{objs[0].ang[2]}")
    # This is to control camera angle
    camtrig = False
    alpha = proj.alpha
    beta = proj.beta
    if keyboard.is_pressed("w"):
        alpha -= math.pi / 180
        camtrig = True
    elif keyboard.is_pressed("s"):
        alpha += math.pi / 180
        camtrig = True
    elif keyboard.is_pressed("a"):
        beta += math.pi / 180
        camtrig = True
    elif keyboard.is_pressed("d"):
        beta -= math.pi / 180
        camtrig = True
    if camtrig == True:
        # To prevent it from going out of bounds
        if alpha > math.pi:
            alpha = math.pi
        elif alpha < 0:
            alpha = 0
        if beta > math.pi:
            beta = -2 * math.pi + beta
        elif beta < -math.pi:
            beta = 2 * math.pi + beta
        proj.set_normal(alpha, beta)
        cam_ang_text.set(f"Camera Angles: {alpha},{beta}")
    root.after(1000 // FPS, step)


def draw():
    # For initial drawing
    # Not an important function,mostly initializing values
    global CAMPOS
    CAMPOS = proj.campos
    dist_sorted_objs = sorted(objs, key=Objdist, reverse=True)
    for obj in dist_sorted_objs:
        obj.model.translate((obj.x, obj.y, obj.z))
        obj.draw(screen.canvas, proj)


# Initialising
FPS = 30
root = tk.Tk()
screen = mycanvas(root)
proj = projection(1000, 100, 100)

# To show main object coords and rotation angles and cam angles
coord_text = tk.StringVar()
coord_text.set("Coords: what")
tk.Label(root, textvariable=coord_text).grid(column=0, row=1)

ang_text = tk.StringVar()
ang_text.set("Angles: what")
tk.Label(root, textvariable=ang_text).grid(column=0, row=2)

cam_ang_text = tk.StringVar()
cam_ang_text.set(f"Camera Angles: {proj.alpha},{proj.beta}")
tk.Label(root, textvariable=cam_ang_text).grid(column=0, row=3)

# Making the objects
objs = [
    myobj_model(),
    myobj_model(myid="MyTest1"),
    myobj_model(myid="MyTest2"),
    myobj_model(myid="MyTest3"),
    myobj_model(myid="MyTest4"),
    myobj_model(myid="MyTest5"),
]
objs[0].x, objs[0].y, objs[0].z, objs[0].ctrl = 0, 0, 100, True
objs[0].model = mymodel

objs[1].x, objs[1].y, objs[1].z = 0, 100, 0
objs[1].model = mymodel1

objs[2].x, objs[2].y, objs[2].z = 100, 0, 0
objs[2].model = mymodel2

objs[3].x, objs[3].y, objs[3].z = 0, -100, 0
objs[3].model = mymodel3

objs[4].x, objs[4].y, objs[4].z = -100, 0, 0
objs[4].model = mymodel4

objs[5].x, objs[5].y, objs[5].z = 0, 0, -100
objs[5].model = mymodel5

# Start
draw()
step()
root.mainloop()
