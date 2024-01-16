import math

# for use on local machine, vpython is too hard to install

    
def sqrt(x):
    return math.sqrt(x)

class vec:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "<" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ">"

    def mag(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)
    
    def hat(self):
        return vec(self.x / self.mag(), self.y / self.mag(), self.z / self.mag())
    
    def __add__(self, other):
        return vec(self.x + other.x, self.y + other.y, self.z + other.z)
    
    def __sub__(self, other):
        return vec(self.x - other.x, self.y - other.y, self.z - other.z)
    
    def __mul__(self, other):
        if isinstance(other, (float, int)):
            return vec(self.x * other, self.y * other, self.z * other)
        else:
            print("Error: non-scalar multiplication not supported")
    
    def __rmul__(self, other):
        return self.__mul__(other)

def mag(v):
    if (type(v) == vec):
        return v.mag()
    else:
        print("Error: mag() takes a vec object as an argument")

def hat(v):
    if (type(v) == vec):
        return v.hat()
    else:
        print("Error: hat() takes a vec object as an argument")

class Particle:
    def __init__(self, charge, position):
        self.charge = charge
        self.position = position

    def __str__(self):
        build = ""
        if (self.charge == 1.6e-19):
            build += "[Proton] "
        elif (self.charge == -1.6e-19):
            build += "[Electron] "
        elif (self.charge == 3.2e-19):
            build += "[Alpha] "

        return build + "Charge: " + str(self.charge) + " Position: " + str(self.position)

# Common particles
electron = Particle(-1.6e-19, vec(0, 0, 0))
proton = Particle(1.6e-19, vec(0, 0, 0))
alpha = Particle(3.2e-19, vec(0, 0, 0))


# CONSTANTS
COULOMBS_CONST = 9.0e9

""" 
    coulombsLaw(q1, p1, q2, p2)

    Returns and prints the force vector of particle 1 on particle 2

    q1: charge of particle 1
    p1: position of particle 1

    q2: charge of particle 2
    p2: position of particle 2

"""
def coulombsLaw(q1, p1, q2, p2):
    v12 = p2 - p1
    law = COULOMBS_CONST * q1 * q2 / (v12.mag())**2 * v12.hat()
    print("Particle 1: ", p1)
    print("Particle 2: ", p2)
    print("Force of p1 on p2: ", law)
    return law

coulombsLaw(proton.charge, proton.position, electron.charge, electron.position + vec(1, 1, 1))

# Path: tools.py
    