import math

# for use on local machine, vpython is too hard to install

    
def sqrt(x):
    return math.sqrt(x)

def answer(x, name):
    if (name == None):
        print(x);
        return x
    else:
        print(name, ": ", x)
        return x

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
    
    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z
    
    def cross(self, other):
        return vec(self.y * other.z - self.z * other.y, self.z * other.x - self.x * other.z, self.x * other.y - self.y * other.x)
    
    def __truediv__(self, other):
        if isinstance(other, (float, int)):
            return vec(self.x / other, self.y / other, self.z / other)
        else:
            print("Error: non-scalar division not supported")

    def __str__(self) -> str:
        return "<" + str(self.x) + ", " + str(self.y) + ", " + str(self.z) + ">"

left = vec(-1, 0, 0)
right = vec(1, 0, 0)
up = vec(0, 1, 0)
down = vec(0, -1, 0)
forward = vec(0, 0, 1)
backward = vec(0, 0, -1)
up_right = (up + right).hat()
up_left = (up + left).hat()
down_right = (down + right).hat()
down_left = (down + left).hat()


def point(v1, v2):
    return v2 - v1

def distance(v1, v2):
    return point(v1, v2).mag()

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

def micro(x):
    return x * 1e-6

def nano(x):
    return x * 1e-9

def pico(x):
    return x * 1e-12

mm = (1/1000)
cm = (1/100)
um = micro(1)

""" GEOMETRY """
# assumes we are finding the center of each segment
def semicircle(center, radius, segment_total, segment_index):
    if (segment_index > segment_total - 1):
        print("Error: segment index exceeds segment total")
        return None
    print("Assuming semicircle faces upwards")
    half_circumference = math.pi * radius
    segment_length = half_circumference / segment_total
    segment_offset = segment_length / 2
    angle = (segment_index * segment_length + segment_offset) / half_circumference
    return center + vec(radius * math.cos(angle), radius * math.sin(angle), 0)

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
anti_proton = Particle(-1.6e-19, vec(0, 0, 0))

# CONSTANTS
COULOMBS_CONST = 9.0e9
EP_NAUGHT = 8.85e-12
TWO_EP_0 = 2*EP_NAUGHT

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
    law = COULOMBS_CONST * q1 * q2 / mag(v12)**2 * hat(v12)
    print("Particle 1: ", p1)
    print("Particle 2: ", p2)
    print("Force of p1 on p2: ", law)
    return law

def coulombsLawParticle(p1, p2):
    return coulombsLaw(p1.charge, p1.position, p2.charge, p2.position)

def vel(p1, p2, dt):
    return (p2 - p1) / dt

def acc(p1, p2, dt):
    return vel(p1, p2, dt) / dt

def force(p1, p2, dt, m):
    return m * acc(p1, p2, dt)

def mom(p1, p2, dt, m):
    return m * vel(p1, p2, dt)

def ke(p1, p2, dt, m):
    return 0.5 * m * mag(vel(p1, p2, dt))**2

def work(p1, p2, dt, m):
    return force(p1, p2, dt, m) * mag(p2 - p1)

# this function lets you use None to solve for unknown parameter
# def electricForce(f_mag, charge, e_field):
#     if (charge): print("Charge: ", charge)
#     if (e_vec): print("Electric Field: ", e_vec)
#     if f_mag: print("Force magnitude: ", f_mag)

#     if (f_mag == None):
#         f_mag = charge * e_vec
#         print("Solving for force: ", f_mag)
#         return f_mag
#     elif (charge == None):
#         charge = f_mag / e_vec
#         print("Solving for charge: ", charge)
#         return charge
#     elif (e_vec == None):
#         e_vec = f_mag / charge
#         print("Solving for electric field: ", e_vec)
#         return e_vec
#     else:
#         print("Error: too many known parameters")
#         return None

def vectorMode(param1, param2, param3):
    v = 0;
    if (type(param1) == vec):
        v += 1
    if (type(param2) == vec):
        v += 1
    if (type(param3) == vec):
        v += 1
    
    if (v == 3):
        print("Vector mode");
        return True
    elif (v == 0):
        return False
    else:
        print("Error: one of these isn't a vector")
        return None
        

def electricForceMag(f, q, e):
    """
    Leave one blank and solve for the other:
    F = Q * E
    f_mag: force magnitude
    q: charge
    e_mag: electric field magnitude
    """
    if (vectorMode(f, q, e)):
        f = f.mag()
        e = e.mag()
        
    if (q == None):
        print("Solving for charge")
        return f / e
    elif (e == None):
        print("Solving for electric field")
        return f / q
    elif (f == None):
        print("Solving for force")
        return q * e
    else:
        print("Error: too many known parameters")
        return None
    
def electricForceVec(f_vec, q, e_vec):
    """
    Leave one blank and solve for the other:
    F = Q * E
    f_vec: force vector
    q: charge
    e_mag: electric field magnitude
    """
    if (f_vec): f_vec = f_vec.mag()
    if (e_vec): e_vec = e_vec.mag()
    return electricForceMag(f_vec, q, e_vec)

def electricField(charge, distance):
    # see if distance is a vector
    if (type(distance) == vec):
        print("Vector mode: vector should point from charge to point of interest")
        return COULOMBS_CONST * charge / distance.mag()**2 * distance.hat()
    else:
        return COULOMBS_CONST * charge / distance**2

def distanceFromChargeGivenElec(q, e_vec):
    if (q < 0):
        q = -q
    return sqrt(COULOMBS_CONST * q / e_vec.mag())

def forceOnCharge(q, e_vec):
    return abs(q) * e_vec


def dipoleMoment(q, s):
    """
    q: charge of dipole (scalar)
    s: separation of dipole (scalar)
    """
    return abs(q)*s

def dipoleParallelMag(p, r):
    """
    p: dipole moment (vec)
    r: distance from dipole (scalar)
    """
    if type(r) == vec:
        print("converting r to scalar")
        r = r.mag()
    return 2 * COULOMBS_CONST * p / (r**3)

def dipolePerpendicularMag(dp, r):
    """
    dp: dipole moment (scalar)
    r: distance from dipole (scalar)
    """
    return (1/2) * dipoleParallelMag(dp, r)


    """ 3D SHAPED CHARGES """

# sphere
    
def sphericalCharge(q, r):
    """
    q: charge of sphere (scalar)
    r: radius of sphere (scalar)
    """
    return 4 * math.pi * COULOMBS_CONST * q / r**2

# rod

def rodElecMagPerpendicular(q, l, r):
    """
    q: charge of rod (scalar)
    l: length of rod (scalar)
    r: distance from rod (scalar)
    """
    return COULOMBS_CONST * q / (r) * (1 / sqrt(r**2 + (l/2)**2))

def approxRodElecMagPerpendicular(q, l, r):
    """
    q: charge of rod (scalar)
    l: length of rod (scalar)
    r: distance from rod (scalar)
    """
    return COULOMBS_CONST * q/l * 2 / r

# ring

def ringElecMagOnAxis(q, r, radius):
    """
    q: charge of ring (scalar)
    r: perp distance from ring (scalar)
    radius: radius of ring (scalar)
    """
    return COULOMBS_CONST * q * r / (r**2 + radius**2)**(3/2)

# disk

def diskElecMagOnAxis(q, z, radius):
    """
    q: charge of disk (scalar)
    z: perp distance from disk (scalar)
    radius: radius of disk (scalar)
    """
    area = math.pi * radius**2
    return answer((q / area) / TWO_EP_0 * (1 - z / sqrt(z**2 + radius**2)),"diskElecMagOnAxis: ")

def diskElecMagOnAxisApprox(q, z, radius):
    """
    q: charge of disk (scalar)
    z: perp distance from disk (scalar)
    radius: radius of disk (scalar)
    """
    area = math.pi * radius**2
    return answer((q / area) / TWO_EP_0 * (1 - z / radius), "approx: ")

def diskElecMagOnAxisWTF(q, radius):
    """
    q: charge of disk (scalar)
    z: perp distance from disk (scalar)
    radius: radius of disk (scalar)
    """
    area = math.pi * radius**2
    return answer(q / area / TWO_EP_0, "WTF: ")



# a_pos = vec(3, 4, 0) * cm
# q1_pos = vec(0, 4, 0) * cm
# q2_pos = vec(0, 0, 0) * cm
# q3_pos = vec(3, 0, 0) * cm

# a_charge = proton.charge * 2
# a_mass = 6.646e-27
# q1_charge = micro(2)
# q2_charge = micro(9)
# q3_charge = micro(-3)

# q1_field = electricField(q1_charge, point(q1_pos, a_pos))
# q2_field = electricField(q2_charge, point(q2_pos, a_pos))
# q3_field = electricField(q3_charge, point(q3_pos, a_pos))

# net_field = q1_field + q2_field + q3_field

# net_force = forceOnCharge(a_charge, net_field)
# initial_accel = net_force / a_mass
# print("Acceleration: ", initial_accel)

