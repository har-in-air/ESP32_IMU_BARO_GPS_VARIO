# -*- coding: utf-8 -*-

import math # import radians, degrees, sin, cos, atan2, asin, acos, pi, sqrt
import numpy as np
from time import sleep


"""
 Simulation of a rotating 3D Cube
 Developed by Leonel Machava <leonelmachava@gmail.com>

 http://codeNtronix.com
"""
import sys, math, pygame
from operator import itemgetter

class Point3D:
    def __init__(self, x = 0, y = 0, z = 0):
        self.x, self.y, self.z = float(x), float(y), float(z)
 
    def rotateQ(self, q):
        v = Rotate(q, Vector(self.x, self.y, self.z))
        return Point3D(-v.y, v.z, v.x)
      
    def rotateX(self, angle):
        """ Rotates the point around the X axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        y = self.y * cosa - self.z * sina
        z = self.y * sina + self.z * cosa
        return Point3D(self.x, y, z)
 
    def rotateY(self, angle):
        """ Rotates the point around the Y axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        z = self.z * cosa - self.x * sina
        x = self.z * sina + self.x * cosa
        return Point3D(x, self.y, z)
 
    def rotateZ(self, angle):
        """ Rotates the point around the Z axis by the given angle in degrees. """
        rad = angle * math.pi / 180
        cosa = math.cos(rad)
        sina = math.sin(rad)
        x = self.x * cosa - self.y * sina
        y = self.x * sina + self.y * cosa
        return Point3D(x, y, self.z)
 
    def project(self, win_width, win_height, fov, viewer_distance):
        """ Transforms this 3D point to 2D using a perspective projection. """
        factor = fov / (viewer_distance + self.z)
        x = self.x * factor + win_width / 2
        y = -self.y * factor + win_height / 2
        return Point3D(x, y, self.z)

class Simulation:
    def __init__(self, win_width = 720, win_height = 480):
        pygame.init()

        self.screen = pygame.display.set_mode((win_width, win_height))
        pygame.display.set_caption("IMU logger visualization")
        
        self.clock = pygame.time.Clock()

        self.vertices = [
            Point3D(-1.0,0.5,-0.2),
            Point3D(1.0,0.5,-0.2),
            Point3D(1.0,-0.5,-0.2),
            Point3D(-1.0,-0.5,-0.2),
            Point3D(-1.0,0.5,0.2),
            Point3D(1.0,0.5,0.2),
            Point3D(1.0,-0.5,0.2),
            Point3D(-1.0,-0.5,0.2)
        ]

        # Define the vertices that compose each of the 6 faces. These numbers are
        # indices to the vertices list defined above.
        self.faces  = [(0,1,2,3),(1,5,6,2),(5,4,7,6),(4,0,3,7),(0,4,5,1),(3,2,6,7)]

        # Define colors for each face
        self.colors = [(255,0,0), # R
                       (255,0,255),
                       (0,255,0), # G
                       (0,0,255), # B
                       (0,255,255),
                       (255,255,0)]
                    
    def step(self, att_q, msg):
 #       """ Main Loop """
#        while 1:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self.clock.tick(500)
            self.screen.fill((0,32,0))

            font=pygame.font.Font(None,30)
            scoretext=font.render(msg, 1,(255,255,255))
            self.screen.blit(scoretext, (0, 0))
   
            # It will hold transformed vertices.
            t = []
            
            for v in self.vertices:
                # Rotate the point around X axis, then around Y axis, and finally around Z axis.
                #r = v.rotateZ(angle[0]).rotateX(angle[1]).rotateY(angle[2])
                r = v.rotateQ(att_q)
                # Transform the point from 3D to 2D
                p = r.project(self.screen.get_width(), self.screen.get_height(), self.screen.get_height()*0.75, 4)
                # Put the point in the list of transformed vertices
                t.append(p)

            # Calculate the average Z values of each face.
            avg_z = []
            i = 0
            for f in self.faces:
                z = (t[f[0]].z + t[f[1]].z + t[f[2]].z + t[f[3]].z) / 4.0
                avg_z.append([i,z])
                i = i + 1

            # Draw the faces using the Painter's algorithm:
            # Distant faces are drawn before the closer ones.
            for tmp in sorted(avg_z,key=itemgetter(1),reverse=True):
                face_index = tmp[0]
                f = self.faces[face_index]
                pointlist = [(t[f[0]].x, t[f[0]].y), (t[f[1]].x, t[f[1]].y),
                             (t[f[1]].x, t[f[1]].y), (t[f[2]].x, t[f[2]].y),
                             (t[f[2]].x, t[f[2]].y), (t[f[3]].x, t[f[3]].y),
                             (t[f[3]].x, t[f[3]].y), (t[f[0]].x, t[f[0]].y)]
                pygame.draw.polygon(self.screen,self.colors[face_index],pointlist)
                           
            pygame.display.flip()











# pythong port of https://github.com/daPhoosa/Math3D/blob/master/Math3D.h

def radians(a): return np.float32(math.radians(a))
def degrees(a): return np.float32(math.degrees(a))
def sin(a): return np.float32(math.sin(a))
def cos(a): return np.float32(math.cos(a))
def atan2(a,b): return np.float32(math.atan2(a,b))
def asin(a): return np.float32(math.asin(a))
def acos(a): return np.float32(math.acos(a))
def pi(): return np.float32(math.pi)
def sqrt(a): return np.float32(math.sqrt(a))

class Quat:
  def __init__(self):
    self.w = np.float32(1.0)
    self.x = np.float32(0.0)
    self.y = np.float32(0.0)
    self.z = np.float32(0.0)
    
  def Vec(self):
    return Vector(self.x, self.y, self.z)
    
  def __str__(self):
    return "[%+0.2e %+0.2ei %+0.2ej %+0.2ek]" % (self.w, self.x, self.y, self.z)
  def __repr__(self):
    return __str__(self)   

class Vec3:
  def __init__(self):
    self.x = np.float32(0.0)
    self.y = np.float32(0.0)
    self.z = np.float32(0.0)
  def __str__(self):
    return "(%+0.2e %+0.2e %+0.2e)" % (self.x, self.y, self.z)
  def __repr__(self):
    return __str__(self)   
      
def MulQQ(a, b):
  assert isinstance(a, Quat)
  assert isinstance(b, Quat)
  r = Quat()
  r.x = a.w * b.x + a.z * b.y - a.y * b.z + a.x * b.w
  r.y = a.w * b.y + a.x * b.z + a.y * b.w - a.z * b.x
  r.z = a.y * b.x - a.x * b.y + a.w * b.z + a.z * b.w
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
  return r
  
def MulQV(a, b):
  assert isinstance(a, Quat)
  assert isinstance(b, Vec3)
  r = Quat()
  r.x =  a.w * b.x + a.z * b.y - a.y * b.z  
  r.y =  a.w * b.y + a.x * b.z - a.z * b.x
  r.z =  a.y * b.x - a.x * b.y + a.w * b.z
  r.w = -a.x * b.x - a.y * b.y - a.z * b.z
  return r

def MulQF(a, b): # multiply: quat * float
  assert isinstance(a, Quat)
  b = np.float32(b)
  r = Quat()
  r.w = a.w * b
  r.x = a.x * b
  r.y = a.y * b
  r.z = a.z * b
  return r

def MulFQ(a, b): # multiply: float * quat
  return MulQF(b, a)


def MulVF(a, b): # multiply: vec3 * float
  assert isinstance(a, Vec3)
  b = np.float32(b)
  r = Vec3()
  r.x = a.x * b
  r.y = a.y * b
  r.z = a.z * b
  return r

def MulFV(a, b): # multiply: float * vec3
  return MulVF(b, a)


def Mul(a, b):
  if isinstance(a, Quat) and isinstance(b, Quat):
    return MulQQ(a,b)
  elif isinstance(a, Quat) and isinstance(b, Vec3):
    return MulQV(a,b)
  elif isinstance(a, Quat):
    return MulQF(a,b)
  elif isinstance(b, Quat):
    return MulFQ(a,b)
  elif isinstance(a, Vec3):
    return MulVF(a,b)
  elif isinstance(b, Vec3):
    return MulFV(a,b)
  else:
    raise exception

def SumQQ(a, b):
  assert isinstance(a, Quat)
  assert isinstance(b, Quat)
  r = Quat()
  r.w = a.w + b.w
  r.x = a.x + b.x
  r.y = a.y + b.y
  r.z = a.z + b.z
  return r

def SumVV(a, b):
  assert isinstance(a, Vec3)
  assert isinstance(b, Vec3)
  r = Vec3()
  r.x = a.x + b.x
  r.y = a.y + b.y
  r.z = a.z + b.z
  return r

def Sum(a,b):
  if isinstance(a, Quat) and isinstance(b, Quat):
    return SumQQ(a,b)
  if isinstance(a, Vec3) and isinstance(b, Vec3):
    return SumVV(a,b)
  else:
    raise exception

def SubVV(a, b):
  assert isinstance(a, Vec3)
  assert isinstance(b, Vec3)
  r = Vec3()
  r.x = a.x - b.x
  r.y = a.y - b.y
  r.z = a.z - b.z
  return r

def Sub(a,b):
  if isinstance(a, Vec3) and isinstance(b, Vec3):
    return SubVV(a,b)
  else:
    raise exception

def CrossVV(a, b): # cross product of 3D vectors 
  assert isinstance(a, Vec3)
  assert isinstance(b, Vec3)
  r = Vec3()
  r.x = a.y * b.z - a.z * b.y;
  r.y = a.z * b.x - a.x * b.z;
  r.z = a.x * b.y - a.y * b.x;
  return r

def CrossQQ(a, b): # cross product of quaternions
  assert isinstance(a, Quat)
  assert isinstance(b, Quat)
  r = Quat()
  r.w = np.float32(0)
  r.x = a.y * b.z - a.z * b.y;
  r.y = a.z * b.x - a.x * b.z;
  r.z = a.x * b.y - a.y * b.x;
  return r
  
def Cross(a,b):
  if isinstance(a, Vec3) and isinstance(b, Vec3):
    return CrossVV(a,b)
  if isinstance(a, Quat) and isinstance(b, Quat):
    return CrossQQ(a,b)
  else:
    raise exception

def DotQQ(a, b):
  assert isinstance(a, Quat)
  assert isinstance(b, Quat)
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z

def DotVV(a, b):
  assert isinstance(a, Vec3)
  assert isinstance(b, Vec3)
  return a.x * b.x + a.y * b.y + a.z * b.z
  
def Dot(a,b):
  if isinstance(a, Vec3) and isinstance(b, Vec3):
    return DotVV(a,b)
  elif isinstance(a, Quat) and isinstance(b, Quat):
    return DotQQ(a,b)
  else:
    raise exception
  
def MagQ(a):
  assert isinstance(a, Quat)
  return sqrt(a.w*a.w + a.x*a.x + a.y*a.y + a.z*a.z)

def MagV(a):
  assert isinstance(a, Vec3)
  return sqrt(a.x*a.x + a.y*a.y + a.z*a.z)

def Mag(a):
  if isinstance(a, Quat):
    return MagQ(a)
  elif isinstance(a, Vec3):
    return MagV(a)
  else:
    raise exception

def NormQ(a):
  assert isinstance(a, Quat)
  return Mul(a, 1/Mag(a))

def NormV(a):
  assert isinstance(a, Vec3)
  return Mul(a, 1/Mag(a))

def Norm(a):
  if isinstance(a, Quat):
    return NormQ(a)
  elif isinstance(a, Vec3):
    return NormV(a)
  else:
    raise exception

       
def Vector(x, y, z):
  r = Vec3()
  r.x = np.float32(x)
  r.y = np.float32(y)
  r.z = np.float32(z)
  return r
  
def ConjQ(a):
  assert isinstance(a, Quat)
  r = Quat()
  r.w = a.w  
  r.x = -a.x  
  r.y = -a.y  
  r.z = -a.z  
  return r
  
def Conj(a):
  if isinstance(a, Quat):
    return ConjQ(a)
  else:
    raise exception

  
def Vector2Quat(a): # [0,Vec3] to Quat
  assert isinstance(a, Vec3)
  r = Quat()
  r.w = np.float32(0.0)
  r.x = a.x
  r.y = a.y
  r.z = a.z
  return r

def Quat2Vector(a): # Quat to Vec3
  assert isinstance(a, Quat)
  r = Vec3()
  r.x = a.x
  r.y = a.y
  r.z = a.z
  return r

  
def RotateVQ(v, q): # Vector rotated by a Quaternion (matches V^ = V * Matrix)
  assert isinstance(v, Vec3)
  assert isinstance(q, Quat)
  # v + 2*r X (r X v + q.w*v) -- https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons
  r = Vec3()    # vector r is the three imaginary coefficients of quaternion q -- future test: make static to increase speed
  r.x = -q.x    # reverse signs to change direction of rotation
  r.y = -q.y
  r.z = -q.z
  return Sum(v, Cross(Sum(r, r), Sum(Cross(r, v), Mul(q.w, v)))) 

def RotateQV(q, v): # Vector rotated by a Quaternion (matches V^ = Matrix * V)
  assert isinstance(q, Quat)
  assert isinstance(v, Vec3)
  r = Vec3() # v + 2*r X (r X v + q.w*v) -- https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#Performance_comparisons
  r.x = q.x
  r.y = q.y
  r.z = q.z
  return Sum(v, Cross(Sum(r, r), Sum(Cross(r, v), Mul(q.w, v))))    

def Rotate(a,b):
  if isinstance(a, Vec3) and isinstance(b, Quat):
    return RotateVQ(a,b)
  elif isinstance(a, Quat) and isinstance(b, Vec3):
    return RotateQV(a,b)
  else:
    raise exception

def QuaternionS(w): # (angle vector[rad]) -- Small angle approximation
  assert isinstance(w, Vec3)
  
  r = Quat()
  r.x  = w.x / np.float32(2.0)
  r.y  = w.y / np.float32(2.0)
  r.z  = w.z / np.float32(2.0)
  r.w  = np.float32(1.0) - np.float32(0.5) * (r.x * r.x + r.y * r.y + r.z * r.z)
  return r 

def Quaternion(a): # (angle vector[rad], )  --Large Rotation Quaternion
  assert isinstance(a, Vec3)
  
  vMag = Mag(a)
  if vMag == 0:
    r = Quat()
    r.w = np.float32(1.0)
    r.x = np.float32(0.0)
    r.y = np.float32(0.0)
    r.z = np.float32(0.0)
    return r
    
  theta_2 = vMag * np.float32(0.5)            # rotation angle divided by 2
  Sin_Mag = sin(theta_2) / vMag   # computation minimization
  
  r = Quat()
  r.x = a.x * Sin_Mag
  r.y = a.y * Sin_Mag
  r.z = a.z * Sin_Mag
  r.w = cos(theta_2)
  return r

def QuaternionRPY(r,p,y): # rpy vector[deg]
  cr = cos(radians(r)/np.float32(2))
  cp = cos(radians(p)/np.float32(2))
  cy = cos(radians(y)/np.float32(2))
  sr = sin(radians(r)/np.float32(2))
  sp = sin(radians(p)/np.float32(2))
  sy = sin(radians(y)/np.float32(2))
  
  q = Quat()
  q.w = cr*cp*cy+sr*sp*sy
  q.x = sr*cp*cy-cr*sp*sy
  q.y = cr*sp*cy+sr*cp*sy
  q.z = cr*cp*sy-sr*sp*cy
  
  return q
    
def AngleVec(a): # convert from quaternion to angle vector[rad]
  assert isinstance(a, Quat)
  
  if(a.w < 0): a = Mul(a,-1)
  
  r = Vec3()
  th_2 = acos(max(-1, min(1, a.w)))
  
  if abs(th_2) > radians(0.005):
    s = sin(th_2)/(np.float32(2)*th_2)
    r.x = a.x/s
    r.y = a.y/s
    r.z = a.z/s
    #print a, th_2, s, r
  return r
      
  
class rpyQ:
  def __init__(self, q):
    assert isinstance(q, Quat)
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # Body Z-Y-X sequence
                                
    q0 = q.w
    q1 = q.x
    q2 = q.y                             
    q3 = q.z                             
            
    p0 = max(np.float32(-1.0), min(np.float32(1.0), 2*(q0*q2 - q3*q1)))
    p = asin(p0)     
    
    r0 = 2*(q0*q1 + q2*q3)        
    r1 = 1-2*(q1*q1+q2*q2)
    if r0 == 0 and 11 == 0:
      print "r: atan(0,0)!", q      
      r = 0
    else:
      r = atan2(r0, r1)
    
    y0 = 2*(q0*q3 + q1*q2)
    y1 = 1 - 2*(q2*q2 + q3*q3)
    if y0 == 0 and y1 == 0:
      print "y: atan(0,0)!", q      
      y = 0
    else:
      y = atan2(y0, y1)
            
    self.r = degrees(r)
    self.p = degrees(p)
    self.y = degrees(y)
                                 
  def __str__(self):
      return "(%+07.2f, %+07.2f, %+07.2f)" % (self.r, self.p, self.y)
  def __repr__(self):
    return __str__(self)   

def ArcQQ(a, b):
  assert isinstance(a, Quat)
  assert isinstance(b, Quat)

  q = Mul(b, Conj(a)) # order may be wrong, but this is what works for determining the bodyframe rotation needed to go from a->b
  th = np.float32(2)*atan2(Mag(q.Vec()), q.w)
  return th, q

def ArcVV(a, b): # returns the angle in radians and the angle vector
  assert isinstance(a, Vec3)
  assert isinstance(b, Vec3)
  ma = Mag(a)
  mb = Mag(b)
  
  if ma != 0 and mb != 0:
    th = np.float32(acos(min(max(-1, Dot(a,b)/(ma*mb)), 1)))
    r = Cross(a,b)
  else:
    th = np.float32(0)
    r = Vector(0.0, 0.0, 0.0)
  
  return th, r

def Arc(a,b):
  if isinstance(a, Quat) and isinstance(b, Quat):
    return ArcQQ(a,b)
  elif isinstance(a, Vec3) and isinstance(b, Vec3):
    return ArcVV(a,b)
  else:
    raise exception

def Degrees(a):
  if isinstance(a, Vec3):
    return Mul(180/radians(180), a)
  else:
    return np.float32(a)*180/radians(180)

def QuantizeF(a, m, r):
  q = float(m)/float(r)
  aq = int(a/q)*q
  return aq

def QuantizeV(a, m, r):
  return Vector(QuantizeF(a.x,m,r), QuantizeF(a.y,m,r), QuantizeF(a.z,m,r))

def Quantize(a, m, r):
  if isinstance(a, Vec3):
    return QuantizeV(a,m,r)
  elif isinstance(a, np.float32):
    return QuantizeF(a,m,r)
  else:
    raise exception

def get_params( line):
	params = []
	for item in line.split(','):
		params.append( float(item) )
	return params
  
        
def main():
	S = Simulation()
	with open("quatout.txt") as input_file:
		for line in input_file:
			line  = line.strip()
			pars  = get_params(line)
			qn   = Quat()
			qn.w = pars[0]
			qn.x = pars[1]
			qn.y = pars[2]
			qn.z = pars[3]
			yaw   = pars[4]
			pitch = pars[5]
			roll  = pars[6]
			S.step(qn, "Roll %+04.0f°  Pitch %+04.0f°  Yaw %+04.0f°" % (roll, pitch, yaw))
    
main()
