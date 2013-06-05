import math

class Vector:
	def __init__(self, data):
		self.data = None
		t = type(data)
		if t is list or t is tuple:
			self.data = data[:]
		elif t is int or t is float:
			self.data = [data]
		elif isinstance(data,Vector):
			self.data = data.data[:]
    
	def __repr__(self):
		return repr(self.data)  
    
	def __add__(self, other):
		v1 = self.data
		t = type(other)
		if t is list or t is tuple:
			v2 = other
		elif t is int or t is float:
			v2 = [other]*len(self.data)
		else:
			v2 = other.data
		return Vector([vv[0]+vv[1] for vv in zip(v1,v2)])
	
	def __sub__(self, other):
		v1 = self.data
		t = type(other)
		if t is list or t is tuple:
			v2 = other
		elif t is int or t is float:
			v2 = [other]*len(self.data)
		else:
			v2 = other.data
		return Vector([vv[0]-vv[1] for vv in zip(v1,v2)])

       	def __mul__(self, other):
		v1 = self.data
		t = type(other)
		if t is list or t is tuple:
			v2 = other
		elif t is int or t is float:
			v2 = [other]*len(self.data)
		else:
			v2 = other.data
		return Vector3([vv[0]*vv[1] for vv in zip(v1,v2)])

       	def __div__(self, other):
		v1 = self.data
		t = type(other)
		if t is list or t is tuple:
			v2 = other
		elif t is int or t is float:
			v2 = [other]*len(self.data)
		else:
			v2 = other.data
		return Vector3([vv[0]/vv[1] for vv in zip(v1,v2)])

	def dot(self, other):
		v1 = self.data
		t = type(other)
		if t is list or t is tuple:
			v2 = other
		else:
			v2 = other.data
		return sum([vv[0]*vv[1] for vv in zip(v1,v2)])


	def __getitem__(self, index):
		return self.data[index]

	def __len__(self):
		return len(self.data)

	def dist(self, other):
		return math.sqrt(self.dist_sq(other))

	def dist_sq(self, other):
		v1 = self.data
		t = type(other)
		if t is list or t is tuple:
			v2 = other
		else:
			v2 = other.data
		return sum([pow(vv[0]-vv[1],2) for vv in zip(v1,v2)])

	def norm(self):
		return math.sqrt(self.norm_sq())

	def norm_sq(self):
		return self.dot(self)
	
	def normalize(self):
		s = self.norm()
		if abs(s-1.0) < 1e-5:
			return
		s = 1. / s
		self.data= [ xi * s for xi in self.data]


class Vector2(Vector): 
	def __init__(self,*args):
		if len(args)==2:
			Vector.__init__(self,args)
		elif len(args)==1 and isinstance(args[0],(float,int)):
			Vector.__init__(self,[args[0]]*2)
		elif args==None:
			Vector.__init__(self,[0.]*2)
		else:
			Vector.__init__(self,*args)

	def cross(self, other):
		v1 = self.data
		t = type(other)
		if t is list or t is tuple:
			v2 = other
		else:
			v2 = other.data
		return v1[0]*v2[1]-v1[1]*v2[0]

class Vector3(Vector): 
	def __init__(self,*args):
		if len(args)==3:
			Vector.__init__(self,args)
		elif len(args)==1 and isinstance(args[0],(float,int)):
			Vector.__init__(self,[args[0]]*3)
		elif args==None:
			Vector.__init__(self,[0.]*3)
		else:
			Vector.__init__(self,*args)

	def cross(self, other):
		v1 = self.data
		t = type(other)
		if t is list or t is tuple:
			v2 = other
		else:
			v2 = other.data
		return Vector3([v1[1]*v2[2]-v1[2]*v2[1],
			       v1[2]*v2[0]-v1[0]*v2[2],
			       v1[0]*v2[1]-v1[1]*v2[0]] )
						

class Quaternion(object):
	def __init__(self, q=None):
		if q and len(q) == 4:
			self.q = q[:]
		else:
			self.q = [0 for i in range(4)]
		
	def __repr__(self):
		return str(self.q)
		
	def normalize(self):
		d = sum([qq*qq for qq in self.q])
		if d == 1.0:
			return
		d = math.sqrt(d)
		if abs(d-1.0) < 1e-5:
			return
		d = 1.0 / d
		for i in range(4):
			self.q[i] = self.q[i]*d
		
	def conj(self):
		w,x,y,z = self.q
		q = Quaternion()
		q.q = [w, -x, -y, -z]
		return q
		
	def set_axis_angle(self, axis, angle):
		w = math.cos(angle * 0.5)
		self.q[0] = w
		sinq = math.sin(angle * 0.5)
		axis = Vector3(axis)
		axis.normalize()
		axis = axis * sinq
		for i in range(3):
			self.q[i+1] = axis.data[i]
		self.normalize()
	
	def axis_angle(self):
		w,x,y,z = self.q
		angle = 2 * math.acos(w)
		d = math.sqrt(1 - (w*w))
		if d == 0:
			axis = Vector3([0.,1.,0.])
		else:
			d = 1. / d
			axis = Vector3([x*d,y*d,z*d])
		return (axis, angle)
		
	def set_point(self, point):
		self.q[0] = 0
		for i in range(3):
			self.q[i+1] = point.data[i]
			
	def point(self):
		if abs(self.q[0]) >= 1e-4:
			raise RuntimeError("Not a point: " + str(self))
		return Vector3(self.q[1:])
		
	def set_vector_rotation(self, startV, endV):
		startV = Vector3(startV)
		endV = Vector3(endV)
		startV.normalize()
		endV.normalize()
		
		cross = startV.cross(endV)
		dot = startV.dot(endV)
		
		if abs(dot+1.0) <= 1e-4:
			# Rotation of 180 degrees
			# Pick an axis at right angles to startV and angle of 180
			q = Quaternion()
			# KH: NOTE: there may be an error here when startV = -xVector!
			q.set_vector_rotation(Vector3([1.,0.,0.]), startV)
			axis = q*Vector3([0.,1.,0.])
			self.set_axis_angle(axis, math.pi)
		else:
			self.q[0] = 1+dot
			for i in range(3):
				self.q[i+1] = cross.data[i]
			
		self.normalize()

	# A straightforward but slow way of multiplying two quaternions		
#	def __mulold__(self, other):
#		w1 = self.q[0]
#		w2 = other.q[0]
#		v1 = Vector3(self.q[1:])
#		v2 = Vector3(other.q[1:])
#		
#		wr = w1*w2 - v1.dot(v2)
#		vr = v1.cross(v2) + (v2*w1) + (v1*w2)
#		
#		q = Quaternion()
#		q.q[0] = wr
#		for i in range(3):
#			q.q[i+1] = vr.data[i]
#		return q
		
	def __mul__(self, other):
		t = type(other)
		if t is type(self):
			q = Quaternion()
			w1,x1,y1,z1 = self.q
			w2,x2,y2,z2 = other.q
			q.q[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2
			q.q[1] = x1*w2 + w1*x2 + y1*z2 - z1*y2
			q.q[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2
			q.q[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2
			return q
		else:
			w,x,y,z = self.q
			px,py,pz = other.data
			w2 = w*w
			x2 = x*x
			y2 = y*y
			z2 = z*z
			xy = 2*x*y
			xz = 2*x*z
			wx = 2*w*x
			wy = 2*w*y
			wz = 2*w*z
			yz = 2*y*z
			v = Vector3()
			v.data[0] = (w2+x2-y2-z2)*px + (xy-wz)*py + (xz+wy)*pz
			v.data[1] = (xy+wz)*px + (w2-x2+y2-z2)*py + (yz-wx)*pz
			v.data[2] = (xz-wy)*px + (yz+wx)*py + (w2-x2-y2+z2)*pz
			
			return v

