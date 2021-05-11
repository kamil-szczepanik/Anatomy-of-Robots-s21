from math import acos, asin, atan2, sin, cos, pi, atan, sqrt
import numpy as np

params = {"base":{"a":0,"d":0.05,"alpha":0}, 
	"base_ext":{"a":0,"d":0,"alpha":-pi/2},
	"arm":{"a":1,"d":0,"alpha":0},
	"hand":{"a":0.5,"d":0,"alpha":0}}

def calc(t1, t2 ,t3):

	positions = [t1, t2, t3, 0]
	T = np.eye(4)

	for i in range(len(params.keys())):

		theta = positions[i]

		if i == 0:
			link = 'base'
		if i == 1:
			link = 'base_ext'
		if i == 2:
			link = 'arm'
		if i == 3:
			link = 'hand'
		
		d = params[link]['d']
		a = params[link]['a']
		alpha = params[link]['alpha']
		
		Rotx = np.array([[1, 0, 0, 0],
							[0, cos(alpha), -sin(alpha), 0],
							[0, sin(alpha), cos(alpha), 0],
							[0, 0, 0, 1]])

		Transx = np.array([[1, 0, 0, a],
							[0, 1, 0, 0],
							[0, 0, 1, 0],
							[0, 0, 0, 1]])

		Rotz = np.array([[cos(theta), -sin(theta), 0, 0],
							[sin(theta), cos(theta), 0, 0],
							[0, 0, 1, 0],
							[0, 0, 0, 1]])

		Transz = np.array([[1, 0, 0, 0],
							[0, 1, 0, 0],
							[0, 0, 1, d],
							[0, 0, 0, 1]])

		T_curr = Rotx@Transx@Rotz@Transz
		T = T @ T_curr

	xyz = [round(T[0][3],4), round(T[1][3],4), round(T[2][3],4)]

	return xyz

def un_calc(x, y, z):
	return [un_calc1(x,y,z),un_calc2(x,y,z),un_calc3(x,y,z),un_calc4(x,y,z)]

def un_calc1(x,y,z):
	a2 = params["arm"]["a"]
	a3 = params["hand"]["a"]
	d1 = params["base"]["d"]
	try:
		t1 = atan2(y,x)
		t3 = acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
		t2 = -asin(a3*sin(t3)/((z-d1)**2+x**2+y**2))-atan2((z-d1),sqrt(x**2+y**2))
	except Exception:
		print("1\n")
		t1=t2=t3=0
	return (t1,t2,t3)

def un_calc2(x,y,z):
	a2 = params["arm"]["a"]
	a3 = params["hand"]["a"]
	d1 = params["base"]["d"]
	try:
		t1 = atan2(y,x)
		t3 = -acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
		t2 = -asin(a3*sin(t3)/((z-d1)**2+x**2+y**2))-atan2((z-d1),sqrt(x**2+y**2))
	except Exception:
		print("2\n")
		t1=t2=t3=0
	return (t1,t2,t3)

def un_calc3(x,y,z):
	a2 = params["arm"]["a"]
	a3 = params["hand"]["a"]
	d1 = params["base"]["d"]
	try:
		t1 = atan2(y,x)+pi
		t3 = acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
		t2 = pi - asin(a3*sin(t3)/((z-d1)**2+x**2+y**2)) + atan2((z-d1),sqrt(x**2+y**2))
	except Exception:
		print("3\n")
		t1=t2=t3=0
	return (t1,t2,t3)

def un_calc4(x,y,z):
	a2 = params["arm"]["a"]
	a3 = params["hand"]["a"]
	d1 = params["base"]["d"]
	try:
		t1 = atan2(y,x)+pi
		t3 = -acos(((z-d1)**2+x**2+y**2-a2**2-a3**2)/(2*a2*a3))
		t2 = pi - asin(a3*sin(t3)/((z-d1)**2+x**2+y**2)) + atan2((z-d1),sqrt(x**2+y**2))
	except Exception:
		print("4\n")
		t1=t2=t3=0
	return (t1,t2,t3)

u_case = un_calc(0,1,0)

print(u_case[0],calc(*u_case[0]))
print(u_case[1],calc(*u_case[1]))
print(u_case[2],calc(*u_case[2]))
print(u_case[3],calc(*u_case[3]))
