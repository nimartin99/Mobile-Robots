import numpy as np


# a) Compute Points 
print("\n------------------------------------------- a) -------------------------------------------------")
p1 = np.array([8459056.66210967 , -740071.56205804 , 20004411.47522843])
p2 = np.array([20254541.05620208 , 0. , -3571422.07007576])
p3 = np.array([17071468.6094096 , 6213506.42903493 , 6612275.43091516])

d1_2 = np.linalg.norm(p1-p2)
print(p2-p1)
d1_3 = np.linalg.norm(p1-p3)

u = d1_2
print("U: ", u)
v_x = (p2-p1)@(p3-p1) / d1_2
print("V_x: ", v_x)
v_y = (np.sqrt(d1_3**2 - v_x ** 2))
print("V_y: ", v_y)

x_new = p2-p1   #(spannvektor der von p1 nach p2 zeigt, achse der Satelliten CS)
p1p3 = p3-p1    #(spannvektor, der von p1 nach p3 zeigt)
z_new = -1 * np.cross(x_new, p1p3)  #z achse ist nun orthogonaler vector zu x_new(p1p2) und vektor p1p1 --> vektorprodukt
y_new = np.cross(x_new, z_new)      #y achse ist orthogonaler vector zu x und z achse --> vektorprodukt

# norm vectors
x_new = x_new / np.linalg.norm(x_new)
y_new = y_new / np.linalg.norm(y_new)
z_new = z_new / np.linalg.norm(z_new)

W_R_S = np.array([  [x_new[0], y_new[0], z_new[0], p1[0]],
                    [x_new[1], y_new[1], z_new[1], p1[1]],
                    [x_new[2], y_new[2], z_new[2], p1[2]],
                    [0, 0, 0, 1]])
print("W_R_S = ", W_R_S)
S_R_W = np.linalg.inv(W_R_S)
print("S_R_W = ", S_R_W)

p1 = np.array([8459056.66210967 , -740071.56205804 , 20004411.47522843, 1])
p2 = np.array([20254541.05620208 , 0. , -3571422.07007576, 1])
p3 = np.array([17071468.6094096 , 6213506.42903493 , 6612275.43091516, 1])
print("p1 in S-CS:", S_R_W @ p1)
print("p2 in S-CS:", S_R_W @ p2)
print("p3 in S-CS:", S_R_W @ p3)


# b) compute distances and coordinates of p
print("\n------------------------------------------- b) -------------------------------------------------")
t0 = 436543.45999646

s1 = 436543.40699823
s2 = 436543.39960552
s3 = 436543.41280555

def clock2dist(t, s):
    #return (t - s) * 299792458
    return (t - s) * 300000000 

d1 = clock2dist(t0,s1)
d2 = clock2dist(t0,s2)
d3 = clock2dist(t0,s3)

print("Distances: ", d1, d2, d3)

x = (d2**2 - u**2 - d1**2) / (-2*u)
y = (d3**2 - ((d2**2-u**2-d1**2)/(u))*v_x - v_x**2 - v_y**2 - d1**2) / (-2*v_y)
z = np.sqrt(d1**2 - x**2 - y**2)
print("Coords of P in S-CS:", x, y, z)

# c) Transform Coords from from S to W CS
print("\n------------------------------------------- c) -------------------------------------------------")
p_s = np.array([x,y,z,1])
p_w = W_R_S@p_s
print("P in ECEF:", p_w)


# d) ECEF to LatLon
print("\n------------------------------------------- d) -------------------------------------------------")
import pyproj

transformer = pyproj.Transformer.from_crs(
    {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
    {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
    )

lon1, lat1, alt1 = transformer.transform(p_w[0],p_w[1],p_w[2],radians=False)
print("Point:", lat1, lon1, alt1 )
lon1, lat1, alt1 = transformer.transform(p1[0],p1[1],p1[2],radians=False)
print("S1:", lat1, lon1, alt1 )
lon1, lat1, alt1 = transformer.transform(p2[0],p2[1],p2[2],radians=False)
print("S2:", lat1, lon1, alt1 )
lon1, lat1, alt1 = transformer.transform(p3[0],p3[1],p3[2],radians=False)
print("S3:", lat1, lon1, alt1 )


# e) Adapt Formula to account for bias
print("\n------------------------------------------- e) -------------------------------------------------")

def clock2dist_bias(t,s):
    d = (t + 0.1 - s) * 300000000
    return d

d1 = clock2dist_bias(t0,s1)
d2 = clock2dist_bias(t0,s2)
d3 = clock2dist_bias(t0,s3)
print("Distances (biased): ", d1, d2, d3)

x = (d2**2 - u**2 - d1**2) / (-2*u)
y = (d3**2 - ((d2**2-u**2-d1**2)/(u))*v_x - v_x**2 - v_y**2 - d1**2) / (-2*v_y)
z = np.sqrt(d1**2 - x**2 - y**2)
print("Coords of P in S-CS (biased):", x, y, z)

p_s = np.array([x,y,z,1])
p_w = W_R_S@p_s
print("P in ECEF (biased):", p_w)


import pyproj

transformer = pyproj.Transformer.from_crs(
    {"proj":'geocent', "ellps":'WGS84', "datum":'WGS84'},
    {"proj":'latlong', "ellps":'WGS84', "datum":'WGS84'},
    )

lon1, lat1, alt1 = transformer.transform(p_w[0],p_w[1],p_w[2],radians=False)
print("Point (biased): ", lat1, lon1, alt1 )