height = 1.6
width = 4.3
depth = 2.7
mass = 1600
ixx = 1/12.0*mass*(height**2+depth**2)
iyy = 1/12.0*mass*(height**2+width**2)
izz = 1/12.0*mass*(width**2+depth**2)
print(ixx)
print(iyy)
print(izz)