import numpy as np
import numpy as np


def G2C_RGF(longitude, latitude, h) :

  GRS_a = 6378137
  GRS_f = 1/298.257222101
  GRS_b = GRS_a*(1-GRS_f)
  GRS_e = np.sqrt((np.power(GRS_a,2) - np.power(GRS_b,2)) / np.power(GRS_a,2))
  N = GRS_a / np.sqrt(1 - np.power(GRS_e,2) * np.power(np.sin(latitude),2))

  # TODO : compute the coordinates of the point in the RGF93 frame

  X = 0
  Y = 0
  Z = 0

  return np.array([X, Y, Z])

def C_RGF2ENU(P_ECEF, l, phi, h) :
  eO = G2C_RGF(l, phi, h)
  P_ECEF = P_ECEF[np.newaxis].T # (3,1)

  # TODO : compute the coordinates of the point in the local frame
  oAe = np.eye(3)

  P_ENU = oAe @ (P_ECEF -  eO[np.newaxis].T)

  return P_ENU[:, 0]



def gdfs_to_local(nodes_proj, path):
  l = nodes_proj.at[nodes_proj.index[0],"x"] * np.pi / 180
  phi = nodes_proj.at[nodes_proj.index[0],"y"] * np.pi / 180
  h0 = 342.

  # put the points in a frame in meter :
  waypoints_ECEF = np.zeros((nodes_proj.loc[path]["x"].size, 3))
  waypoints_ENU = np.zeros((nodes_proj.loc[path]["x"].size, 3))
  
  for i in range(nodes_proj.loc[path]["x"].size):
      longitude = nodes_proj.loc[path[i]]["x"] * np.pi / 180
      latitude =  nodes_proj.loc[path[i]]["y"] * np.pi / 180

      # TODO : compute the coordinates of the point in the RGF frame
      waypoints_ECEF[i] = G2C_RGF(longitude, latitude, h0)
      # TODO : compute the coordinates of the point in the local frame
      waypoints_ENU[i] = C_RGF2ENU(waypoints_ECEF[i], l, phi, h0)

  return waypoints_ENU

