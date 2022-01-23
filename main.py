import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, pi, sqrt
from time import time

global g
# Gravity, 10 is used for debugging
g = -9.8

def getPathOnXYFunction(funcs):
  '''
    Funcs - An array of two functions, the first one will return the x component of an objects trajectory at time point t, and the second will return the y component of an objects trajectory at time point t. It will run these function until the object hits the ground.

    Returns an array of x positions, y positions, and the time it took to hit the ground
  '''
  xFunc = funcs[0]
  yFunc = funcs[1]
  delta_t = 0.01
  t = delta_t
  x = [xFunc(t)]
  y = [yFunc(t)]
  
  # If you want this program to work with objects shot backwards, then remove x[-1] > 0
  while y[-1] > 0 and x[-1] > 0:
    t += delta_t
    x.append(xFunc(t))
    y.append(yFunc(t))
  return x, y, t

def returnXYFuncs(theta, v_i):
  '''This will return two funcions, for the x and y component of the objects path, depending upon the objects initial launch angle (theta) and initial velocity'''
  return returnXFunc(theta, v_i), returnYFunc(theta, v_i)

def returnXFunc(theta, v_i):
  '''Returns the x component of the objects trajecotry using the following formula'''
  return lambda t: cos(theta) * v_i * t

def returnYFunc(theta, v_i):
  ''' Returns the x component of the objects trajecotry using the following formula.'''
  # NOTE: This function is assuming that you live on earth and thus acceleration is gravity
  return lambda t: (1/2 * g * t * t + sin(theta) * v_i * t)

def returnPossiblePathFunc(v_i):
  '''This returns the function that, depending on the initial velocity, will compute the maximum (x,y) position that any object can hit.'''
  v_i_2 = v_i * v_i
  return lambda t: (((g)/(2 * v_i_2)) * t * t - (v_i_2/(2 * g)))

def getUntilZero(func):
  '''This function is just used to run a function until the f(x) component of the function reaches 0'''
  delta_t = 0.01
  t = delta_t
  x = [t]
  y = [func(t)]
  
  while y[-1] > 0:
    t += delta_t
    y.append(func(t))
    x.append(t)
  return x, y

def calculateRequiredInitialVelocityToPassThroughAPoint(coords):
  '''This function is very useful and, just like the name says, it will calculuate the required initial velocity in order for an object to pass through the inputted point.
  
  The formula for this equation is:



         ______________________________________________
         |         ____________________________________
   v_i = |-2gy + __|((2gy)^2 - (4 * -(g^2 * x^2)))
         |----------------------------------------------
       __|                      2

  '''
  x = coords[0]
  y = coords[1]

  w = 2 * g * y
  q = - g * g * x * x

  squareRoot = sqrt((w * w) - (4 * q))
  expression = (-w + squareRoot)/2

  return (sqrt(expression))

def getThetaForPathToHitPoint(v_i, point, sizeOfPoint = 0.05):
  '''This function, when given the initial velocity required, will output the angle needed to shoot at.
    point - point we want to hit
    sizeOfPoint - the tolerance at which we can hit the point, at extereme initial velocities, this needs to be very high
  '''
  delta_theta = -pi/700
  theta = (pi/2) + delta_theta
  go = True
  i = 0

  # How this works is that it plots the trajectory of the object at changing angles of being shot, and it returns the correct angle once it is hit. This can be optimized by a hell of a lot and there is probably a mathetmatical formula that you can use to get the correct point in like 2 milliseconds buuuuuuut I already made a very good formula before that used a lot of brain power and Winter break was almost over so I settled on this solution, if I need to run this formula on a system that actually shoots things and is very time sensitive, then I will fix this, but otherwise there isn't a need to fix it. 
  while go:
    theta += delta_theta
    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, v_i))
    for x,y in zip(_x,_y):
      if abs(point[0] - x) < sizeOfPoint:
        if abs(point[1] - y) < sizeOfPoint:
          go = False
    if len(_y) <= 1:
      go = False
    i += 1
    if i > 1000:
      go = False
      print(theta)

  return theta
      

def getMinViAndThetaForPoint(point, plot = False, sizeOfPoint = 0.1):
  '''This function will get the minimum initial velocity and angle at which you need to shoot the object at for a specific point, if you want to plot that then set plot to True.'''
  v_i = calculateRequiredInitialVelocityToPassThroughAPoint(point)
  #__x, __y = getUntilZero(returnPossiblePathFunc(v_i))
  #plt.plot(__x,__y)
  
  theta = getThetaForPathToHitPoint(v_i, point)
  sizeOfPoint = ((v_i * v_i)/(2 * g)) / 100
  if plot:
    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, v_i))
    plt.plot(_x,_y, label = "pathOfObject")
    plt.plot([point[0]-sizeOfPoint, point[0] - sizeOfPoint, point[0] + sizeOfPoint, point[0] + sizeOfPoint, point[0]-0.1], [point[1] - sizeOfPoint, point[1] + sizeOfPoint, point[1] + sizeOfPoint, point[1] - sizeOfPoint, point[1] - sizeOfPoint], label = "target")
    plt.legend()
    plt.show()
  
  return v_i, theta

def plotAllTrajectoriesOfObject(v_i):
  '''This function was used for me to discover the other function that gets the maximum point it is possible for an object to each at a specific velocity, it just plots all possible paths an object can take, it makes a pretty cool shape'''
  delta_theta = pi/180
  theta = delta_theta
  _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, v_i))
  while len(_y) > 1:
    plt.plot(_x,_y, label = f"theta = {theta}")
    theta += delta_theta
    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, v_i))
  func = returnPossiblePathFunc(v_i)
  _x, _y = getUntilZero(func)
  plt.plot(_x, _y)
  plt.show()

def plotTrajectoryForAnObjectToReachAPoint(point):
  '''This is the big boy function, it plots the optimal trajectory for an object to take in order to each a desired point.'''
  v_i = calculateRequiredInitialVelocityToPassThroughAPoint(point)

  delta_theta = pi/360
  theta = 0
  _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, v_i))
  while len(_y) > 1:
    plt.plot(_x,_y, label = f"theta = {theta}")
    theta += delta_theta
    if point[0] in np.round(np.array(_x) * 100) * 0.01 and point[1] in np.round(np.array(_y) * 100) * 0.01:
      print(theta)
    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, v_i))
  func = returnPossiblePathFunc(v_i)
  _x, _y = getUntilZero(func)
  plt.plot((np.arange(100) * 0.01) + point[0], (np.arange(100) * 0.01) + point[1], label = "Goal")
  plt.plot(_x, _y)
  plt.legend()
  plt.show()

if __name__ == "__main__":
  # CHANGE THE TARGET RIGHT HERE
  target = [5,2]
  print(f"FOR INTENDED TARGET OF: " + repr(target) + f":\nMinimum Initial velocity is: {round(v_i * 100) / 100} m/s, with an angle of {round( 100 * theta * 180 / pi) / 100} degrees")

  v_i, theta = (getMinViAndThetaForPoint(target, plot = True))
