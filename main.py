import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos, pi, sqrt

global g
# Gravity, 10 is used for debugging
g = -9.8

def getPathOnXYFunction(funcs, delta_t = 0.01):
  '''
    Funcs - An array of two functions, the first one will return the x component of an objects trajectory at time point t, and the second will return the y component of an objects trajectory at time point t. It will run these function until the object hits the ground.

    Returns an array of x positions, y positions, and the time it took to hit the ground
  '''

  # Get the x and y functions out of the functions array so that we can more intuitively refer to them
  xFunc = funcs[0]
  yFunc = funcs[1]

  # Start off time at delta time (no need to compute the x and y positions at t=0 because we know that it will start on the group)
  t = delta_t

  # Create an array to store the x and y positions, initialize the array with the x and y positions at the first timestep
  x = [xFunc(t)]
  y = [yFunc(t)]
  
  # Run the functions until the y x of the function is less than 0 (the object has hit the ground)
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

def calculateRequiredInitialVelocityToPassThroughAPoint(coords):
  '''
  This is a function that I derived in order to calculate the required initial velocity for an object to pass through a point.
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
  theta = 0
  go = True
  iterations = 0
  minimum_distance = 0

  # How this works is that it plots the trajectory of the object at changing angles of being shot, and it returns the correct angle once it is hit. This can be optimized by a hell of a lot and there is probably a mathetmatical formula that you can use to get the correct point in like 2 milliseconds buuuuuuut I already made a very good formula before that used a lot of brain power and Winter break was almost over so I settled on this solution, if I need to run this formula on a system that actually shoots things and is very time sensitive, then I will fix this, but otherwise there isn't a need to fix it. 
  while go:
    new_theta_1 = theta + (pi / 4) * 2 ** (-iterations)
    new_theta_2 = theta - (pi / 4) * 2 ** (-iterations)

    # Run a simulation for both new theta angles

    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(new_theta_1, v_i))
    minimum_distance_theta_1 = float('inf')

    # Find the point that is the closest to the point we want to hit
    for x, y in zip(_x, _y):
      distance = sqrt((x - point[0])**2 + (y - point[1])**2)
      minimum_distance_theta_1 = min(minimum_distance_theta_1, distance)
    
    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(new_theta_2, v_i))
    minimum_distance_theta_2 = float('inf')

    # Find the point that is the closest to the point we want to hit
    for x, y in zip(_x, _y):
      distance = sqrt((x - point[0])**2 + (y - point[1])**2)
      minimum_distance_theta_2 = min(minimum_distance_theta_2, distance)
    
    # If the new theta angles are closer to the point we want to hit, then we will use those angles
    if minimum_distance_theta_1 < minimum_distance_theta_2:
      minimum_distance = minimum_distance_theta_1
      theta = new_theta_1
    else:
      minimum_distance = minimum_distance_theta_2
      theta = new_theta_2

    print("Minimum distance: ", minimum_distance, "theta: ", theta)
    
    # If the point we want to hit is within the tolerance, then we are done
    if minimum_distance < sizeOfPoint:
      go = False

    iterations += 1
    if iterations > 50:
      print("Reached maximimum iterations!", theta)
      go = False

  return theta
      

def getMinViAndThetaForPoint(point, plot = False, sizeOfPoint = 0.1):
  '''This function will get the minimum initial velocity and angle at which you need to shoot the object at for a specific point, if you want to plot that then set plot to True.'''
  
  # Add a little bit of velocity because this current number is the minimum required
  v_i = calculateRequiredInitialVelocityToPassThroughAPoint(point)
  
  theta = getThetaForPathToHitPoint(v_i, point, sizeOfPoint=0.01)
  sizeOfPoint = ((v_i * v_i)/(2 * g)) / 100
  if plot:
    _x, _y, _t = getPathOnXYFunction(returnXYFuncs(theta, v_i))
    plt.plot(_x,_y, label = "Path of Object")
    plt.plot(point[0], point[1], 'ro', label = "Goal")
    plt.xlabel("X Position Of Object (m)")
    plt.ylabel("Y Position Of Object (m)")
    plt.title("Path of Object Towards A Goal Point")
    plt.legend()
  plt.show()
  return v_i, theta


if __name__ == "__main__":
  input_from_user = input("Enter the target: ")

  # Parse input from the user, if the user inputs a comma then parse it as a list, if 
  # the user doesn't have a comma then assume that the user separated the number via a space
  if "," in input_from_user:
    target = list(map(float, input_from_user.split(",")))
  else:
    target = list(map(float, input_from_user.split(" ")))

  v_i, theta = (getMinViAndThetaForPoint(target, plot = True))
  print(f"FOR INTENDED TARGET OF: " + repr(target) + f":\nMinimum Initial velocity is: {round(v_i * 100) / 100} m/s, with an angle of {round( 100 * theta * 180 / pi) / 100} degrees")

