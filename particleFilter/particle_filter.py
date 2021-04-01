import robot
import numpy as np
from math import sqrt,exp,log
from copy import deepcopy

def normpdf(d,mu,sigma):
  '''
  For a Gaussian probability density function with mean mu and standard
  deviation sigma, returns the value of that pdf at point d ("given the
  distribution with mean u and standard deviation sigma, what is the
  probability of d?").
  '''
  return (1/sqrt(2*np.pi*sigma*sigma))*exp(-.5*pow(d-mu,2)/pow(sigma,2))

class Particle:
  '''
  Represents one particle.  Can do things particles do.
  '''
  def __init__(self,room,
      x=None,y=None,yaw=None,
      c_o=1,sigma_o=.5,
      b_l=1,sigma_l=.2,
      b_r=1,sigma_r=.1):
    '''
    Must be given a pointer to the room object.  If x, y, and yaw are not
    given, the particle is placed randomly within the room.  Linear-Gaussian
    noise parameters can also be input.  *_o are parameters for observation
    noise, *_l are parameters for straight line driving, and *_r are for
    turning (in radians).
    '''
    self.__maxX=room.maxX
    self.__maxY=room.maxY
    self.__room=room

    if x is None:
      self.__x=np.random.random()*self.__maxX
    else:
      self.__x=x
    if y is None:
      self.__y=np.random.random()*self.__maxY
    else:
      self.__y=y
    if yaw is None:
      self.__yaw=np.random.random()*2*np.pi
    else:
      self.__yaw=yaw
    self.__yaw=robot.fixAngle(self.__yaw)

    self.c_o=c_o
    self.sigma_o=sigma_o
    self.b_l=b_l
    self.sigma_l=sigma_l
    self.b_r=b_r
    self.sigma_r=sigma_r

    self.weight = 0

  def obs(self):
    '''
    Returns three non-noisy measurements from the particle.
    '''
    angles=[robot.fixAngle(angle) for angle in [self.__yaw-np.pi/6,self.__yaw,self.__yaw+np.pi/6]]
    trueObs=[self.__room.trueObservation(self.__x,self.__y,angle) for angle in angles]
    return trueObs
  
  def drive(self,linearDist):
    '''
    Performs noisy driving, based on the noise parameters b_l and sigma_l,
    stopping short if it runs into a wall.
    '''
    realDist = self.b_l*linearDist+np.random.normal(scale=self.sigma_l)

    toWall=self.__room.trueObservation(self.__x,self.__y,self.__yaw)
    if toWall<realDist:
      realDist=toWall-1e-5
    self.__x+=realDist*np.cos(self.__yaw)
    self.__y+=realDist*np.sin(self.__yaw)

  def turn(self,angle):
    '''
    Performs noisy driving, based on the noise parameters b_r and sigma_r.
    '''
    realAngle= self.b_r*angle+np.random.normal(scale=self.sigma_r)
    self.__yaw+=realAngle
    self.__yaw=robot.fixAngle(self.__yaw)

  def pose(self):
    '''
    Returns the pose of the particle as a tuple of x, y, yaw
    '''
    return (self.__x,self.__y,self.__yaw)

def move(rob,room,particles):
  '''
  Should move the robot and particles for the particle filter.
  '''
  TorD=input("(T)urn, (D)rive, or (Q)uit? ")
  if TorD=='T' or TorD=='t':
    angle=float(input(" Angle? "))
    #TODO
    for p in particles:
      p.turn(angle)
    rob.turn(angle)

  elif TorD=='D' or TorD=='d':
    distance=float(input(' Distance? '))
    #TODO
    for p in particles:
      p.drive(distance)
    rob.drive(distance)
  elif TorD=='Q' or TorD=='q':
    exit()

def observe(rob,room,particles):
  '''
  Should take an observation from the robot and perform particle resampling.
  Return the list of new particles.
  '''
  #TODO
  # Weighting with multiple observations
  observations = rob.obs()
  weight_sum = _weightParticles(particles, observations)

  # resample the weights and return
  return _resample(particles, weight_sum)

def _weightParticles(particles, observations):
  '''
  Returns the sum of the particles' weights using monte carlo localization
  '''
  sum_ = 0
  for p in particles:
    p.weight=np.log(1)
    p_observations = p.obs()
    for o, op in zip(observations, p_observations):
      addend = normpdf(o - p.c_o*op, 0, p.sigma_o)
      if addend == 0:
        addend = 10**-15
      p.weight += log(addend)
    
    p.weight = exp(p.weight)
    sum_ += p.weight

  return sum_

def _resample(particles, weightsum):
  '''
  Resample the particles so the most likely are kept. 
  Returns the list of new particles. 
  '''
  newParticles = []
  for i in range(len(particles)):
    r = weightsum * np.random.random()
    samplesum = 0
    for p in particles:
      samplesum += p.weight 
      if r <= samplesum:
        newParticles.append(deepcopy(p))
        break 
  
  return newParticles 


def main():
  N=10000
  room=robot.Room()
  rob=robot.Robot(room,sigma_o=.3,sigma_l=.2,sigma_r=.1)
  particles = [Particle(room,sigma_o=.8,sigma_l=.2,sigma_r=.1) for i in range(N)]

  while True:
    robot.display(rob,room,particles)
    move(rob,room,particles)
    robot.display(rob,room,particles)
    particles=observe(rob,room,particles)
    robot.display(rob,room,particles)

if __name__ == "__main__":
  main()
