#!/usr/bin/env python

from __future__ import print_function

import time

import rospy
from std_srvs.srv import *

from gastronomous.srv import *
from gastronomous.Stepper import Stepper

pasta = [[1000, 5], [2000, 0]]
sauce = [[1000, 10], [2000, 10], [3000, 10], [4000, 0]]

def handlerStep(req):
  start = time.time()
  if 'pasta' in rospy.get_name():
    ramp = pasta
  elif 'sauce' in rospy.get_name():
    ramp = sauce
  else:
    return MoveByResponse(False, 'Incorrect node names')

  if stepper.isBusy():
    return MoveByResponse(False, 'BUSY!!!!!!!!!!')

  ret = stepper.run(req.distance, sleep=req.sleep, ramp=ramp)
  if ret == 0:
    txt = 'Moved %.1f in %.3fs' % (req.distance, time.time() - start)
  else:
    txt = stepper.errText(ret)
  return MoveByResponse(ret == 0, txt)

def handlerSleep(req):
  if stepper.isBusy():
    return SetBoolResponse(False, 'Currently Running, ignoring...')
  if req.data:
    return SetBoolResponse(stepper.sleep(), '')
  else:
    return SetBoolResponse(stepper.wake(), '')

def server():
  rospy.init_node('stepper_server', anonymous=True)
  args = rospy.get_param("~")

  global stepper
  stepper = Stepper(400, args['stepPin'], args['dirPin'], args['sleepPin'],
                    ratio=20, micro=Stepper.full, reverse=True)

  s = rospy.Service('move', MoveBy, handlerStep)
  s2 = rospy.Service('sleep', SetBool, handlerSleep)
  rospy.spin()

if __name__ == "__main__":
  server()
