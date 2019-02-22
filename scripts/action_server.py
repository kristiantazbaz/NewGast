#! /usr/bin/env python

from __future__ import print_function

from collections import deque
from threading import Thread, Event
from uuid import uuid4
from time import sleep
from random import randint

import rospy
import rosservice

from gastronomous.srv import *
from gastronomous.msg import *

q = deque()

class MealReq:
  amounts = {'small':120, 'medium':240, 'large':360,
             'reduced':120, 'normal':240, 'extra':360}
  cooktime = {'fusilli':5, 'pennette':5, 'rigatoni':5,
              'meat':5, 'tomato':5, 'cream':5}

  def __init__(self, req, line):
    self._pastaType = req.pastaType
    self._pastaAmount = req.pastaAmount
    self._sauceType = req.sauceType
    self._sauceAmount = req.sauceAmount
    self._line = line
    self._state = 0
    self._id = uuid4().hex
    self._pub = rospy.Publisher('/meal/update', MealUpdate, latch=True, queue_size=10)

  def id(self):
    return self._id

  def update(self, msg, done, line=None, state=None, stateTime=0):
    if line is not None:
      self._line = line
    if state is not None:
      self._state = state
    self._pub.publish(self._id, self._line, self._state, stateTime, msg, done)

  def pastaAmount(self):
    return MealReq.amounts[self._pastaAmount]

  def sauceAmount(self):
    return MealReq.amounts[self._sauceAmount]

  def sauceCook(self):
    return MealReq.cooktime[self._sauceType]

  def pastaCook(self):
    return MealReq.cooktime[self._pastaType]

def fakeRobot(sec=None):
  if sec is None:
    sleep(randint(1,5))
  else:
    sleep(sec)

def handler(req):
  meal = MealReq(req, len(q))
  q.append(meal)
  rospy.loginfo('Received new meal total %d' % len(q))
  if len(q) == 1:
    txt = 'Order received. You are first in line.'
  else:
    txt = 'Order received. You are number %d in line.' % (len(q)-1)
  return MealResponse(meal.id(), len(q)-1, True, txt)

class MyThread(Thread):
  def __init__(self):
    Thread.__init__(self)
    self.exitEvt = Event()

  def popNotify(self):
    q.popleft()
    for idx, ea in enumerate(q):
      print(idx, ea.id())
      ea.update('You are now %d in line' % idx, False, line=idx)

  def run(self):
    rospy.wait_for_service('pasta1/move')
    rospy.wait_for_service('sauce1/move')
    pasta1 = rospy.ServiceProxy('pasta1/move', MoveBy)
    sauce1 = rospy.ServiceProxy('sauce1/move', MoveBy)
    while not self.exitEvt.is_set():
      if len(q) > 0:
        meal = q[0] # Don't pop yet as new request ack message based on this
        meal.update('', False, state=1)
        pasta1(meal.pastaAmount(), True)
        meal.update('', False, state=2, stateTime=meal.pastaCook())
        fakeRobot(meal.pastaCook())
        meal.update('', False, state=3)
        sauce1(meal.sauceAmount(), True)
        meal.update('', False, state=4, stateTime=meal.sauceCook())
        fakeRobot(meal.sauceCook())
        meal.update('', True, state=5)
        self.popNotify()
      else:
        sleep(0.1)

  def onExit(self):
    self.exitEvt.set()

if __name__ == '__main__':
  rospy.init_node('meal_server')
  s = rospy.Service('meal', Meal, handler)

  t = MyThread()
  t.start()
  rospy.on_shutdown(t.onExit)

  rospy.spin()
  t.join()
