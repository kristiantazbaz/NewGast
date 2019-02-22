from __future__ import print_function
from __future__ import division

import pigpio
from pigpio import OUTPUT, INPUT
import time
from copy import deepcopy

class Stepper:
  full = 1
  half = 2
  quarter = 4
  eigth = 8
  sixteenth = 16

  def __init__(self, steps, step, direction, sleep=0, ratio=1, ms=[0, 0, 0], micro=eigth, reverse=False):
    self.pi = pigpio.pi('192.168.0.109')
    self.steps = steps
    self.step = step
    self.direction = direction
    self.sleepPin = sleep
    self.ratio = ratio
    self.ms = ms
    self.reverse = reverse

    self.setMicroStep(micro)
    self.error = 0

    self.pi.set_mode(step, OUTPUT)
    self.pi.set_mode(direction, OUTPUT)
    if self.sleepPin > 0:
      self.pi.set_mode(self.sleepPin, OUTPUT)
      self.sleep()

  def cleanup(self):
    return self.pi.stop()

  def __del__(self):
    self.cleanup()

  def errText(self, err):
    if err == 0:
      return ''
    return pigpio.error_text(err)

  def setMicroStep(self, micro):
    self.micro = micro
    self.degStep = 360 / self.steps / micro
    if self.ms[2] != 0:
      pi.set_mode(self.ms[2], OUTPUT)
      if micro == sixteenth:
        pi.write(self.ms[2], True)
    if self.ms[1] != 0:
      pi.set_mode(self.ms[1], OUTPUT)
      if micro in (quarter, eigth):
        pi.write(ms[1], True)
    if self.ms[0] != 0:
      pi.set_mode(self.ms[0], OUTPUT)
      if micro in (half, eight, sixteenth):
        pi.write(self.ms[0], True)

  def connected(self):
    return self.pi.connected

  def wake(self):
    if self.sleep > 0:
      self.pi.write(self.sleepPin, True)
      time.sleep(0.01)
      return True
    return False

  def sleep(self):
    if self.sleep > 0:
      self.pi.write(self.sleepPin, False)
      return True
    return False

  def genWave(self, ramp):
    self.pi.wave_clear()
    length = len(ramp)
    wid = [0]*length

    adr = 1 << self.step
    for i in range(length):
      freq = ramp[i][0]
      micros = 1000000 // freq
      wf = []
      wf.append(pigpio.pulse(adr, 0, micros))
      wf.append(pigpio.pulse(0, adr, micros))
      self.pi.wave_add_generic(wf)
      wid[i] = self.pi.wave_create()

    chain = []
    for i in range(length):
      steps = ramp[i][1]
      x = steps & 255
      y = steps >> 8
      chain += [255, 0, wid[i], 255, 1, x, y]

    return self.pi.wave_chain(chain)

  def isBusy(self):
    return self.pi.wave_tx_busy() == 1

  def waitBusy(self):
    while self.isBusy():
      time.sleep(0.1)
    return True

  def genRamp(self, pulses):
    #ramp = {self.full:[[1000, 10], [2000, 10], [3000, 10], [4000, 10], [5000, pulses-40*2]]}
    ramp = {self.full:[[250, 5], [300, pulses-5*2]]}
    ramp[self.micro].extend(reversed(ramp[self.micro][:-1]))
    return ramp[self.micro]

  def parseRamp(self, ramp, pulses):
    amp = deepcopy(ramp)
    cnt = sum(v for _, v in amp)
    amp[-1][1] = pulses - cnt * 2
    amp.extend(reversed(amp[:-1]))
    return amp

  def run(self, deg, sleep=False, async=False, ramp=None):
    if deg < 0:
      self.pi.write(self.direction, not self.reverse)
      deg *= -1
    elif deg > 0:
      self.pi.write(self.direction, self.reverse)
    else:
      return 0

    self.wake()
    pulsesDec = deg * self.ratio / self.degStep
    pulses = int(round(pulsesDec))
    self.error += (pulsesDec - pulses) * self.degStep
    errMod = int(self.error % self.degStep)
    if errMod > 0:
      if self.error > 0:
        pulses -= errMod
        self.error -= errMod * self.degStep
      else:
        pulses += errMod
        self.error += errMod * self.degStep
    if ramp is None:
      ramp = self.genRamp(pulses)
    else:
      ramp = self.parseRamp(ramp, pulses)
    ret = self.genWave(ramp)
    if async:
      return ret
    else:
      self.waitBusy()
      if sleep:
        self.sleep()
      return ret

if __name__ == '__main__':
  import sys
  deg = float(sys.argv[1])
  print("You told me to turn %s degrees." % deg)

  stepper = Stepper(400, 24, 23, ratio=20, micro=Stepper.full, reverse=True)
  stepper.run(deg)
  stepper.cleanup()
