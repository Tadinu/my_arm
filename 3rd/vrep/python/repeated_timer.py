#Ref: https://stackoverflow.com/questions/474528/what-is-the-best-way-to-repeatedly-execute-a-function-every-x-seconds-in-python
# https://stackoverflow.com/questions/36901/what-does-double-star-asterisk-and-star-asterisk-do-for-parameters

import time
import threading
from threading import Timer
class RepeatedTimer(object):
  def __init__(self, interval, timerTask, *args, **kwargs):
    self._timer = None
    self.interval = interval
    self.timerTask = timerTask
    self.args = args
    self.kwargs = kwargs
    self.is_running = False
    self.next_call = time.time()

  def _run(self):
    # Run the task
    self.timerTask(*self.args, **self.kwargs)

    # Start the new timer
    self.is_running = False
    self.start()

  def start(self):
    if not self.is_running:
      self.next_call += self.interval
      self._timer = threading.Timer(self.next_call - time.time(), self._run)
      self._timer.start()
      self.is_running = True

  def stop(self):
    self._timer.cancel()
    self.is_running = False
