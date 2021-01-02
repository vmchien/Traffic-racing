import heapq
import sys
from termcolor import colored
import math
import numpy as np

# vẽ map
def get_color_coded_background(i):
    return "\033[4{}m {} \033[0m".format(i+1, i)
def print_a_ndarray(map, row_sep=" "):
    n, m = map.shape
    fmt_str = "\n".join([row_sep.join(["{}"]*m)]*n)
    print(fmt_str.format(*map.ravel()))

class PriorityQueue:
  def __init__(self):
    self.queue = []
  def push(self, value, label):
    heapq.heappush(self.queue, (value, label))
  def pop(self):
    return heapq.heappop(self.queue)
  def is_empty(self): 
    return len(self.queue) == 0