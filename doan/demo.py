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

# hàng đợi ưu tiên
class PriorityQueue: 
  def __init__(self):
    self.queue = []

  # đưa giá trị vào hàng đợi
  def push(self, value, label):
    heapq.heappush(self.queue, (value, label))
  
  # lấy giá trị ra khỏi hàng đợi 
  def pop(self):
    return heapq.heappop(self.queue)
    
  # kiểm tra khi hàng đợi rỗng
  def is_empty(self): 
    return len(self.queue) == 0

# tạo bản đồ, các tham số truyền vào: kích thước, tường, các trạm nhiên liệu
class MapTraffic:
  def __init__(self, atlas, walls, material):
    self.n = len(atlas)
    self.m = len(atlas[0])
    self.atlas = atlas
    self.walls = walls
    self.material = material

  # kiểm tra có phải là tường
  def passable(self, p):
    for wall_pos in self.walls:
      if wall_pos == p:
        return False
    return True
  
  # giá trị bước đi xung quanh
  def neighbors(self, p):
    x, y = p
    neighbors = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]
    valid_neighbors = []
    for pos in neighbors:
      if self.in_bounds(pos) and self.passable(pos):
        valid_neighbors.append(pos)
    return valid_neighbors
   