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

 