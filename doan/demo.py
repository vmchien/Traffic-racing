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
	
   # hiển thị ra bản đồ
  def show(self, show_weight=False, path=[]):
    arr = np.empty((0,10), int)
    for i in range(self.n):
      a = []
      for j in range(self.m):
        if (i,j) in path:
          if (i,j) in self.material:
                a.append(7)
          else:
                a.append(5)
        elif (i, j) in self.material:
              a.append(4)
        elif self.passable((i,j)):
             a.append(0)
        else:
            a.append(3)
      arr = np.append(arr, np.array([a]), axis=0)
    arr[0][0] = 1
    arr[9][9] = 2
    back_map_modified = np.vectorize(get_color_coded_background)(arr)
    print_a_ndarray(back_map_modified, row_sep="")
    
 # tìm kiếm
class SearchAlg:
  def __init__(self, grid, start, energy): #khởi tạo giá trị
    self.grid = grid
    self.start = start
    self.goal = (grid.n - 1, grid.m - 1)
    self.energy = energy
    self.lastEnergy = 0
    self.came_from = {}

  def trace_path(self): # lưu lại thông tin đường đi
    node =  self.goal
    energy = self.lastEnergy
    path = []
    while node != self.start:
      path.append(node)
      curr = self.came_from[(node, energy)]
      node, energy = curr
    
    path.append(self.start)
    path.reverse()
    return path

  # dự đoán chi phí đến đích
  def heuristic(self,a1, a2, heu_type="Manhanttan"):
      return abs(a1[0]-a2[0]) + abs(a1[1]-a2[1])
  # thuật toán BFS 
  def BFS(self):
    energy = self.energy
    queue = []
    queue.append((self.start, energy))
    visited = []    
    self.came_from = {}
    while len(queue) > 0:
      curr = queue.pop(0)      
      node, energy = curr
      visited.append(curr)
      if energy > 0:
        for next_node in self.grid.neighbors(node):
          if next_node in self.grid.material:
              new_energy = self.energy
          else:
              new_energy = energy - 1
          if next_node == self.goal:
            self.lastEnergy = new_energy
            self.came_from[(self.goal, self.lastEnergy)] = (node, energy)
            return True
          elif (next_node, new_energy) not in visited:            
            queue.append((next_node, new_energy))
            self.came_from[(next_node, new_energy)] = (node, energy)
    return False