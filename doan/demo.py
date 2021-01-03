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

  # kiểm tra bước có thể đi nằm trong phạm vi của bản đồ
  def in_bounds(self, p):
    x,y  = p
    return x >=0 and y>=0 and x<self.n and y<self.m

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

  # thuật toán A*
  def a_star(self):
    open_list = PriorityQueue()
    gScore = {(self.start, self.energy): 0}
    fScore_start = self.heuristic(self.start, self.goal) # f = g + h = 0 + heu(start, goal)
    open_list.push(fScore_start, (self.start, self.energy))
    self.came_from = {} # dùng để lưu dấu đường đi

    while not open_list.is_empty():
        item = open_list.pop()
        curr_fScore, curr = item
        curr_node, curr_energy = curr
        if curr_node == self.goal:
            return True
        if curr_energy > 0:
          for next_node in self.grid.neighbors(curr_node):
              if next_node in self.grid.material:
                new_energy = self.energy
              else:
                new_energy = curr_energy - 1 # mỗi khi đi 1 bước, nguyên liệu sẽ giảm 1
              new_g = gScore[(curr_node, curr_energy)] + 1
              if ((next_node, new_energy) not in gScore) or (new_g < gScore[(next_node, new_energy)]): 
                  gScore[(next_node, new_energy)] = new_g
                  fScore_next_node = gScore[(next_node, new_energy)] + self.heuristic(next_node, self.goal)

                  if next_node == self.goal:
                    self.lastEnergy = new_energy
                  open_list.push(fScore_next_node, (next_node, new_energy))
                  self.came_from[(next_node, new_energy)] = (curr_node, curr_energy)    
    return False

# thuật toán DFS
  def DFS(self):
    energy = self.energy
    stack = []
    stack.append((self.start, energy))
    visited = []    
    self.came_from = {}
    while len(stack) > 0:
      curr = stack.pop()      
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
            stack.append((next_node, new_energy))
            self.came_from[(next_node, new_energy)] = (node, energy)
    return False

# thuật toán UCS
  def UCS(self):
      open_list = PriorityQueue()
      gScore = {(self.start, self.energy): 0}
      open_list.push(gScore, (self.start, self.energy))
      self.came_from = {}
      while not open_list.is_empty():        
          item = open_list.pop()
          curr_gScore, curr = item
          curr_node, curr_energy = curr
          if curr_node == self.goal:
              return True
          if curr_energy > 0:
            for next_node in self.grid.neighbors(curr_node):
                if next_node in self.grid.material:
                  new_energy = self.energy
                else:
                  new_energy = curr_energy - 1
                new_g = gScore[(curr_node, curr_energy)] + 1
                if ((next_node, new_energy) not in gScore) or (new_g < gScore[(next_node, new_energy)]): 
                    gScore[(next_node, new_energy)] = new_g
                    if next_node == self.goal:
                      self.lastEnergy = new_energy
                    open_list.push(new_g, (next_node, new_energy))
                    self.came_from[(next_node, new_energy)] = (curr_node, curr_energy)
      return False


# đọc từ tệp input 
atlas = np.zeros((10, 10))
wall = []
material = []
matrix=np.loadtxt("input.txt",dtype='i',delimiter=',')
for i,j in enumerate(matrix):
   for k,l in enumerate(j):
     if l==3:
         a = (i,k)
         wall.append(a)
     if l==4:
         b = (i,k)
         material.append(b)
M = MapTraffic(atlas, wall, material)

print ( "1: là điểm bắt đầu (màu xanh lá)")
print ( "2: là điểm kết thúc (màu vàng)")
print ( "3: là chướng ngại vật (màu xanh dương)")
print ( "4: là trạm xăng (màu tím)")
print ( "5: là đường đi qua (màu xanh nhạt)")
print ( "7: là điểm đã đổ xăng (màu đen)")
print("--------------------------Bản đồ---------------------------- ")
M.show(show_weight=True)
print("--------------------------------------------------------------")
print("------ 1.DFS       2.BFS      3.UCS       4.A*   0.exit  ----")
print("------------------------------------------------------------")

while(True):
  number=input('\nNhập thuật toán muốn chạy: ')
  if number=="1":
    print("DFS")
    E = int(input("\nNhập số lít xăng ban đầu:"))
    search = SearchAlg(M, (0,0), E)
    search.DFS()
    M.show(show_weight=True, path=search.trace_path())
  if number=="2":
    print("BFS")
    E = int(input("\nNhập số lít xăng ban đầu:"))
    search = SearchAlg(M, (0,0), E)
    search.BFS()
    M.show(show_weight=True, path=search.trace_path())
  if number=="3":
    print("UCS")
    E = int(input("\nNhập số lít xăng ban đầu:"))
    search = SearchAlg(M, (0,0), E)
    search.UCS()
    M.show(show_weight=True, path=search.trace_path())
  if number=="4":
    print("A*")
    E = int(input("\nNhập số lít xăng ban đầu:"))
    search = SearchAlg(M, (0,0), E)
    search.a_star()
    M.show(show_weight=True, path=search.trace_path())
  if number=="0":
    break
 # NGUỒN THAO KHẢO CHÍNH https://stackoverflow.com/