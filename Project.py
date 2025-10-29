import heapq
import math
from itertools import count


def euclid_distance(state, goal, col):
  sum = 0
  for i , tile in enumerate(state): 
    if tile != 0:#no need to calcualte blank square
      x_state, y_state = (i)%col, (i)//col #convert 1d to 2d
      x_goal, y_goal = (tile -1) %col, (tile-1) //col
      sum += math.sqrt((x_goal - x_state)**2 + (y_goal-y_state)**2)
  return sum


def back_track(end):
  curr = (end, '')
  print_stack = []
  while curr is not None:
    print_stack.append(curr[1])
    print_stack.append(curr[0])
    curr = curr[0].get_path()
  
  while len(print_stack) != 0:
    element = print_stack.pop()
    if isinstance(element, str):
      print(element)
    else:
      element.state_print()

class Node(object):
  def __init__(self, state, distance, goal_state, func, size): #set up initials
    self.state = state
    self.disatant_from_start = distance
    self.size = size
    self.heuristic = func(state, goal_state,size)
    self.parent = None


  def __eq__(self,rhs):
    for i in range(len(self.state)):
      if self.state[i] != rhs.state[i]:
        return False
    return True
  
  def __hash__(self):#need this for the class of sets
    return hash(tuple(self.state))
  
  def children(self, goal_state, h_function):
    childs = []
    child_distance = self.disatant_from_start + 1
    blank = None #blank between the numbers
    for i, tile in enumerate(self.state):#check where is the 0
      if tile == 0:
         blank = i
         break
      

    if blank % self.size != 0: #swap left move
      left_state = self.state.copy()
      left_state[blank], left_state[blank-1] = left_state[blank-1], left_state[blank]
      left_child = Node(left_state, child_distance, goal_state, h_function, self.size)
      left_child.set_path((self,'L'))
      childs.append(left_child)

    if blank % self.size != self.size -1: #swap right move
      right_state = self.state.copy()
      right_state[blank], right_state[blank +1] = right_state[blank +1], right_state[blank]
      right_child = Node(right_state, child_distance, goal_state, h_function, self.size)
      right_child.set_path((self,'R'))
      childs.append(right_child)

    if blank >= self.size: #swap above
      above_state = self.state.copy()
      above_state[blank], above_state[blank - self.size] = above_state[blank-self.size], above_state[blank]
      above_child = Node(above_state, child_distance, goal_state, h_function, self.size)
      above_child.set_path((self,'A'))
      childs.append(above_child)

    
    if blank < self.size**2 - self.size:#swap below
      below_state = self.state.copy()
      below_state[blank], below_state[blank+self.size] = below_state[blank + self.size], below_state[blank]
      below_child = Node(below_state, child_distance, goal_state, h_function, self.size)
      below_child.set_path((self,'B'))
      childs.append(below_child)
      
    return childs
  
  def get_depth(self):
    return self.disatant_from_start
  def set_path(self, parent):
    self.parent = parent
  def get_path(self):
    return self.parent
  def fg_sum(self):
    return self.disatant_from_start + self.heuristic
  

  def state_checking(self, another_state):
    for i in range(len(self.state)):
      if(self.state[i]) != another_state[i]:
        return False
    return True
  def state_print(self):
    for i , tile in enumerate(self.state):
      if i%self.size == 0 and i != 0:
          print()
      print(tile if tile != 0 else 'b', end=' ')
    print()
    print()

  def g_n(self):
    return self.disatant_from_start

  def h_n(self):
    return self.heuristic

def answer(h_function, initial, size):
  max_queue_size = 0
  expand_cnt = 0
  depth = 0
  
  goal_state = sorted(initial, key=lambda x:(x==0, x)) 
  #since the sort make the first number is 0
  #this push 0 to the end, whih which is what we want as goal state
  visited = set() #set of visited nodes
  frontier = []


  if (goal_state == initial):
    print("It's already a correct answer")
  counter = count()

  init_node = Node(initial, 0, goal_state, h_function, size)
  heapq.heappush(frontier, (init_node.fg_sum(), next(counter), init_node))
  while True:
    if(len(frontier) == 0):
      print("invalid")
      return (expand_cnt, max_queue_size, depth)
    
    _, _, curr_state = heapq.heappop(frontier) #only keep curr_state part

    if expand_cnt >= 1:
      print(f"The best state to expand with g(n) = {curr_state.g_n()} and h(n) = {curr_state.h_n()} is ")
    else:
      print("Expanding to state")
      
    curr_state.state_print()
    expand_cnt += 1

    #expanding to neighbors
    route = curr_state.children(goal_state, h_function)
    
    #check if we reach the goal
    for path in route:
      if path.state_checking(goal_state):
        depth = path.get_depth()
        print("GOALLLLLL!!")
        path.state_print()
        return(expand_cnt, max_queue_size, depth)
      

    #mark curr_state as visisted
    visited.add(curr_state)


    #see if other neighbor is repeated state, if not, add into the heap
    for path in route:
      if path not in visited and all(path!= node[2] for node in frontier):
        heapq.heappush(frontier, (path.fg_sum(), next(counter), path))
    
    max_queue_size = max(max_queue_size, len(frontier))


def main():
  choice = int (input("Welcome to chsu115 and rjour001 8 puzzle solver. Type “1” to use a default puzzle, or “2” to enter your own puzzle: "))
  puzzle_size = 3
  init_state = [1,0,3,4,2,6,7,5,8]
  if choice == 2:
    for i in range (puzzle_size):
      values = None 
      if (i+1) %10 == 1: #making the rows
        values = input(f"Enter the {i+1}st row: ").split()
      elif (i+1)%10 ==2:
        values = input(f"Enter the {i+1}st row: ").split()
      elif (i+1) %10 == 3:
        values = input(f"Enter the {i+1}st row: ").split()
      else:
        values = input(f"Enter the {i+1}st row: ").split()
      row = [int(num) for num in values]
      init_state.extend(item for item in row)
  print("Enter your choice of algorithm \n"
          "1) Uniform Cost Search\n"
          "2) A* with the Misplaced Tile heuristic.\n"
          "3) A* with the Euclidean distance heuristic.\n")
  #store the state as array with size 1 * w and 1d
  choice = int(input())

  #lambda functions
  zero_h = lambda state, goal, colums = None:0 
  misplaced_h = lambda state, goal, colums = None:sum(1 for i , tile in enumerate(state) if  tile != goal [i] and tile != 0)
  euclid_h = lambda state, goal, colums: euclid_distance(state, goal, colums)

  rst = None
  if choice == 1:
    rst = zero_h
  elif choice == 2:
    rst = misplaced_h
  else:
    rst = euclid_h

  expanded, max_queue, depth = answer(rst, init_state, puzzle_size)

  print(f"To solve this problem the search algorithm expanded a total of {expanded} nodes.\n")
  print("The maximum number of nodes in the queue at any one time: ", max_queue)
  print("The depth of the goal node was: ", depth)


main()