from heapq import heappush,heappop
from collections import deque
import heapq
import os
import time
import sys
from queue import PriorityQueue
import numpy as np
import queue
import math

initial_cost=float('inf')
directions=[[0,1],[1,0],[-1,0],[0,-1],[1,1],[1,-1],[-1,-1],[-1,1]]

def heuristic(node1,node2):
		x1,y1=node1[0],node1[1]
		x2,y2=node2[0],node2[1]
		dx=(abs(x1-x2))**2
		dy=(abs(y1-y2))**2
		e_dist=math.sqrt(dx+dy)
		return(e_dist)

def single_execution(grid):
	pass

class Input_Path(object):
	def __init__(self,input_data:str):
		file1=open("input.txt",'r').read()
		text=file1.split("\n")
		self.inp=str(input_data)
		self.algorithm=str(text[0])
		self.col,self.row=[int(i) for i in text[1].split(" ")]
		self.start=tuple([int(i) for i in text[2].split(" ")])
		self.max_height=int(text[3])
		self.no_of_sites=int(text[4])
		self.sites_list=[tuple([int(j) for j in i.split(' ')[:2]]) for i in text[5:5+self.no_of_sites]]
		self.grid=[[int(column) for column in r.split()[0:self.col]] for r in text[5+self.no_of_sites:5+self.no_of_sites+self.row]]
		outstring=""
		outstring+="FAIL\n"

	def grid_input(self):
		return(self.grid)


	def All_Path_Algorithm(self):
		if(self.algorithm=="BFS"):
			result=BFS(self.inp).calculate()
			file2=open("output.txt",'w')
			for i in result:
				if(i=="FAIL"):
					file2.write("FAIL")
					file2.write('\n')
				else:
					for j in i:
						re=str(j[0])+","+str(j[1])+" "
						file2.write(re)
					file2.write('\n')
		elif(self.algorithm=="UCS"):
			pass
			result=UCS(self.inp).calculate()
			file2=open("output.txt",'w')
			for i in result:
				if(i=="FAIL"):
					file2.write("FAIL")
					file2.write('\n')
				else:
					for j in i:
						re=str(j[0])+","+str(j[1])+" "
						file2.write(re)
					file2.write('\n')
		elif(self.algorithm=="A*"):
			result=astar(self.inp).calculate()
			file2=open("output.txt",'w')
			for i in result:
				if(i=="FAIL"):
					file2.write("FAIL")
					file2.write('\n')
				else:
					for j in i:
						re=str(j[0])+","+str(j[1])+" "
						file2.write(re)
					file2.write('\n')



#Breadth First Search
class BFS(Input_Path):
	class cost_determine:
		def __init__(self,p=None,pos=None):
			self.p=p;self.pos=pos
			self.cost=0
			self.g=0
			self.h=0
			self.f=0
		def __eq__(self,o:object):
			return(self.pos==o.pos)

	def __init__(self,inp):
		self.grid1=directions
		super().__init__(inp)
		self.grid1=directions

	def jump(self,grid_point,neigbhor_point,grid1_point,neigbhor1_point):
		row=len(self.grid)
		col=len(self.grid[0])
		c=[(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
		c1=[(row-1,col-1),(row+1,col-1),(row-1,col+1),(row+1,col+1)]
		x1=grid_point
		x2=grid1_point
		y1=neigbhor_point
		y2=neigbhor1_point
		if((x1+y1<=self.col-1  and x1+y1>=0 and x2+y2<=self.row-1 and x2+y2>=0)==False):
			return(False)
		if(self.grid[x2+y2][x1+y1]<0):
			d1=abs(self.grid[x2+y2][x1+y1])
		else:
			d1=0
		if(self.grid[x2][x1]<0):
			d2=abs(self.grid[x2][x1])
		else:
			d2=0
		if((abs(d1-d2))<=self.max_height)==False:
			return(False)
		return(True)

	def calculate(self):
		if not self.grid: 
			return
		row=len(self.grid)
		col=len(self.grid[0])
		final=[]
		c=[(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
		c1=[(row-1,col-1),(row+1,col-1),(row-1,col+1),(row+1,col+1)]
		flag=0
		res=0
		for i in self.sites_list:
			dest_list=[self.cost_determine(None,i)]
			for dest in dest_list:
				q=deque()
				q.append(self.cost_determine(None,self.start))
				flag=False
				visited=set()
				while q:
					curr_node=q.popleft()
					visited.add(curr_node.pos)
					if(curr_node==dest):
						final.append(self.path_find(curr_node))
						flag=True
						break
					for neigbhors in [(0,1),(0,-1),(-1,0),(1,0),(-1,-1),(-1, 1),(1,-1),(1,1)]:
						if(self.jump(curr_node.pos[0],neigbhors[0],curr_node.pos[1],neigbhors[1]))==True:
							jump_node=(curr_node.pos[0]+neigbhors[0],curr_node.pos[1]+neigbhors[1])
							if jump_node in visited: 
								continue
							new_node=self.cost_determine(curr_node,jump_node)
							visited.add(jump_node)
							q.append(new_node)
					res+=1
			if not flag: 
				final.append("FAIL")
		return(final)

	def path_find(self,current):
		path=[]
		while current:
			path.append(current.pos)
			current=current.p
		return(path[::-1])


#Uniform Cost Search
class UCS(Input_Path):
	class cost_determine:
		def __init__(self,p=None,pos=None,cost=0):
			self.p=p;self.pos=pos
			self.cost=0
			self.g=0
			self.h=0
			self.f=0
		def __eq__(self, o: object):
			return(self.pos==o.pos)
		def __gt__(self,o:object):
			return(self.cost>o.cost)
		def __lt__(self,o:object):
			return(self.cost<o.cost)
		def __ge__(self,o:object):
			return(self.cost>=o.cost)
		def __le__(self,o:object):
 			return(self.cost<=o.cost)

	def __init__(self,inp):
		self.grid1=directions
		super().__init__(inp)
		self.grid1=directions

	def jump(self,grid_point,neigbhor_point,grid1_point,neigbhor1_point):
		row=len(self.grid)
		col=len(self.grid[0])
		c=[(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
		c1=[(row-1,col-1),(row+1,col-1),(row-1,col+1),(row+1,col+1)]
		x1=grid_point
		x2=grid1_point
		y1=neigbhor_point
		y2=neigbhor1_point
		if((x1+y1<=self.col-1  and x1+y1>=0 and x2+y2<=self.row-1 and x2+y2>=0)==False):
			return(False)
		if(self.grid[x2+y2][x1+y1]<0):
			d1=abs(self.grid[x2+y2][x1+y1])
		else:
			d1=0
		if(self.grid[x2][x1]<0):
			d2=abs(self.grid[x2][x1])
		else:
			d2=0
		if((abs(d1-d2))<=self.max_height)==False:
			return(False)
		return(True)
		
	def calculate(self):
		if not self.grid: 
			return
		final=[]
		res=0
		row=len(self.grid)
		col=len(self.grid[0])
		c=[(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
		c1=[(row-1,col-1),(row+1,col-1),(row-1,col+1),(row+1,col+1)]
		for i in self.sites_list:
			dest_list=[self.cost_determine(None,i,initial_cost)]
			for dest in dest_list:
				q=queue.PriorityQueue()
				q.put(self.cost_determine(None,self.start,0))
				visited=set()
				flag=False
				while not q.empty():
					curr_node=q.get()
					cost=curr_node.cost
					visited.add(curr_node.pos)
					if(curr_node==dest):
						final.append(self.path_find(curr_node))
						flag=True
						break
					for neigbhors in [(0,-1),(0,1),(-1,0),(1,0)]:
						if(self.jump(curr_node.pos[0],neigbhors[0],curr_node.pos[1],neigbhors[1]))==True:
							jump1=(curr_node.pos[0]+neigbhors[0],curr_node.pos[1]+neigbhors[1])
							if jump1 not in visited: 
								visited.add(jump1)
								new_node=self.cost_determine(curr_node,jump1)
								new_node.cost=cost+10
								q.put(new_node)
					for neigbhors in [(-1,-1),(-1,1),(1,-1),(1,1)]:
						if(self.jump(curr_node.pos[0],neigbhors[0],curr_node.pos[1],neigbhors[1]))==True:
							jump1=(curr_node.pos[0]+neigbhors[0],curr_node.pos[1]+neigbhors[1])
							if jump1 not in visited: 
								visited.add(jump1)
								new_node=self.cost_determine(curr_node,jump1)
								new_node.cost=cost+14
								q.put(new_node)
					res+=1
			if not flag: 
				final.append("FAIL")
		return(final)
	
	def path_find(self,current):
		path=[]
		while current:
			path.append(current.pos)
			current=current.p
		return path[::-1]

#Astar
class astar(Input_Path):
	class cost_determine:
		def __init__(self,p=None,pos=None):
			self.p=p;self.pos=pos
			self.cost=0
			self.g,self.h,self.f=0,0,0
		def __eq__(self, o: object):
			return(self.pos==o.pos)
		def __gt__(self,o:object):
			return(self.f>o.f)
		def __lt__(self,o:object):
			return(self.f<o.f)
		def __ge__(self,o:object):
			return(self.f>=o.f)
		def __le__(self,o:object):
			return(self.f<=o.f)

	def __init__(self,inp):
		self.grid1=directions
		super().__init__(inp);self.inp=inp
		self.grid1=directions


	def jump(self,grid_point,neigbhor_point,grid1_point,neigbhor1_point):
		row=len(self.grid)
		col=len(self.grid[0])
		c=[(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
		c1=[(row-1,col-1),(row+1,col-1),(row-1,col+1),(row+1,col+1)]
		x1=grid_point
		x2=grid1_point
		y1=neigbhor_point
		y2=neigbhor1_point
		if((x1+y1<=self.col-1  and x1+y1>=0 and x2+y2<=self.row-1 and x2+y2>=0)==False):
			return(False)
		if(self.grid[x2+y2][x1+y1]<0):
			d1=abs(self.grid[x2+y2][x1+y1])
		else:
			d1=0
		if(self.grid[x2][x1]<0):
			d2=abs(self.grid[x2][x1])
		else:
			d2=0
		if((abs(d1-d2))<=self.max_height)==False:
			return(False)
		return(True)


	def calculate(self):
		if not self.grid: 
			return
		flag=0
		res=0
		final=[]
		row=len(self.grid)
		col=len(self.grid[0])
		c=[(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
		c1=[(row-1,col-1),(row+1,col-1),(row-1,col+1),(row+1,col+1)]
		for i in self.sites_list:
			dest_list=[self.cost_determine(None,i)]
			for dest in dest_list:
				open_list=queue.PriorityQueue()
				start=self.cost_determine(None,self.start)
				open_list.put(start)
				visited=set()
				flag=False
				visited=set()
				closed_list={}
				closed_list[start.pos]=0
				while not open_list.empty():
					current_node=open_list.get()
					visited.add(current_node.pos)
					if current_node.pos in closed_list: 
						del closed_list[current_node.pos]
					if(current_node==dest):
						final.append(self.path_find(current_node))
						flag=True
						break
					child=[]
					for neigbhors in [(0,-1),(0,1),(-1,0),(1,0)]:
						if self.jump(current_node.pos[0],neigbhors[0],current_node.pos[1],neigbhors[1]):
							new_child=self.cost_determine(current_node,(current_node.pos[0]+neigbhors[0],current_node.pos[1]+neigbhors[1]))
							new_child.g=initial_cost
							new_child.h=initial_cost
							new_child.f=initial_cost
							child.append(new_child)
					row=len(self.grid)
					col=len(self.grid[0])
					c=[(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
					c1=[(row-1,col-1),(row+1,col-1),(row-1,col+1),(row+1,col+1)]
					for neighbor in child:
						if neighbor.pos in visited: 
							continue
						current_m=self.grid[current_node.pos[1]][current_node.pos[0]]
						neighbor_node_check=self.grid[neighbor.pos[1]][neighbor.pos[0]]
						if(neighbor_node_check<0):
							mud=0
						else:
							mud=self.grid[neighbor.pos[1]][neighbor.pos[0]] 
						if(current_m<0):
							mud1=current_m
						else:
							mud1=0
						height_change=abs(mud-mud1)
						neighbor.g=current_node.g+10+mud+height_change
						if neighbor.pos in closed_list:
							if closed_list[neighbor.pos]<=neighbor.g:
								continue
						node1=(neighbor.pos[0],neighbor.pos[1])
						node2=(current_node.pos[0],current_node.pos[1])
						heur=heuristic(node1,node2)
						neighbor.h=int(math.sqrt((neighbor.pos[0]-current_node.pos[0])**2+(neighbor.pos[1]-current_node.pos[1])**2))
						neighbor.f=neighbor.g+neighbor.h
						open_list.put(neighbor)
						closed_list[neighbor.pos]=neighbor.g
					child_diagonal=[]
					for neigbhors in [(-1,-1),(-1,1),(1,-1),(1,1)]:
						if self.jump(current_node.pos[0],neigbhors[0],current_node.pos[1],neigbhors[1]):
							new_neighbor=self.cost_determine(current_node,(current_node.pos[0]+neigbhors[0],current_node.pos[1]+neigbhors[1]))
							new_neighbor.g=initial_cost
							new_neighbor.h=initial_cost
							new_neighbor.f=initial_cost
							child_diagonal.append(new_neighbor)
					row=len(self.grid)
					col=len(self.grid[0])
					c=[(row-1,col),(row+1,col),(row,col-1),(row,col+1)]
					c1=[(row-1,col-1),(row+1,col-1),(row-1,col+1),(row+1,col+1)]
					for neighbor in child_diagonal:
						if neighbor.pos in visited: 
							continue
						current_m=self.grid[current_node.pos[1]][current_node.pos[0]]
						neighbor_node_check=self.grid[neighbor.pos[1]][neighbor.pos[0]]
						if(neighbor_node_check<0):
							mud=0
						else:
							mud=self.grid[neighbor.pos[1]][neighbor.pos[0]] 
						if(current_m<0):
							mud1=current_m
						else:
							mud1=0
						height_change=abs(mud-mud1)
						neighbor.g=int(current_node.g+14+mud+height_change)
						if neighbor.pos in closed_list:
							if closed_list[neighbor.pos]<=neighbor.g:
								continue
						node3=(neighbor.pos[0],neighbor.pos[1])
						node4=(current_node.pos[0],current_node.pos[1])
						heur=heuristic(node3,node4)
						neighbor.h=int(math.sqrt((neighbor.pos[0]-current_node.pos[0])**2+(neighbor.pos[1]-current_node.pos[1])**2))
						neighbor.f=neighbor.g+neighbor.h
						open_list.put(neighbor)
						closed_list[neighbor.pos]=neighbor.g
				res+=1
			if not flag: 
				final.append("FAIL")
		return(final)

	def path_find(self,current):
		path=[]
		while current:
			path.append(current.pos)
			current=current.p
		return path[::-1]

ip=Input_Path("input_file")
ip.All_Path_Algorithm()