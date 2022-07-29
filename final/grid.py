from random import randint
import serial #you need to install this first in your laptop (e.g., pip install pyserial)

def manhattan(i, j, n):
	x1 = i // n
	x2 = j // n
	y1 = i % n
	y2 = j % n
	return abs(x1 - x2) + abs(y1 - y2)

def init_walls(n):
	global dirs
	ma = [set() for i in range(n * n)]
	for i in range(n * n):
		x = i // n
		y = i % n
		for s in range(4):
			tx = x + dirs[s]
			ty = y + dirs[s + 1]
			if(tx < 0 or ty < 0 or tx >= n or ty >= n):
				continue
			ma[i].add(int(tx * n + ty + 1e-8))
	return ma

#up right down left
path = []
dirs = [-1,0,1,0,-1]
control_set = dict()
num_stars = 0
end_game = False
catchphrase = ['In Africa, there are 60 seconds in a minute',
				'Everyone who dares to stop me is my father',
				'Yield to the dark power and you will become powerful too',
				'My kid bears such resemblance with my best friend']
success_response = ['Bad one',
				'Terrible gameplay',
				'Invalid human',
				'Lack of free will']
def dfs(cur, star, n):
	global dirs
	global path
	#print(path)
	if(cur == star):
		return True
	x = cur // n
	y = cur % n
	manhattan_pos_pair = []
	for s in range(4):
		tx = x + dirs[s]
		ty = y + dirs[s + 1]
		if(tx < 0 or ty < 0 or tx >= n or ty >= n):# or (tx * n + ty) in path):
			continue
		#print(x,y,tx,ty)
		manhattan_pos_pair.append([manhattan(tx * n + ty, star, n), tx * n + ty])
	manhattan_pos_pair.sort(key = lambda e : e[0])
	candidates = len(manhattan_pos_pair)
	if(candidates == 0):
		return False
	elif(candidates == 1):
		path.append(manhattan_pos_pair[0][1])
		if(dfs(manhattan_pos_pair[0][1], star, n)):
			return True
		else:
			path.pop()
			return False
	elif(candidates == 2):
		rand = randint(0, 5)
		if(rand >= 4):
			tmp = manhattan_pos_pair[0]
			manhattan_pos_pair[0] = manhattan_pos_pair[1]
			manhattan_pos_pair[1] = tmp
		for tl in manhattan_pos_pair:
			path.append(tl[1])
			if(dfs(tl[1], star, n)):
				return True
			else:
				path.pop()
		return False

	else:
		rand = randint(0, 5)
		if(rand == 5):
			tmp = manhattan_pos_pair[2]
			manhattan_pos_pair[2] = manhattan_pos_pair[1]
			manhattan_pos_pair[1] = manhattan_pos_pair[0]
			manhattan_pos_pair[0] = tmp
		elif(rand >= 3):
			tmp = manhattan_pos_pair[0]
			manhattan_pos_pair[0] = manhattan_pos_pair[1]
			manhattan_pos_pair[1] = tmp

		for tl in manhattan_pos_pair:
			path.append(tl[1])
			if(dfs(tl[1], star, n)):
				return True
			else:
				path.pop()
		return False

def gen_path(start, star, n, walls):
	global path
	path.append(start)
	dfs(start, star, n)
	for i in range(len(path) - 1):
		p = path[i]
		q = path[i + 1]
		if q in walls[p]:
			walls[p].remove(q)
			walls[q].remove(p)
	#print(path)
	st = path[0] if len(path) == 0 else path[randint(0, len(path) - 1)]
	#TODO: change
	#st = 0
	path = []
	return st


def generate_grid(n, m):
	grid = [[' ' for i in range(n)] for j in range(n)]
	#print(grid)
	arr = init_walls(n)
	start = n * n - 1
	grid[n - 1][n - 1] = '@'
	stars = set()
	for i in range(m):
		rand = randint(0, n * n - 2)
		while(rand in stars) or manhattan(rand, start, n) < (int)(n * 0.8):
			rand = randint(0, n * n - 2)
		stars.add(rand)
		sx = rand // n
		sy = rand % n
		grid[sx][sy] = '✶'
	#print(stars)
	for star in stars:
		start = gen_path(start, star, n, arr)

	return [grid, arr]

def print_grid(grid, walls, show_star, pos):
	n = len(grid)
	for i in range(n):
		print('· - ', end = "")
	print('·')
	for i in range(n):
		print('|', end = "")
		for j in range(n):
			print(' ', end = "")
			if(show_star == False and grid[i][j] not in [' ', '@'] and (abs(i - pos // n) > 1 or abs(j - pos % n) > 1)):
				print(' ', end = "")
			else:
				print(grid[i][j], end = "")
			print(' ', end = "")
			if(i * n + j + 1 in walls[i * n + j] or j == n - 1):
				print('|', end = "")
			else:
				print(' ', end = "")
		print()
		for j in range(n):
			print('· ', end = "")
			if(i * n + j + n in walls[i * n + j] or i == n - 1):
				print('-', end = "")
			else:
				print(' ', end = "")
			print(' ', end = "")
		print('·')

def print_grid_limited(grid, walls, pos):
	global dirs
	n = len(grid)
	x = pos // n
	y = pos % n
	limited_walls = [set() for i in range(n * n)]
	for i in range(max(0, x - 1), min(n, x + 2)):
		for j in range(max(0, y - 1), min(n, y + 2)):
			ind = i * n + j
			for nei in walls[ind]:
				limited_walls[ind].add(nei)
	print_grid(grid, limited_walls, False, pos)

def move(grid, walls, dirl):
	global dirs
	global num_stars
	global end_game
	global control_set
	if(dirl not in control_set):
		print("what is wrong with your command?")
		return False
	direction = control_set[dirl]
	x = 0
	y = 0
	n = len(grid)
	for i in range(n):
		for j in range(n):
			if(grid[i][j] == '@'):
				x = i
				y = j
	#grid[x][y] = ' '
	tx = x + dirs[direction]
	ty = y + dirs[direction + 1]
	if(tx < 0 or ty < 0 or tx >= n or ty >= n or (tx * n + ty) in walls[x * n + y]):	
		return False
	else:
		grid[x][y] = ' '
		b = (grid[tx][ty] not in [' ', '@'])
		grid[tx][ty] = '@'
		print_grid_limited(grid, walls, tx * n + ty)
		if(b):
			num_stars -= 1
			response_ind = randint(0, len(success_response) - 1)
			print(success_response[response_ind])
			success_response.pop(response_ind)
			if(num_stars > 0):
				print()
				print()
				print("Number of stars remaining: ", num_stars)
			else:
				print("But You win!")
				end_game = True
		return True

def main():
	global num_stars
	global end_game
	global control_set
	control_set['w'] = 0
	control_set['d'] = 1
	control_set['s'] = 2
	control_set['a'] = 3
	print("Maze Game ha. Select a mode by typing in 'easy', 'medium', or 'hard'! \n\nQuick! Make a decision or you have failed!\n")
	while(True):
		val = input("Difficulty selection: ")
		if(val == 'easy'):
			n = 5
			m = 1
			break
		elif(val == 'medium'):
			n = 7
			m = 2
			break
		elif(val == 'hard'):
			n = 9
			m = 2
			break
		else:
			s = catchphrase[0]
			print()
			print(s)
			print()
			catchphrase.pop(0)
			catchphrase.append(s)
	num_stars = m
	grid_arr = generate_grid(n, m)
	grid = grid_arr[0]
	walls = grid_arr[1]
	#print_grid(grid, walls)
	print_grid_limited(grid, walls, n * n - 1)
	print("control by inputting 'w', 'a', 's', 'd'")
	print("Number of stars:", m)
	while(True):
		#val = arduino.read(size = 1)
		val = input("command: ")
		move(grid, walls, val)
		if(end_game):
			break
	#print_grid_limited(grid, walls, n * n // 2)

if __name__ == '__main__':
	main()
