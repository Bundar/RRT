import numpy as np


if __name__ == '__main__':
	# from rrt import isCollisionFree
	from rrt import isCollisionFree
	# f = open('gr.csv', 'a')
	tot = 0
	''' two simple tests'''
	if isCollisionFree([(0,0), (0,2), (2,2), (2,0)], (4,4), [[(5,5),(2,4),(1,5),(2,6)]]) == False:
		tot += 5
	else:
		print("1 Should be False")
	print("1 ",tot)
	if isCollisionFree([(0,0), (3,1), (4,1)], (2,4), [[(3,3),(3,4),(4,4),(4,3)], [(3,5),(3,6),(4,6),(4,5)]]) == True:
		tot += 5
	else:
		print("2 Should be True")
	''' exactly same '''
	print("2 ",tot)
	if isCollisionFree([(0,0), (0,2), (2,2), (2,0)], (4,4), [[(4,4),(4,6),(6,6),(6,4)]]) == False:
		tot += 5
	else:
		print("3 Should be False")
	# ''' cut in coordinate'''
	print("3 ",tot)
	if isCollisionFree([(0,0), (0,2), (2,2),(2,0)], (4,4), [[(4,4),(2,6),(4,8),(6,6)]]) == False:
		tot += 5
	else:
		print("4 Should be False")
	''' 2 tangent test '''
	print("4 ",tot)
	if isCollisionFree([(0,0), (0,2), (2,2), (2,0)], (4,4), [[(5,6), (5,8), (7,8), (7,6)]]) == True:
		tot += 5
	else:
		print("5 Should be True")
	print("5 ",tot)
	if isCollisionFree([(0,0), (0,2), (2,2),(2,0)], (4,4), [[(4,5), (4,7), (6,7), (6,5)]]) == False:
		tot += 5
	else:
		print("6 Should be False")
	print("6 ",tot)
	''' completely contained'''
	if isCollisionFree([(0,0), (0,4), (4,4), (4,0)],(3,3), [[(4,4), (4,6), (6,6), (6,4)]]) == False:
		tot += 5 #obs is inside robot
	else:
		print("7 Should be False")
	print("7 ",tot)
	if isCollisionFree([(0,0), (0,2), (2,2), (2,0)],(4,4), [[(3,3), (3,7), (7,7), (7,3)]]) == False:
		tot += 5 # robot inside obs
	else:
		print("8 Should be False")
	print("8 ",tot)
	'''Concave obstacle'''
	if isCollisionFree([(0,0), (0,2), (2,2),(2,0)], (4,4), [[(4,4),(2,8),(6,6),(3,7)]]) == True:
		tot += 5 # tangent overlap
	else:
		print("9 Should be True")
	print("9 ",tot)
	if isCollisionFree([(0,0), (0,4), (4,4),(4,0)], (3,3), [[(3,3),(2,8),(7,7),(4,6)]]) == False:
		tot += 5
	else:
		print("10 Should be False")
	# f.write(str(tot)+'\n')
	print("10 ",tot)