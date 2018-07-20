import math, sys

def CosClamped(n, L, U):
	#Input number, Lower bound, upper bound
	if n < L:
		return 1#Lower than lower bound, return 1
	elif n > U:
		return -1#Higher than upper bound, return -1
	elif n > ((U-L*1.0)/2)-((U-L)*0.1) and n < ((U-L*1.0)/2)+((U-L)*0.1):
		return 0
	else:
		percent = (n - L)*1.0/(U-L)#Percent between lower and upper bound, multiplied by 1.0 cos python is stupid
		cosvalue = math.radians(percent * 180)#Scale between 0 and pi rad
		value = math.cos(cosvalue)#Cos the value
		return value

print(sys.argv[1])
print str(CosClamped(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])))
