import numpy as np
import random as rdm

import landmark as lm

import rospy as rp



NUM_LANDMARKS_SQRT = 10
XLIM = [float(elem) for elem in rp.get_param('xlim', "-0.4 2.4").split()]
YLIM = [float(elem) for elem in rp.get_param('ylim', "-2.0 1.4").split()]

rp.logwarn(XLIM)
rp.logwarn(YLIM)

NAMES = rp.get_param('/names').split()

LANDMARKS = dict()
for name in NAMES:
	LANDMARKS[name] = set()

rdm.seed(89)

for index in range(NUM_LANDMARKS_SQRT**2):
	x = float(index%NUM_LANDMARKS_SQRT)*(
		XLIM[1]-XLIM[0])/NUM_LANDMARKS_SQRT+XLIM[0]
	y = float(index//NUM_LANDMARKS_SQRT)*(
		YLIM[1]-YLIM[0])/NUM_LANDMARKS_SQRT+YLIM[0]
	pos = (x,y)
	ori = (1,0)
	lmk = lm.Landmark(pos, ori)
	#landmarks[rdm.choice(landmarks.keys())].add(lmk)
	LANDMARKS[NAMES[0]].add(lmk)


if __name__ == '__main__':
	print landmarks
