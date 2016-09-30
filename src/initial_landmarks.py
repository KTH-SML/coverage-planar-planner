import numpy as np
import random as rdm

import landmark as lm

NUM_LANDMARKS_SQRT = 25
XLIM = (-3,3)
YLIM = (-3,3)

landmarks = dict()
landmarks["Axel"] = set()
landmarks["Bo"] = set()
landmarks["Calle"] = set()
landmarks["David"] = set()

rdm.seed(89)

for index in range(NUM_LANDMARKS_SQRT**2):
	x = float(index%NUM_LANDMARKS_SQRT)*(XLIM[1]-XLIM[0])/NUM_LANDMARKS_SQRT+XLIM[0]
	y = float(index//NUM_LANDMARKS_SQRT)*(YLIM[1]-YLIM[0])/NUM_LANDMARKS_SQRT+YLIM[0]
	pos = (x,y)
	ori = (1,0)
	lmk = lm.Landmark(pos, ori)
	#landmarks[rdm.choice(landmarks.keys())].add(lmk)
	landmarks["Axel"].add(lmk)


if __name__ == '__main__':
	print landmarks
