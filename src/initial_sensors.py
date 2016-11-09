import numpy as np
import sensor as sn
import footprints as fp
import rospy as rp


NAMES = rp.get_param('/names').split()


sensors = dict()
#sensors["Axel"] = sn.Sensor(fp=fp.OmnidirectionalFootprint(gain=0.2))
#sensors["Bo"] = sn.Sensor(fp=fp.OmnidirectionalFootprint(gain=0.2))
#sensors["Calle"] = sn.Sensor(fp=fp.EggFootprint(best_distance=1.0))
#sensors["David"] = sn.Sensor(fp=fp.EggFootprint(best_distance=2.0))

sensors[NAMES[0]] = sn.Sensor(fp=fp.EggFootprint(best_distance=0.5))
sensors[NAMES[1]] = sn.Sensor(fp=fp.EggFootprint(best_distance=1.0))
sensors[NAMES[2]] = sn.Sensor(fp=fp.OmnidirectionalFootprint(gain=0.5))
sensors[NAMES[3]] = sn.Sensor(fp=fp.OmnidirectionalFootprint(gain=0.5))

#sensors["Axel"] = sn.Sensor(fp=fp.EggFootprint())
#sensors["Bo"] = sn.Sensor(fp=fp.EggFootprint())
#sensors["Calle"] = sn.Sensor(fp=fp.EggFootprint())
#sensors["David"] = sn.Sensor(fp=fp.EggFootprint())
