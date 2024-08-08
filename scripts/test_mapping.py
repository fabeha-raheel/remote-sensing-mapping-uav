import pickle
import rospkg
import sys

try:
    pkg_path = rospkg.RosPack().get_path('remote_sensing_mapping_uav')
except:
    print("Unable to locate package path. Try sourcing your ROS workspace.")
    sys.exit()

LOG_FILEPATH = pkg_path + r'/logs/data.pickle'


# test_data = [(1, (37.4136171,-121.9963503),"person"), 
#              (2,(37.4137407,-121.9966990),"person"), 
#              (3,(37.4133402,-121.9965327),"car"), 
#              (4, (37.4132891,-121.9971442),"person")]

test_data = [(1, (37.4139324,-121.9962698),"person"), 
             (2,(37.4140603,-121.9953364),"person"), 
             (3,(37.4134382,-121.9953310),"car"), 
             (4, (37.4131570,-121.9965380),"person")]

def write_to_log(data):

    f = open(LOG_FILEPATH, 'wb')
    pickle.dump(data, f)
    f.close()

write_to_log(test_data)