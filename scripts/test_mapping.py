import pickle
import rospkg
import sys

try:
    pkg_path = rospkg.RosPack().get_path('remote_sensing_mapping_uav')
except:
    print("Unable to locate package path. Try sourcing your ROS workspace.")
    sys.exit()

LOG_FILEPATH = pkg_path + r'/logs/test_data.pickle'


# test_data = [(1, (37.4136171,-121.9963503),"person"), 
#              (2,(37.4137407,-121.9966990),"person"), 
#              (3,(37.4133402,-121.9965327),"car"), 
#              (4, (37.4132891,-121.9971442),"person")]

test_data = [(57, (24.146700, 47.270541),"landmine 1"), 
             (20,(24.146595,47.270596),"landmine 2"), 
             (30,(24.146557,47.270543),"landmine 3")]

def write_to_log(data):

    f = open(LOG_FILEPATH, 'wb')
    pickle.dump(data, f)
    f.close()

write_to_log(test_data)