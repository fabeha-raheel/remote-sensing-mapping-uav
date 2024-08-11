import pickle
import rospkg
import sys

try:
    pkg_path = rospkg.RosPack().get_path('remote_sensing_mapping_uav')
except:
    print("Unable to locate package path. Try sourcing your ROS workspace.")
    sys.exit()

def read_data():
        f = open(pkg_path + '/logs/data.pickle', 'rb')
        data = pickle.load(f)
        print(data)
        f.close()

read_data()