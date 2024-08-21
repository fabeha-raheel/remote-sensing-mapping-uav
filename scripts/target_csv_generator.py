import csv
import rospkg
import sys

try:
    pkg_path = rospkg.RosPack().get_path('remote_sensing_mapping_uav')
except:
    print("Unable to locate package path. Try sourcing your ROS workspace.")
    sys.exit()

# Sample data
exp4 = [(212, (37.413676689373474, -121.99647061259972), 'person'), (209, (37.41362093779271, -121.99674564945715), 'person'), (139, (37.4134155777631, -121.99638845341578), 'person'), (318, (37.41333209135805, -121.99674024716238), 'person'), (58, (37.413334745753424, -121.99695889678125), 'vehicle'), (241, (37.41336760786671, -121.99698083326444), 'vehicle'), (169, (37.413287430238896, -121.99695034979851), 'vehicle'), (228, (37.41334008993654, -121.99706553910417), 'vehicle')]
exp5 = [(72, (37.41367550285474, -121.99646905372035), 'person'), (197, (37.41364610918951, -121.99642665218192), 'person'), (10, (37.41355187742603, -121.99627992229362), 'person'), (40, (37.41348870384002, -121.99628154415358), 'person'), (62, (37.41368707414824, -121.99666251344262), 'person'), (152, (37.413625531033944, -121.99673834689855), 'person'), (178, (37.413609678857355, -121.99668693836534), 'person'), (56, (37.413424525037165, -121.99638174918158), 'person'), (73, (37.41354619165582, -121.99659049150041), 'person'), (130, (37.41342889760671, -121.99641371129572), 'person'), (79, (37.4133399359941, -121.9967321489684), 'person'), (108, (37.41332983195918, -121.99669352012315), 'person'), (68, (37.413115693728614, -121.99655658191054), 'vehicle'), (22, (37.413317326591816, -121.99694942196997), 'vehicle'), (154, (37.41333438982595, -121.99699916778413), 'vehicle'), (130, (37.41311477936239, -121.99651517150882), 'vehicle'), (220, (37.41327706983022, -121.99697115036577), 'vehicle')]

data = exp5
file_name = 'exp5'

# File name
csv_file = pkg_path + '/extras/' + file_name + '.csv'

# Writing to csv file
with open(csv_file, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['lat', 'lng', 'name', 'color', 'note'])  # Header row

    for item in data:
        d, (lat, lng), name = item
        writer.writerow([lat, lng, name, 'blue', ''])

print(f"Data has been written to {csv_file}")
