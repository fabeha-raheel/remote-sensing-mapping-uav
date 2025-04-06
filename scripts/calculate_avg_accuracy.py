import math

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two GPS locations.
    
    Locations should be passed as a tuple in the form (lat, long).

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2[0] - aLocation1[0]
    dlong = aLocation2[1] - aLocation1[1]
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


# runway world coordinates
# actual_coordinates = [[(-35.3631748, 149.165069), 'Person1'], [(-35.3633794, 149.1654061), 'Person2'], 
#                       [(-35.3634607, 149.1650938), 'Person3'], [(-35.3631265, 149.1653252), 'Person4'],
#                        [(-35.3634764, 149.1648519), 'Vehicle1'], [(-35.3636816, 149.1652431), 'Vehicle2']]

# baylands world coordinates
actual_coordinates = [[(37.4136691, -121.9964709), 'Person1'], [(37.4136257, -121.9967352), 'Person2'], 
                      [(37.4133371, -121.9967087), 'Person3'], [(37.4134173, -121.9963866), 'Person4'],
                       [(37.413315, -121.9969565), 'Vehicle1'], [(37.4131148, -121.9965529), 'Vehicle2']]

exp1_coordinates = []

exp2_coordinates = [[(-35.36314685, 149.1650675), 'Person1'], [(-35.36317672, 149.1650609), 'Person1'], 
                    [(-35.36337262, 149.1653936), 'Person2'], [(-35.36341834, 149.1653802), 'Person2'],
                    [(-35.3634419, 149.1650727), 'Person3'], [(-35.36347265, 149.1650798), 'Person3'],
                    [(-35.36344756, 149.1651181), 'Person3'], [(-35.3630961, 149.165324), 'Person4'],
                    [(-35.36313053, 149.1653154), 'Person4']]

exp3_coordinates = [[(37.41369928, -121.9964434), 'Person1'], [(37.41366512, -121.9964665), 'Person1'],
                    [(37.41363896, -121.9965113), 'Person1'], [(37.41360672, -121.9967646), 'Person2'],
                    [(37.41358985, -121.9966998), 'Person2'], [(37.41336091, -121.9966862), 'Person3'],
                    [(37.41331542, -121.996743), 'Person3'], [(37.41343164, -121.9963798), 'Person4'],
                    [(37.41342368, -121.9963852), 'Person4'], [(37.41335699, -121.9969708), 'Vehicle1'],
                    [(37.41335614, -121.9969229), 'Vehicle1'], [(37.41331445, -121.9969529), 'vehicle1'],
                    [(37.41312952, -121.9965644), 'Vehicle2'], [(37.41316877, -121.9965092), 'Vehicle2'],
                    [(37.41312123, -121.9964668), 'Vehicle2']]

exp4_coordinates = [[(37.413676689373, -121.9964706126), 'Person1'], [(37.413620937793, -121.99674564946), 'Person2'], 
                    [(37.413332091358, -121.99674024716), 'Person3'], [(37.413415577763, -121.99638845342), 'Person4'],
                    [(37.413334745753, -121.99695889678), 'Vehicle1'], [(37.413287430239, -121.9969503498), 'Vehicle1']]


exp5_coordinates = [[(37.413675502855, -121.99646905372), 'Person1'], [(37.41364610919, -121.99642665218), 'Person1'],
                    [(37.413687074148, -121.99666251344), 'Person2'], [(37.413625531034, -121.9967383469), 'Person2'],
                    [(37.413609678857, -121.99668693837), 'Person2'], [(37.413339935994, -121.99673214897), 'Person3'],
                    [(37.413329831959, -121.99669352012), 'Person3'], [(37.413428897607, -121.9964137113), 'Person4'],
                    [(37.413424525037, -121.99638174918), 'Person4'], [(37.413334389826, -121.99699916778), 'Vehicle1'],
                    [(37.413317326592, -121.99694942197), 'Vehicle1'], [(37.41327706983, -121.99697115037), 'Vehicle1'],
                    [(37.413115693729, -121.99655658191), 'Vehicle2'], [(37.413114779362, -121.99651517151), 'Vehicle2']]

sum = 0
count = 0
avg_accuracy = 0

for measurement in exp5_coordinates:
    if measurement[1] == 'Person1':
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[0][0])
    elif measurement[1] == 'Person2':
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[1][0])
    elif measurement[1] == 'Person3':
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[2][0])
    elif measurement[1] == 'Vehicle1':
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[4][0])
    elif measurement[1] == 'Vehicle2':
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[5][0])
    else:
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[3][0])

    count +=1

print("Avg_accuracy = ", sum/count)