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


actual_coordinates = [[(-35.3631748, 149.165069), 'Person1'], [(-35.3633794, 149.1654061), 'Person2'], 
                      [(-35.3634607, 149.1650938), 'Person3'], [(-35.3631265, 149.1653252), 'Person4'],
                       [(-35.3634764, 149.1648519), 'Vehicle1'], [(-35.3636816, 149.1652431), 'Vehicle2']]

exp1_coordinates = []

exp2_coordinates = [[(-35.36314685, 149.1650675), 'Person1'], [(-35.36317672, 149.1650609), 'Person1'], 
                    [(-35.36337262, 149.1653936), 'Person2'], [(-35.36341834, 149.1653802), 'Person2'],
                    [(-35.3634419, 149.1650727), 'Person3'], [(-35.36347265, 149.1650798), 'Person3'],
                    [(-35.36344756, 149.1651181), 'Person3'], [(-35.3630961, 149.165324), 'Person4'],
                    [(-35.36313053, 149.1653154), 'Person4']]

sum = 0
count = 0
avg_accuracy = 0

for measurement in exp2_coordinates:
    if measurement[1] == 'Person1':
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[0][0])
    elif measurement[1] == 'Person2':
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[1][0])
    elif measurement[1] == 'Person3':
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[2][0])
    else:
        sum = sum + get_distance_metres(measurement[0], actual_coordinates[3][0])

    count +=1

print("Avg_accuracy = ", sum/count)