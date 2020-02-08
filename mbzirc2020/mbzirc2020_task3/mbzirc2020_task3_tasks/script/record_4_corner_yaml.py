#!/usr/bin/env python
import rospy
import yaml
from spinal.msg import Gps

def recordGpsPoints(point_number, is_height=False):
    data = []
    for i in range(point_number):
        while True:
            print('Getting data for point '+ str(i+1) + '/' +str(point_number) + '. Enter to proceed')
            raw_input()
            loc = getGpsLocation()
            if is_height:
                print('Enter height: ')
                height = input()
                loc.append(height)
            print('Ok to proceed with: '+str(loc)+' ? type "y" to proceed: ')
            proceed = raw_input()
            if proceed == "y":
                data.append(loc)
                break
    return data


def getGpsLocation():
    msg = rospy.wait_for_message('/gps' , Gps)
    return [msg.location[0]*1e-7, msg.location[1]*1e-7]

def addParamFromInput(data, private_param_name, default_value, type_cast_function):
    rospy.get_param('~'+private_param_name, default_value)
    print('Enter value for '+private_param_name+' (or skip for default: '+default_value +' )')
    input_value = raw_input()
    if not input_value:
        value = default_value
    else:
        value = input_value
    data[private_param_name] = type_cast_function(value)
    return data

if __name__ == '__main__':
    rospy.init_node('record_4_corner_yaml', anonymous=True)

    print('Enter new yaml file name')
    filename = raw_input()
    f = open(filename, "wr")

    data = {}

    addParamFromInput(data, 'search_area_number', '1', int)
    initial_waypoints = []
    area_corners = []
    for i in range(data['search_area_number']):
        print('Enter Number of Initial Waypoints')
        waypoint_num = input()
        initial_waypoints.append(recordGpsPoints(waypoint_num, True))
        print('Recording 4 corners...')
        corners = []
        print('Enter mode {(a)bsolute/(r)elative}')
        mode = raw_input()
        if mode in {'r', 'relative'}:
            for i in range(4):
                tmp_data = dict()
                addParamFromInput(tmp_data, str(i+1)+"'th corner_x", '0', float)
                addParamFromInput(tmp_data, str(i+1)+"'th corner_y", '0', float)
                corners.append([tmp_data[str(i+1)+"'th corner_x"], tmp_data[str(i+1)+"'th corner_y"]])
        elif mode in {'a', 'absolute'}:
            corners = recordGpsPoints(4)
        print('Enter Trip Number')
        trip_num = input()
        corners.append(trip_num)
        area_corners.append(corners)
    data['initial_waypoints'] = initial_waypoints
    data['area_corners'] = area_corners

    print('>>>>>>>>>>>>> Output:')
    print('')
    print(yaml.dump(data, default_flow_style=False))
    print('')
    print('<<<<<<<<<<<<<')
    print('Written to: '+filename)

    f.write(yaml.dump(data, default_flow_style=False))
    f.close

