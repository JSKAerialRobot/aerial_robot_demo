#!/usr/bin/env python
import rospy
import yaml
import os
from spinal.msg import Gps

def recordGpsPoints(point_number, is_height=False, default_height=None):
    data = []
    for i in range(point_number):
        while True:
            print('Getting data for point '+ str(i+1) + '/' +str(point_number) + '. Enter to proceed')
            raw_input()
            loc = getGpsLocation()
            if is_height:
                if type(default_height) in {int, float}:
                    height = default_height
                else:
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
    return [msg.location[0], msg.location[1]]

def addParamFromInput(data, private_param_name, preset_default_value, type_cast_function):
    default_value = rospy.get_param('~'+private_param_name, preset_default_value)
    print('Enter value for '+private_param_name+' (or skip for default: '+str(default_value) +' )')
    input_value = raw_input()
    if not input_value:
        value = default_value
    else:
        value = input_value
    data[private_param_name] = type_cast_function(value)
    return data

def main():
    rospy.init_node('record_waypoints', anonymous=True)

    tmp_data = {}
    addParamFromInput(tmp_data, 'waypoint_number', '1', int)
    addParamFromInput(tmp_data, 'search_height', '1', int)
    waypoints = []
    waypoints.append(recordGpsPoints(tmp_data['waypoint_number'], True, tmp_data['search_height']))
    data = {}
    data['initial_waypoints'] = waypoints

    print('>>>>>>>>>>>>> Output:')
    print('')
    print(yaml.dump(data, default_flow_style=False))
    print('')
    print('<<<<<<<<<<<<<')

    filename = rospy.get_param('~output_filename', 'waypoints.yaml')
    f = open(filename, "wr")
    f.write(yaml.dump(data, default_flow_style=False))
    f.close
    print('Written to: '+os.getcwd()+'/'+filename)

if __name__ == '__main__':
    main()
