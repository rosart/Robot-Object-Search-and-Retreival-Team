import os.path
import subprocess
import time

while(True):
    print 'Waiting for map....'
    time.sleep(1)
    if (os.path.isfile('/home/smartlab-tb01/catkin_ws_hydro/src/BinMen_2015/turtlebot_nav/smartlab-map/smartlab_map.yaml') and os.path.isfile('/home/smartlab-tb01/catkin_ws_hydro/src/BinMen_2015/turtlebot_nav/smartlab-map/smartlab_map.pgm')):
        subprocess.call('roslaunch turtlebot_nav mapSrv_amcl.launch', shell = True)
        break
    
