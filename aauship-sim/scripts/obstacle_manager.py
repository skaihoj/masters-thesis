#!/usr/bin/env python
import rospy

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.msg import  ModelState
from geometry_msgs.msg import Pose, Twist, Quaternion
from tf.transformations import quaternion_from_euler,  quaternion_multiply


xml = """<?xml version='1.0'?>
      <sdf version='1.4'>
        <model name="my_robot">
        <static>false</static>

          <link name='chassis'>
            <pose>0 0 .1 0 0 0</pose>

            <collision name='collision'>
              <geometry>
                <box>
                  <size>{0} {1} .1</size>
                </box>
              </geometry>
            </collision>

            <visual name='visual'>
              <geometry>
                <box>
                  <size>{0} {1} .1</size>
                </box>
              </geometry>
            </visual>
          </link>

      </model>
    </sdf>
"""

def nothing():
    pass
spawn_gazebo = nothing


obs_list = []
obs_num = 0
timestep = 0.1
pub = []

class Obstacle:
    def __init__(self, pos, vel, size, name):
        self.pos = pos
        self.vel = vel
        self.size = size
        self.name = name
    
    def update_obs(self):
        delta_x = self.vel*self.pos.orientation.z*timestep
        delta_y = self.vel*self.pos.orientation.w*timestep
        print ('update of ' + self.name + ' position ' + str(self.pos.position.x) + ' ' + str(self.pos.position.y))
        self.pos.position.x = self.pos.position.x + delta_x
        self.pos.position.y = self.pos.position.y + delta_y
        state = ModelState(self.name, self.pos, Twist(), '')
        pub.publish(state)
        
        
        
    
def spawn(pos, vel, size):
    global obs_num
    new_xml = xml.format(size[0], size[1])
    spawn_gazebo('obs_'+str(obs_num), new_xml, '/', pos, '')
    obs_list.append(Obstacle(pos, vel, size, 'obs_'+str(obs_num)))
    obs_num = obs_num + 1

def do_updates(test):
    print(test)
    print('start of updates')
    for o in obs_list:
        o.update_obs()
    
    print('end of updates')


if __name__ == "__main__":
    rospy.init_node('obstacle_manager')
    pub = rospy.Publisher('gazebo/set_model_state', ModelState, queue_size = 1000)
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_gazebo = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    

    for i in range(10):
        pos = Pose()
        pos.position.x = i * 10
        pos.position.z = 1.0
        pos.orientation = Quaternion(*quaternion_from_euler(0,0,1))
        spawn(pos, 1,[i+1,1])
        
    rospy.Timer(rospy.Duration(timestep), do_updates)
    while not rospy.is_shutdown():
        rospy.spin
    
    
    

