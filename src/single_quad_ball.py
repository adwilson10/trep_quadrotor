#!/usr/bin/env python

# Quadrotor Simulator v1
import sys
import math
import roslib; roslib.load_manifest('quadsim')
import rospy
import tf

import trep
import trep.potentials
import trep.visual as visual

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np

# initilize joystick global vars
axis = [0.0,0.0,0.0]
buttons=[0,0,0]

# Main Simulation Routine
def QuadMain():

    quadcm_m = 1.0
    quadmotor_m = 1.0
    ball_m = 2.5

    dt = 0.01  # timestep set to 10ms

    # Create a new trep system - 6 generalized coordinates for quadrotor: x,y,z,roll,pitch,yaw
    system = trep.System()
    trep.potentials.Gravity(system, name="Gravity")

    quadz = trep.Frame(system.world_frame, trep.TZ, "quadz")
    quady = trep.Frame(quadz,trep.TY,"quady")
    quadx = trep.Frame(quady,trep.TX,"quadx")

    quadrx = trep.Frame(quadx,trep.RX,"quadrx",kinematic=True)
    quadry = trep.Frame(quadrx,trep.RY,"quadry",kinematic=True)
   # quadrz = trep.Frame(quadry,trep.RZ,"quadrz")
    quadx.set_mass(quadcm_m) # central point mass

    #    # define motor positions and mass
    #    quad1 = trep.Frame(quadrz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(3,0,0)),"quad1")
    #    quad1.set_mass(quadmotor_m)
    #    quad2 = trep.Frame(quadrz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(-3,0,0)),"quad2")
    #    quad2.set_mass(quadmotor_m)
    #    quad3 = trep.Frame(quadrz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,3,0)),"quad3")
    #    quad3.set_mass(quadmotor_m)
    #    quad4 = trep.Frame(quadrz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,-3,0)),"quad4")
    #    quad4.set_mass(quadmotor_m)

    # define ball frame
    ballrx = trep.Frame(quadx,trep.RX,"ballrx",kinematic=False)
    ballry = trep.Frame(ballrx,trep.RY,"ballry",kinematic=False)
    ball = trep.Frame(ballry,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,0,-1)),"ballcm")
    ball.set_mass(ball_m)

    # set thrust vector with input u1
    trep.forces.BodyWrench(system,quadry,(0,0,'u1',0,0,0),name='wrench1')

    #    # set four thrust vectors with inputs u1,u2,u3,u4
    #    trep.forces.BodyWrench(system,quad1,(0,0,'u1',0,0,0),name='wrench1')
    #    trep.forces.BodyWrench(system,quad2,(0,0,'u2',0,0,0),name='wrench2')
    #    trep.forces.BodyWrench(system,quad3,(0,0,'u3',0,0,0),name='wrench3')
    #    trep.forces.BodyWrench(system,quad4,(0,0,'u4',0,0,0),name='wrench4')

    # set quadrotor initial altitude at 3m
    system.get_config("quadz").q = 3.0

    # Now we'll extract the current configuration into a tuple to use as
    # initial conditions for a variational integrator.
    q0 = system.q

    # Create and initialize the variational integrator
    mvi = trep.MidpointVI(system)
    mvi.initialize_from_configs(0.0, q0, dt, q0)

    # These are our simulation variables.
    q = mvi.q2
    t = mvi.t2
#    u0=tuple([(4*quadmotor_m+quadcm_m+ball_m)/4*9.8,(4*quadmotor_m+quadcm_m+ball_m)/4*9.8,(4*quadmotor_m+quadcm_m+ball_m)/4*9.8,(4*quadmotor_m+quadcm_m+ball_m)/4*9.8])
#    u=[u0]

    u0=np.array([(quadcm_m+ball_m)*9.8])
    u=tuple(u0)
    
    # start ROS node to broadcast transforms to rviz
    rospy.init_node('quad_simulator')
    broadcaster = tf.TransformBroadcaster()
    pub = rospy.Publisher('config',Float32MultiArray)
    statepub = rospy.Publisher('joint_states',JointState)

    jointstates = JointState()
    configs = Float32MultiArray()

    # subscribe to joystick topic from joy_node
    rospy.Subscriber("joy",Joy,joycall)

    r = rospy.Rate(100) # simulation rate set to 100Hz

    # PD control variables
    P=200
    D=20

    # Simulator Loop
    while not rospy.is_shutdown():

        # reset simulator if trigger pressed
        if buttons[0]==1:
            mvi.initialize_from_configs(0.0, q0, dt, q0)

#        # calculate thrust inputs
#        u1 = u0[0] + P*(q[4]+0.1*axis[0]) + D*system.configs[4].dq - P*(q[0]-3.0) - D*system.configs[0].dq
#        u2 = u0[1] - P*(q[4]+0.1*axis[0]) - D*system.configs[4].dq - P*(q[0]-3.0) - D*system.configs[0].dq
#        u3 = u0[2] - P*(q[3]+0.1*axis[1]) - D*system.configs[3].dq - P*(q[0]-3.0) - D*system.configs[0].dq
#        u4 = u0[3] + P*(q[3]+0.1*axis[1]) + D*system.configs[3].dq - P*(q[0]-3.0) - D*system.configs[0].dq
#        u = tuple([u1,u2,u3,u4])

        k = np.array([0.1*axis[1],0.1*axis[0]])

        # advance simluation one timestep using trep VI
        mvi.step(mvi.t2+dt,u,k)
        q = mvi.q2
        t = mvi.t2
        configs.data = tuple(q)+u
        
        jointstates.header.stamp = rospy.Time.now()
        jointstates.name = ["q1ballrx","q1ballry"]
        jointstates.position = [q[3],q[4]]
        jointstates.velocity = [system.configs[5].dq,system.configs[6].dq]

        # send transform data to rviz
        broadcaster.sendTransform((q[2],q[1],q[0]),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),"quad1","world")
        broadcaster.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(q[5], q[6], 0),rospy.Time.now(),"quadr","quad1")
        statepub.publish(jointstates)       
        pub.publish(configs)
        r.sleep()

# Joystick callback - retrieve current joystick and button data
def joycall(joydata):
    global axis
    global buttons

    axis = joydata.axes
    buttons = joydata.buttons


# Startup script
if __name__ == '__main__':
    try:
        QuadMain()
    except rospy.ROSInterruptException: pass

