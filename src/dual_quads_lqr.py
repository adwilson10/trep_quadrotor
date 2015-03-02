#!/usr/bin/env python

# Quadrotor Simulator v1
import sys
import math
import roslib; roslib.load_manifest('quadsim')
import rospy
import tf

from math import pi as mpi
import numpy as np

import trep
import trep.potentials
import trep.visual as visual
import trep.discopt as discopt

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarker
from interactive_markers.interactive_marker_server import *

# initilize joystick global vars
axis = [0.0,0.0,0.0]
buttons=[0,0,0]

# Main Simulation Routine
def QuadMain(isjoy):

    def make_state_cost(dsys, base, x):
        weight = base*np.ones((dsys.nX,))
        weight[system.get_config('ballx').index] = 50
        weight[system.get_config('bally').index] = 50
        weight[system.get_config('ballz').index] = 25
        weight[system.get_config('quad1bry').index] = 500
        weight[system.get_config('quad1brx').index] = 100
        weight[system.get_config('quad1ry').index] = 10
        weight[system.get_config('quad1rx').index] = 10
        weight[system.get_config('quad1rz').index] = 1
        weight[system.get_config('quad2bry').index] = 500
        weight[system.get_config('quad2brx').index] = 15
        weight[system.get_config('quad2ry').index] = 10
        weight[system.get_config('quad2rx').index] = 10
        weight[system.get_config('quad2rz').index] = 1
        return np.diag(weight)

    def make_input_cost(dsys, base, x):
        weight = base*np.ones((dsys.nU,))
       # weight[system.get_input('x-force').index] = x
        return np.diag(weight)

    quad1cm_m = 1.0
    quadmotor1_m = 1.0
    quad2cm_m = 1.0
    quadmotor2_m = 1.0
    ball_m = 0.5

    dt = 0.01  # timestep set to 10ms

    # Create a new trep system - 6 generalized coordinates for quadrotor: x,y,z,roll,pitch,yaw
    system = trep.System()
    trep.potentials.Gravity(system, name="Gravity")

    # define ball frame
    ballz = trep.Frame(system.world_frame,trep.TZ,"ballz")
    bally = trep.Frame(ballz,trep.TY,"bally")
    ballx = trep.Frame(bally,trep.TX,"ballx")
    ballx.set_mass(ball_m)

    # define quadrotor 1 frame
    quad1bry = trep.Frame(ballx,trep.RY,"quad1bry")
    quad1brx = trep.Frame(quad1bry,trep.RX,"quad1brx")
    quad1cm = trep.Frame(quad1brx,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,0,2)),"quad1cm")
    quad1ry = trep.Frame(quad1cm,trep.RY,"quad1ry")
    quad1rx = trep.Frame(quad1ry,trep.RX,"quad1rx")
    quad1rz = trep.Frame(quad1rx,trep.RZ,"quad1rz")

    quad1cm.set_mass(quad1cm_m) # central point mass

    # define quadrotor 1 motor positions and mass
    quad1m1 = trep.Frame(quad1rz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(1,0,0)),"quad1m1")
    quad1m1.set_mass(quadmotor1_m)
    quad1m2 = trep.Frame(quad1rz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(-1,0,0)),"quad1m2")
    quad1m2.set_mass(quadmotor1_m)
    quad1m3 = trep.Frame(quad1rz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,1,0)),"quad1m3")
    quad1m3.set_mass(quadmotor1_m)
    quad1m4 = trep.Frame(quad1rz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,-1,0)),"quad1m4")
    quad1m4.set_mass(quadmotor1_m)

    # set four thrust vectors with inputs u1,u2,u3,u4
    trep.forces.BodyWrench(system,quad1m1,(0,0,'u1q1',0,0,0),name='quad1w1')
    trep.forces.BodyWrench(system,quad1m2,(0,0,'u2q1',0,0,0),name='quad1w2')
    trep.forces.BodyWrench(system,quad1m3,(0,0,'u3q1',0,0,0),name='quad1w3')
    trep.forces.BodyWrench(system,quad1m4,(0,0,'u4q1',0,0,0),name='quad1w4')


    # define quadrotor 2 frame
    quad2bry = trep.Frame(ballx,trep.RY,"quad2bry") 
    quad2brx = trep.Frame(quad2bry,trep.RX,"quad2brx")
    quad2cm = trep.Frame(quad2brx,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,0,2)),"quad2cm")
    quad2ry = trep.Frame(quad2cm,trep.RY,"quad2ry")
    quad2rx = trep.Frame(quad2ry,trep.RX,"quad2rx")
    quad2rz = trep.Frame(quad2rx,trep.RZ,"quad2rz")

    quad2cm.set_mass(quad2cm_m) # central point mass

    # define quadrotor 2 motor positions and mass
    quad2m1 = trep.Frame(quad2rz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(1,0,0)),"quad2m1")
    quad2m1.set_mass(quadmotor2_m)
    quad2m2 = trep.Frame(quad2rz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(-1,0,0)),"quad2m2")
    quad2m2.set_mass(quadmotor2_m)
    quad2m3 = trep.Frame(quad2rz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,1,0)),"quad2m3")
    quad2m3.set_mass(quadmotor2_m)
    quad2m4 = trep.Frame(quad2rz,trep.CONST_SE3,((1,0,0),(0,1,0),(0,0,1),(0,-1,0)),"quad2m4")
    quad2m4.set_mass(quadmotor2_m)

    # set four thrust vectors with inputs u1,u2,u3,u4
    trep.forces.BodyWrench(system,quad2m1,(0,0,'u1q2',0,0,0),name='quad2w1')
    trep.forces.BodyWrench(system,quad2m2,(0,0,'u2q2',0,0,0),name='quad2w2')
    trep.forces.BodyWrench(system,quad2m3,(0,0,'u3q2',0,0,0),name='quad2w3')
    trep.forces.BodyWrench(system,quad2m4,(0,0,'u4q2',0,0,0),name='quad2w4')

    # set quadrotor initial altitude at 5m
    system.get_config("ballz").q = 0.0

    system.get_config("quad1bry").q = (math.pi/2-math.acos(1.5/2.0))
    system.get_config("quad2bry").q = -(math.pi/2-math.acos(1.5/2.0))

    horzf = 0.5*ball_m*9.8*math.tan((math.pi/2-math.acos(1.5/2.0)))
    vertf = (0.5*ball_m+quad1cm_m+4*quadmotor1_m)*9.8   
    quad1ang = math.atan(horzf/((quad1cm_m+4*quadmotor1_m)*9.8))

    system.get_config("quad1ry").q = -(math.pi/2-math.acos(1.5/2.0))+quad1ang
    system.get_config("quad2ry").q = (math.pi/2-math.acos(1.5/2.0))-quad1ang

    # Now we'll extract the current configuration into a tuple to use as
    # initial conditions for a variational integrator.
    q0 = system.q   

    # Create the discrete system
    mvi = trep.MidpointVI(system)
    t = np.arange(0.0, 10.0, 0.01)
    dsys = discopt.DSystem(mvi, t)

    # Generate cost function
    Qcost = make_state_cost(dsys, 1, 0.00)
    Rcost = make_input_cost(dsys, 0.01, 0.01)

    totuf = math.sqrt(math.pow(horzf,2)+math.pow(vertf,2))    
    u0=np.array([totuf/4,totuf/4,totuf/4,totuf/4,totuf/4,totuf/4,totuf/4,totuf/4])
    u=tuple(u0)

    mvi.initialize_from_configs(0.0, q0, dt, q0)
    pref = mvi.p2

    Uint = np.zeros((len(t)-1,system.nu))
    qd = np.zeros((len(t),system.nQ))
    pd = np.zeros((len(t),system.nQ))
    for i,t in enumerate(t[:-1]):
        Uint[i,:] = u0
        qd[i,:] = q0
        pd[i,:] = pref
    qd[len(qd)-1,:]=q0
    pd[len(pd)-1,:]=pref
        
    Qk = lambda k: Qcost
    (X,U) = dsys.build_trajectory(Q=qd,p=pd,u=Uint)
    Kstab = dsys.calc_feedback_controller(X,U,Qk)
   
    #print Kstab[0]

    # Create and initialize the variational integrator)
    u=tuple(u0) 
    #mvi.initialize_from_configs(0.0, q0, dt, q0)

    # These are our simulation variables.
    q = mvi.q2
    p = mvi.p2
    pref = p
    t = mvi.t2
 
    # start ROS node to broadcast transforms to rviz
    rospy.init_node('quad_simulator')
    broadcaster = tf.TransformBroadcaster()
    pub = rospy.Publisher('config',Float32MultiArray, queue_size=2)
    statepub = rospy.Publisher('joint_states',JointState, queue_size=2)

    jointstates = JointState()
    configs = Float32MultiArray()
    if isjoy==True:
        markerpub = rospy.Publisher('refmark',Marker, queue_size=2)
        refmark = Marker()

    if isjoy==False:
############### NEW ################################33
        # create an interactive marker server on the topic namespace simple_marker
        server = InteractiveMarkerServer("simple_marker")
        
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/world"
        int_marker.name = "my_marker"
        int_marker.description = "Simple 1-DOF Control"
        int_marker.pose.position.z = 2.0

        # create a grey box marker
        box_marker = Marker()
        box_marker.type = Marker.SPHERE
        box_marker.scale.x = 0.15
        box_marker.scale.y = 0.15
        box_marker.scale.z = 0.15
        box_marker.color.r = 0.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.orientation.w = 1
        box_control.orientation.x = 0
        box_control.orientation.y = 1
        box_control.orientation.z = 0
        box_control.name = "rotate_x"
        box_control.name = "move_x"
        box_control.always_visible = True
        box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        int_marker.controls.append(box_control)

        box_control = InteractiveMarkerControl()
        box_control.orientation.w = 1
        box_control.orientation.x = 0
        box_control.orientation.y = 1
        box_control.orientation.z = 0
        box_control.name = "rotate_x"
        box_control.name = "move_x"
        box_control.always_visible = True
        box_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        box_control.markers.append( box_marker )
        int_marker.controls.append(box_control)

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        server.insert(int_marker, processFeedback)

        # 'commit' changes and send to all clients
        server.applyChanges()
############### END NEW ###############################
    if isjoy==True:
        # subscribe to joystick topic from joy_node
        rospy.Subscriber("joy",Joy,joycall)

    r = rospy.Rate(100) # simulation rate set to 100Hz

    # Simulator Loop
    while not rospy.is_shutdown():

        # reset simulator if trigger pressed
        if buttons[0]==1:
            mvi.initialize_from_configs(0.0, q0, dt, q0)

        qref = [0,axis[1],axis[0],0,0,0,0,0,0,0,0,0,0]
        u = tuple(u0-np.dot(Kstab[0],np.append(q-q0-qref,p-pref)))

        # advance simluation one timestep using trep VI

        try:
            mvi.step(mvi.t2+dt,u)
        except trep.ConvergenceError:
            print 'Trep simulation error - system resetting...'
            rospy.sleep(2.)
            mvi.initialize_from_configs(0.0, q0, dt, q0)

        q = mvi.q2
        p = mvi.p2
        t = mvi.t2
        configs.data = tuple(q)+u

        if isjoy==True:
            refmark.header.frame_id = "/world"
            refmark.header.stamp = rospy.Time.now()
            refmark.type = 2
            refmark.scale.x = 0.15;
            refmark.scale.y = 0.15;
            refmark.scale.z = 0.15;
            refmark.color.r = 0.0;
            refmark.color.g = 1.0;
            refmark.color.b = 0.0;
            refmark.color.a = 1.0;
            refmark.pose.position.y = axis[1]
            refmark.pose.position.x = axis[0]
            refmark.pose.position.z = 2.0
        
        jointstates.header.stamp = rospy.Time.now()
        jointstates.name = ["quad1bry","quad1brx","quad1ry","quad1rx","quad1rz","quad2bry","quad2brx","quad2ry","quad2rx","quad2rz"]
        jointstates.position = [q[3],q[4],q[5],q[6],q[7],q[8],q[9],q[10],q[11],q[12]]
        jointstates.velocity = [system.configs[3].dq,system.configs[4].dq,system.configs[5].dq,system.configs[6].dq,system.configs[7].dq,system.configs[8].dq,system.configs[9].dq,system.configs[10].dq,system.configs[11].dq,system.configs[12].dq]

        # send transform data to rviz
        broadcaster.sendTransform((q[2],q[1],q[0]+2),tf.transformations.quaternion_from_euler(0.0,0.0,0.0),rospy.Time.now(),"ball","world")
        statepub.publish(jointstates)      
        pub.publish(configs)
        if isjoy==True:
            markerpub.publish(refmark)
        r.sleep()

# Joystick callback - retrieve current joystick and button data
def joycall(joydata):
    global axis
    global buttons

    axis = joydata.axes
    axis = [-5*axis[0],5*axis[1],axis[2]]
    buttons = joydata.buttons

# Interactive marker feedback
def processFeedback(feedback):
    global axis
    axis = [feedback.pose.position.x,feedback.pose.position.y,feedback.pose.position.z]


# Startup script
if __name__ == '__main__':
    try:
        if(len(sys.argv) > 0 and sys.argv[1]=="false"):
            QuadMain(False)
        else:
            QuadMain(True)
    except rospy.ROSInterruptException: pass

