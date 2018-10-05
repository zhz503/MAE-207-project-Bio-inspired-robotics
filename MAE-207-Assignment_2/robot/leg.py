import odrive.core
import time
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib

# For symbolic processing
import sympy
from sympy import symbols
from sympy import sin, cos, asin, acos, pi, atan,sqrt
from sympy.utilities.lambdify import lambdify
from sympy import Matrix
from sympy.solvers import solve
from scipy import linalg



l1 = 7.3  # NEED TO UPDATE units of cm
l2 = 14.3  # NEED TO UPDATE units of cm
l_base = 8  # NEED TO UPDATE units of cm
# motor controller parameters
encoder2angle = 2048 * 4
radTOencoder = encoder2angle/(2*pi)

theta0_sym, theta1_sym, alpha0_sym, alpha1_sym, = symbols(
            'theta0_sym theta1_sym alpha0_sym alpha1_sym' , real=True)

class Leg:
    """
    This is our first class in class :)

    We will define a leg class to interface with the leg and standardize 
    the kind of operations we want to perform

    """
    #global l1, l2, l_base, theta0_sym, theta1_sym, alpha0_sym, alpha1_sym, encoder2angle
    #### Variables outside the init function are constants of the class
    # leg geometry
    #leg_position0
    #leg_position1

    ### Methods
    # Classes are initiated with a constructor that can take in initial parameters. At
    # a minimum it takes in a copy of itself (python... weird). The constructor
    # is one place we can define and initialize class variables

    def __init__(self, simulate = True):#False): #True
        """
        This is the constructor for the leg class. Whenever you make a new leg
        this code will be called. We can optionally make a leg that will simulate
        the computations without needing to be connected to the ODrive
        """

        self.simulate = simulate #simulate

            

        # make the option to code without having the odrive connected
        if self.simulate == False:
            self.drv = self.connect_to_controller()
            self.m0 = self.drv.motor0  # easier handles to the motor commands
            self.m1 = self.drv.motor1

            # current positions
            self.joint_0_home = pi/2
            self.joint_1_home = pi/2
            m0_pos, m1_pos = self.get_joint_pos()
            self.joint_0_pos = m0_pos
            self.joint_1_pos = m1_pos

        else:
            self.drv = None
            self.joint_0_pos = pi/2
            self.joint_1_pos = pi/2
            print('Remember: It''‘s a simulation')


        # home angles
        #self.joint_0_home = 0
        #self.joint_1_home = 0
        # current positions
        m0_pos, m1_pos = self.get_joint_pos()
        self.joint_0_pos = m0_pos 
        self.joint_1_pos = m1_pos
        

        # We will compute the jacobian and inverse just once in the class initialization.
        # This will be done symbolically so that we can use the inverse without having
        # to recompute it every time
        print('here2')
        self.J = self.compute_jacobian()


    def connect_to_controller(self):
        """
        Connects to the motor controller
        """
        drv = odrive.core.find_any(consider_usb=True, consider_serial=False)

        if drv is None:
            print('No controller found')
        else:
            print('Connected!')
        return drv

    ###
    ### Motion functions
    ###
    def get_joint_pos(self):
        """
        Get the current joint positions and store them in self.joint_0_pos and self.joint_1_pos in degrees.
        Also, return these positions using the return statement to terminate the function
        """
        # if simulating exit function
        if self.simulate == True:
            return (self.joint_0_pos, self.joint_1_pos)
        else: 
            self.joint_0_pos = self.m0.encoder.pll_pos/encoder2angle*(2*pi) + self.joint_0_home
            self.joint_1_pos = self.m1.encoder.pll_pos/encoder2angle*(2*pi) + self.joint_1_home

        return (self.joint_0_pos, self.joint_1_pos)

    def set_home(self):
        """
        This function updates the home locations of the motors so that 
        all move commands we execute are relative to this location. 
        """
        # if simulating exit function
        if self.simulate == True:
            return
        else:
            self.joint_0_home = self.joint_0_pos  ;
            self.joint_1_home = self.joint_1_pos  ;

    def set_joint_pos(self, theta0, theta1, vel0=0, vel1=0, curr0=0, curr1=0):
        """
        Set the joint positions in units of deg, and with respect to the joint homes.
        We have the option of passing the velocity and current feedforward terms.
        """
        # if simulating exit function
        if self.simulate == True:
            self.joint_0_pos = theta0
            self.joint_1_pos = theta1
        else: 
            #self.get_joint_pos()
            #self.m0.pos_setpoint = (theta0 * radTOencoder)- self.joint_0_pos * radTOencoder;
            #self.m1.pos_setpoint = (theta1 * radTOencoder)- self.joint_0_pos * radTOencoder;
            self.m0.pos_setpoint = (theta0 * radTOencoder)- self.joint_0_home * radTOencoder;
            self.m1.pos_setpoint = (theta1 * radTOencoder)- self.joint_1_home * radTOencoder;
            
            

    def move_home(self):
        """
        Move the motors to the home position
        """
        # if simulating exit function
        if self.simulate == True:
            return
        else: 
            self.m0.pos_setpoint = self.joint_0_home;
            self.m1.pos_setpoint = self.joint_1_home;
        

    def set_foot_pos(self, x, y):
        """
        Move the foot to position x, y. This function will call the inverse kinematics 
        solver and then call set_joint_pos with the appropriate angles
        """
        # if simulating exit function
        if self.simulate == True:
            theta_0, theta_1 = self.inverse_kinematics(x,y)
            self.set_joint_pos(theta_0,theta_1) 
        else:
            theta_0, theta_1 = self.inverse_kinematics(x,y)
            self.set_joint_pos(theta_0,theta_1) # sth may be wrong here
        #
        # Your code here
        #

    def move_trajectory(self, tt, xx, yy):
        """
        Move the foot over a cyclic trajectory to positions xx, yy in time tt. 
        This will repeatedly call the set_foot_pos function to the new foot 
        location specified by the trajectory.
        """
        # if simulating exit function
        if self.simulate == True:
            
            #fig =   plt.figure()
            #ax = plt.subplot(1,1,1)
            empty_theta0 = []
            empty_theta1 = []
            
            for i in range(tt):
                self.set_foot_pos(xx[i],yy[i])
                theta_0, theta_1 = self.get_joint_pos()
                self.set_joint_pos(theta_0,theta_1)
                #print(theta_0,theta_1)
                #plt.plot(xx[i], yy[i], linewidth = '4',color='red', linestyle='-')
                #plt.draw()
                #self.draw_leg(ax)  
                #plt.title("Group1: Dawoon and Zhaoliang") 
                #plt.draw()
                #ax.axis([-20, 20, -10, 25])
                #ax.invert_yaxis()
                empty_theta0 = empty_theta0 + [theta_0]
                empty_theta1 = empty_theta1 + [theta_1]
                plt.pause(0.3)
                plt.show()
                if i < 10:
                    plt.savefig("00"+str(i)+".png")
                else:
                    plt.savefig("0"+str(i)+".png")
            return (empty_theta0, empty_theta1)   
        else:
            for i in range(tt):
                self.set_foot_pos(xx[i],yy[i])
                #print(theta_0,theta_1)
                          
        #
        # Your code here
        #
        

    ###
    ### Leg geometry functions
    ###
    def compute_internal_angles(self, theta_0, theta_1):
        """
        Return the internal angles of the robot leg 
        from the current motor angles
        """
        alpha_0, alpha_1, A, B, C = symbols('alpha_0 alpha_1 A B C', real=True)

        d = sqrt(l_base*l_base + l1*l1- 2*l_base*l1*cos(theta_0))
        beta = -asin(l1/d*sin(theta_0))
        
        A = sympy.simplify(2*l1*l2*cos(theta_1)+2*d*l2*cos(beta))
        B = sympy.simplify(2*l1*l2*sin(theta_1)+2*d*l2*sin(beta))
        C = sympy.simplify(-l1*l1-d*d-2*d*l1*cos(theta_1-beta))
        
        alpha_1 = sympy.simplify(atan(B/A) + acos(C/sqrt(A*A+B*B)))
        alpha_0 = beta + acos((l1*cos(theta_1-beta) + l2*cos(alpha_1-beta)+d)/l2) 
        
        return (alpha_0, alpha_1)

    def compute_jacobian(self):
        """
        This function implements the symbolic solution to the Jacobian.
        """
        # initiate the symbolic variables
        (alpha0_sym,alpha1_sym) = self.compute_internal_angles(theta0_sym, theta1_sym)
        
        x = l_base/2 + l1*cos(theta0_sym) + l2*cos(alpha0_sym)
        y = l1*sin(theta1_sym) + l2*sin(alpha1_sym)
        
        J = Matrix([[sympy.diff(x,theta0_sym), sympy.diff(x,theta1_sym)],[sympy.diff(y,theta0_sym), sympy.diff(y,theta1_sym)]])
        return J

    def inverse_kinematics(self, x, y):
        """
        This function will compute the required theta_0 and theta_1 angles to position the 
        foot to the point x, y. We will use an iterative solver to determine the angles.
        """
        beta = 9e-1
        epsilon = 8e-2
        xy_error = Matrix([1e2, 1e2])
        theta_0,theta_1 = self.get_joint_pos()
        #theta_current = Matrix([[theta_0],[theta_1]]) 


        #count = 0
        while xy_error.norm() > epsilon: 
            alpha_0,alpha_1 = self.compute_internal_angles(theta_0,theta_1)
            x_current = l_base/2 + l1*cos(theta_1) + l2*cos(alpha_1)
            y_current = l1*sin(theta_1) + l2*sin(alpha_1)
            x_error = x-x_current
            y_error = y-y_current
            xy_error = Matrix([x_error, y_error])
            J_current = self.J.subs({theta0_sym:theta_0, #theta_current[0],#
                            theta1_sym: theta_1,#theta_current[1],
                            alpha0_sym: alpha_0,
                            alpha1_sym: alpha_1})
            J_current = sympy.N(J_current)
            J_current_inv = J_current.pinv()
            #theta_current = beta * J_current_inv@xy_error + theta_current
            d_theta = beta * J_current_inv@xy_error 
            
            theta_0 = theta_0 + d_theta[0] 
            theta_1 = theta_1 + d_theta[1]
            #count = count + 1
            
        #print(count)
        return (theta_0, theta_1) 
        #return (theta_current[0], theta_current[1])

    ###
    ### Visualization functions
    ###
    def draw_leg(self,ax=False):
        """
        This function takes in the four angles of the leg and draws
        the configuration
        """
        

        theta1, theta2 = self.joint_0_pos, self.joint_1_pos
        link1, link2, width = l1, l2, l_base

        alpha1, alpha2 = self.compute_internal_angles(theta1,theta2)

        def pol2cart(rho, phi):
            x = rho * np.cos(phi)
            y = rho * np.sin(phi)
            return (x, y)

        if ax == False:
            
            ax = plt.gca()
            ax.cla()


        ax.plot(-width / 2, 0, 'ok')
        ax.plot(width / 2, 0, 'ok')

        ax.plot([-width / 2, 0], [0, 0], 'k')
        ax.plot([width / 2, 0], [0, 0], 'k')

        ax.plot(-width / 2 + np.array([0, link1 * cos(theta1)]), [0, link1 * sin(theta1)], 'k')
        ax.plot(width / 2 + np.array([0, link1 * cos(theta2)]), [0, link1 * sin(theta2)], 'k')

        ax.plot(-width / 2 + link1 * cos(theta1) + np.array([0, link2 * cos(alpha1)]), \
                link1 * sin(theta1) + np.array([0, link2 * sin(alpha1)]), 'k');
        ax.plot(width / 2 + link1 * cos(theta2) + np.array([0, link2 * cos(alpha2)]), \
                np.array(link1 * sin(theta2) + np.array([0, link2 * sin(alpha2)])), 'k');

        ax.plot(width / 2 + link1 * cos(theta2) + link2 * cos(alpha2), \
                np.array(link1 * sin(theta2) + link2 * sin(alpha2)), 'ro');

        #ax.axis([-2, 2, 18, 21])
        ax.axis([-20, 20, -10, 25])
        ax.invert_yaxis()

        plt.draw()
        