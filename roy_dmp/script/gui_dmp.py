#!/usr/bin/python


import sys
import os
import time

from PyQt5 import QtGui
import rospy
from sensor_msgs.msg import JointState
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QWidget,QHBoxLayout, QMainWindow, QPushButton, QMessageBox, QBoxLayout,QVBoxLayout
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from rviz import bindings as rviz
sys.path.append(os.getcwd()+"/src/roy_dmp/src")
from ui_dmp import Ui_MainWindow
from dmp_record import *
from dmp_learn import *
from dmp_execute import *
import numpy as np
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)
from std_msgs.msg import Bool

def get_joints():
    js = rospy.wait_for_message("joint_states",JointState)
    return js

# def check_service():
#     check_srv = rospy.wait_for_service('learn_dmp_from_demo')
#     return check_srv

class ApplicationWindow(QtWidgets.QMainWindow):
    def __init__(self):

        super(ApplicationWindow, self).__init__()

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # START WINDOW
        self.ui.simulationCheckBox.stateChanged.connect(self.setSimulation)
        self.ui.rvizCheckBox.stateChanged.connect(self.setRviz)
        self.ui.stackedWidget.setCurrentIndex(0)
        self.ui.startButton.clicked.connect(self.start_click)
        self.ui.start_loadClasses_PB.clicked.connect(self.load_classes)
        self.ui.Tab.currentChanged.connect(self.browse_recordings)
        self.ui.shutDownButton.clicked.connect(self.shutdown_click)
        self.ui.shutDownButton.setStyleSheet("background-color: red")
        self.ui.start_loadClasses_PB.setEnabled(False)

        #RECORDING WINDOW

        # Recording buttons connect
        self.ui.rec_EE_radio_button.clicked.connect(self.setRecEE)
        self.ui.rec_JS_radio_button.clicked.connect(self.setRecJS)
        self.ui.name_recording_line_edit.textChanged[str].connect(self.setNameRecording)
        self.ui.start_recording_button.clicked.connect(self.startRecording)
        self.ui.stop_recording_button.clicked.connect(self.stopRecording)
        #self.ui.ChooseRobotcomboBox.currentIndexChanged.connect(self.chooseRobot)
        self.ui.IPLineEdit.textChanged[str].connect(self.setIPRobot)
        self.ui.stop_recording_button.setEnabled(False)

        # LEARNING WINDOW

        # Learning buttons connect
        self.ui.load_EE_rec_radio_button.clicked.connect(self.loadEERec)
        self.ui.load_JS_rec_radio_button.clicked.connect(self.loadJSRec)
        self.ui.load_JS_rec_radio_button.setChecked(True)
        # self.ui.dmp_dt_param_lineedit.setValidator(QDoubleValidator(0.0,99,5))
        self.ui.dmp_dt_param_lineedit.textChanged[str].connect(self.setdt)
        self.ui.dmp_K_param_lineedit.setValidator(QIntValidator(0,1000))
        self.ui.dmp_K_param_lineedit.textChanged[str].connect(self.setK)
        self.ui.dmp_D_param_lineedit.setValidator(QIntValidator(0,1000))
        self.ui.dmp_D_param_lineedit.textChanged[str].connect(self.setD)
        self.ui.dmp_basisfunc_param_lineedit.setValidator(QIntValidator(0,9999))
        self.ui.dmp_basisfunc_param_lineedit.textChanged[str].connect(self.setBasisfunc)
        # self.ui.Get_recordings_pushButton.clicked.connect(self.browse_folder)
        self.ui.load_name_rec_comboBox.currentIndexChanged.connect(self.loadNamerec)
        self.ui.generate_DMP_button.clicked.connect(self.generateDMP)
        
        self.ui.save_name_dmp_lineedit.textChanged[str].connect(self.saveWeights)


        # EXECUTION WINDOW

        # Executing buttons connect
        # self.ui.get_active_DMP_pushButton.clicked.connect(self.browse_weights)
        self.ui.set_active_DMP_pushButton.clicked.connect(self.sendActiveDMP)
        self.ui.set_DMP_comboBox.currentIndexChanged.connect(self.setActiveDMP)
        self.ui.get_JS_init_robot_pushButton.clicked.connect(self.getJS_init_robot)
        self.ui.get_JS_goal_robot_pushButton.clicked.connect(self.getJS_goal_robot)
        self.ui.get_CS_init_robot_PB.clicked.connect(self.getCS_init_robot)
        self.ui.get_CS_goal_robot_PB.clicked.connect(self.getCS_goal_robot)

        self.ui.execute_collisionCheck_PB.clicked.connect(self.collision_check)
        
        self.ui.execute_collisionCheck_PB.setText("Please set an active DMP")
        self.ui.execute_collisionCheck_PB.setEnabled(False)

        self.ui.execute_plan_pushButton.clicked.connect(self.execute_plan)

        #LINE EDIT TEXT CHANGES
        # Joint state initial position
        self.ui.set_JointState_init_0_lineEdit.textChanged[str].connect(self.setJS_init_0)
        self.ui.set_JointState_init_1_lineEdit.textChanged[str].connect(self.setJS_init_1)
        self.ui.set_JointState_init_2_lineEdit.textChanged[str].connect(self.setJS_init_2)
        self.ui.set_JointState_init_3_lineEdit.textChanged[str].connect(self.setJS_init_3)
        self.ui.set_JointState_init_4_lineEdit.textChanged[str].connect(self.setJS_init_4)
        self.ui.set_JointState_init_5_lineEdit.textChanged[str].connect(self.setJS_init_5)
        # Joint state Goal position
        self.ui.set_JointState_goal_0_lineEdit.textChanged[str].connect(self.setJS_goal_0)
        self.ui.set_JointState_goal_1_lineEdit.textChanged[str].connect(self.setJS_goal_1)
        self.ui.set_JointState_goal_2_lineEdit.textChanged[str].connect(self.setJS_goal_2)
        self.ui.set_JointState_goal_3_lineEdit.textChanged[str].connect(self.setJS_goal_3)
        self.ui.set_JointState_goal_4_lineEdit.textChanged[str].connect(self.setJS_goal_4)
        self.ui.set_JointState_goal_5_lineEdit.textChanged[str].connect(self.setJS_goal_5)
        # Cartesian space initial position
        self.ui.set_CS_init_tx_lineEdit.textChanged[str].connect(self.setCS_init_tx)
        self.ui.set_CS_init_ty_lineEdit.textChanged[str].connect(self.setCS_init_ty)
        self.ui.set_CS_init_tz_lineEdit.textChanged[str].connect(self.setCS_init_tz)
        self.ui.set_CS_init_rx_lineEdit.textChanged[str].connect(self.setCS_init_rx)
        self.ui.set_CS_init_ry_lineEdit.textChanged[str].connect(self.setCS_init_ry)
        self.ui.set_CS_init_rz_lineEdit.textChanged[str].connect(self.setCS_init_rz)
        self.ui.set_CS_init_w_lineEdit.textChanged[str].connect(self.setCS_init_w)
        # Cartesian space goal position
        self.ui.set_CS_goal_tx_lineEdit.textChanged[str].connect(self.setCS_goal_tx)
        self.ui.set_CS_goal_ty_lineEdit.textChanged[str].connect(self.setCS_goal_ty)
        self.ui.set_CS_goal_tz_lineEdit.textChanged[str].connect(self.setCS_goal_tz)
        self.ui.set_CS_goal_rx_lineEdit.textChanged[str].connect(self.setCS_goal_rx)
        self.ui.set_CS_goal_ry_lineEdit.textChanged[str].connect(self.setCS_goal_ry)
        self.ui.set_CS_goal_rz_lineEdit.textChanged[str].connect(self.setCS_goal_rz)
        self.ui.set_CS_goal_w_lineEdit.textChanged[str].connect(self.setCS_goal_w)

        # self.ui.set_tau_lineedit.setValidator(QIntValidator(1,1000))
        self.ui.set_tau_lineedit.textChanged[str].connect(self.setTau)

        self.ui.avoidanceCheckBox.stateChanged.connect(self.setAvoidance)

        self.ui.safeStopCheckBox.stateChanged.connect(self.setSafeStop)

        self.ui.execute_plan_pushButton.setEnabled(False)
        #Robot variables
        self.arm = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
       
       #Initialize DMP classes from ROS
        self.dmp_record_JS =[]
        self.dmp_mg = []
        self.dmp_me = []



        ############### PARAMETERS ###########################################################


        # Front page params
        self.debug = False
        self.robot = "roslaunch roy_dmp ur_with_dmp.launch"
        self.IP = '10.10.73.235' #TODO repair getting IP from GUI
        self.sim = False
        self.rviz = False
        self.robot_model = 'UR3'
        self.process = QProcess(self)
        self.linkName = "rg2_eef_link"
        self.robot_ready = 0 # 0-not ready 1-ready 2-error

        #Param Recording page
        self.dmp_record = 0  # 0 for EE, 1 for JS, 2 for JS with Filtering
        self.dmp_record_name = "No_name"
        
        #Param Learning page
        self.dmp_record_name_load = "No_name"
        self.dmp_weight_name = None
        self.dmp_load_type = 1 # 0 for EE, 1 for JS, 2 for JS with filtering
        self.dmp_param_dt = 0.008
        self.dmp_param_K = 100
        self.dmp_param_D = 2.0 * np.sqrt(self.dmp_param_K)
        self.dmp_param_basisfunc = 50
        self.directory_recording = os.getcwd()+"/src/roy_dmp/data/rosbag_recordings"

        # Param executing page
        self.active_DMP = None
        self.selected_DMP = 'No_name'
        self.initial_JS = [-0.2273033300982874, -2.298889462147848, -1.0177272001849573, -1.3976243177997034,  1.5502419471740723, 9.261386219655172]
        self.goal_JS = [-2.3324595133410853, -2.2434170881854456, -1.1172669569598597, -1.3543337027179163, 1.5941375494003296, 7.169057373200552]
        self.init_CS = PoseStamped()
        self.goal_CS = PoseStamped()
        self.path_plan = []
        self.directory_weights = os.getcwd()+"/src/roy_dmp/data/weights"
        self.tau = 1*5
        self.avoidance_enabled = False
        self.safe_stop_enabled = False
        

    def browse_recordings(self):
        self.ui.load_name_rec_comboBox.clear()
        rec_dir = os.listdir(self.directory_recording)
        rec_dir = sorted(rec_dir)
        for file in rec_dir:
            if file.endswith(".bag"):
                self.ui.load_name_rec_comboBox.addItem(str(file))
        self.browse_weights()

    def setSimulation(self,state):
        if state == Qt.Checked:
            self.sim = True
        else:
            self.sim = False
    
    def setRviz(self,state):
        if state == Qt.Checked:
            self.rviz = True
        else:
            self.rviz = False
    
    def setAvoidance(self,state):
        if state == Qt.Checked:
            self.avoidance_enabled = True
        else:
            self.avoidance_enabled = False
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.ui.execute_plan_pushButton.setEnabled(False)  
        '''
        if state == Qt.Checked:
            self.avoidance_enabled = True
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.ui.execute_collisionCheck_PB.setEnabled(False)
            self.ui.safeStopCheckBox.setEnabled(False)
            ss_en = self.safe_stop_enabled
            self.ui.safeStopCheckBox.setChecked(False)
            self.safe_stop_enabled = ss_en
            if self.active_DMP != None:
                self.ui.execute_plan_pushButton.setEnabled(True)
        else:
            self.avoidance_enabled = False
            self.ui.safeStopCheckBox.setEnabled(True)
            if self.safe_stop_enabled:
                self.ui.safeStopCheckBox.setChecked(True)
            self.ui.execute_plan_pushButton.setEnabled(False)  
            if self.active_DMP != None:
                self.ui.execute_collisionCheck_PB.setEnabled(True)
        '''
    
    def setSafeStop(self,state):
        if state == Qt.Checked:
            self.safe_stop_enabled = True
        else:
            self.safe_stop_enabled = False

    def setIPRobot(self,text):
        self.IP = text

    def chooseRobot(self):
        self.robot_model = self.ui.ChooseRobotcomboBox.currentText()
        
    def on_ready_read(self):
        output_data = self.process.readAll().data().decode('utf-8')
        if self.debug:
            print(output_data)
        #if 'Action client not connected:' in output_data:
        if 'Failed to connect to robot on IP' in output_data:   
            if self.debug:
                print('Execution failed, robot not connected')
                
            self.process.close()
            self.robot_ready = 2
        elif 'You can start planning now!' in output_data:
            self.robot_ready = 1
            if self.debug:
                print('Robot connected')
        
    
    def start_click(self):
        self.robot_ready = 0
        self.chooseRobot()
        program = self.robot+" robot_model:="+self.robot_model.lower()+" sim:="+str(self.sim).lower()+" robot_ip:="+self.IP
        #self.process.setReadChannel(QProcess.StandardOutput) # QProcess.StandardError
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        #self.process.setArguments(args) 
        self.process.readyRead.connect(self.on_ready_read)
        self.process.start(program)
        self.ui.startButton.setEnabled(False)
        self.ui.startButton.setText("Connecting...")
        self.ui.startButton.setStyleSheet("color: black")
        QApplication.processEvents()
        while self.robot_ready == 0:
            self.process.waitForReadyRead(1)
        if self.robot_ready == 1:
            self.ui.start_loadClasses_PB.setEnabled(True)
            self.ui.startButton.setText("Connected to Robot")
            self.ui.startButton.setStyleSheet("color: green")
        elif self.robot_ready == 2:
            self.ui.startButton.setText("Connection error (Click to reconnect)")
            self.ui.startButton.setStyleSheet("color: red")
            self.process.terminate()
            self.process.waitForFinished()
            self.process.kill()
            self.process.start("rosnode kill -a")
            self.ui.startButton.setEnabled(True)
        if self.rviz:
            self.RviZ = RViz() #TODO optimize to avoid crash
            self.ui.gridLayout_6.addWidget(self.RviZ)

    def shutdown_click(self):
        self.ui.shutDownButton.setStyleSheet("background-color: gray")
        self.ui.shutDownButton.setText("Shutting down the program. Please wait")
        QApplication.processEvents()
        self.process.terminate()
        self.process.waitForFinished()
        self.process.kill()
        self.process.start("rosnode kill -a")
        sys.exit(0)

    def setRecEE(self):
        self.dmp_record = 0
    def setRecJS(self):
        self.dmp_record = 1
        

    def setNameRecording(self, text):
        self.dmp_record_name = text
        # print(self.dmp_record_name)

    def startRecording(self):
        self.ui.start_recording_button.setEnabled(False)
        self.ui.start_recording_button.setText("Recording")
        self.ui.start_recording_button.setStyleSheet("background-color: gray")
        self.ui.stop_recording_button.setEnabled(True)
        if self.dmp_record ==1:
            self.dmp_record_JS.start_record(self.dmp_record_name,self.arm,self.dmp_record_name)
        elif self.dmp_record == 0:
            pass


        
    def stopRecording(self):
        self.dmp_record_JS.stop_record()
        self.ui.start_recording_button.setEnabled(True)
        self.ui.stop_recording_button.setText("Recording saved as: " + self.dmp_record_name)
        self.ui.stop_recording_button.setEnabled(False)
        self.ui.stop_recording_button.setStyleSheet("color: green")
        self.ui.start_recording_button.setStyleSheet("background-color: white")
        self.ui.start_recording_button.setText("Start Recording")
    def loadEERec(self):
        self.dmp_load_type = 0
        self.change_DMP_button()
    def loadJSRec(self):
        # self.dmp_mg.loadMotionFromJointStates(self.dmp_record_name_load,self.arm)
        self.dmp_load_type = 1
        self.change_DMP_button()

    def setdt(self,text):
        try:
            self.dmp_param_dt = float(text)
        except:
            print("Wrong Type")
        self.change_DMP_button()
    def setK(self,text):
        try:
            self.dmp_param_K = float(text)
        except:
            print("Wrong Type")
        self.change_DMP_button()
    def setD(self,text):
        try:
            self.dmp_param_D = float(text)
        except:
            print("Wrong Type")
        self.change_DMP_button()
    def setBasisfunc(self,text):
        try:
            self.dmp_param_basisfunc = int(text)
        except:
            self.dmp_param_basisfunc = 50
            print("Wrong Type")
        self.change_DMP_button()
    def loadNamerec(self):
        self.dmp_record_name_load = self.ui.load_name_rec_comboBox.currentText()
        self.change_DMP_button()
    def saveWeights(self,text):
        self.dmp_weight_name = text
        print(self.dmp_weight_name)
        self.change_DMP_button()
    def generateDMP(self):
        try:
            if self.dmp_load_type == 0:
                self.dmp_mg.loadMotionFromEndEffector
            else:
                self.dmp_mg.loadMotionFromJointStates(self.dmp_record_name_load,self.arm,self.dmp_param_dt,self.dmp_param_K,self.dmp_param_D,self.dmp_param_basisfunc, self.dmp_weight_name)
            self.ui.generate_DMP_button.setStyleSheet("color: green")
            self.ui.generate_DMP_button.setText("DMP generated")
            self.ui.generate_DMP_button.setEnabled(False)
        except:
            self.ui.generate_DMP_button.setStyleSheet("background-color: red")
            self.ui.generate_DMP_button.setText("Recording not valid. Choose another.")
           

    def change_DMP_button(self):
        self.ui.generate_DMP_button.setStyleSheet("background-color: white")
        self.ui.generate_DMP_button.setEnabled(True)
        self.ui.generate_DMP_button.setText("Generate DMP")
    def browse_weights(self):
        self.ui.set_DMP_comboBox.clear()

        weight_dir = os.listdir(self.directory_weights)
        weight_dir = sorted(weight_dir)
        for file in weight_dir:
            if file.endswith(".yaml"):
                self.ui.set_DMP_comboBox.addItem(str(file))
    
    def setActiveDMP(self):
        self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
        self.selected_DMP = self.ui.set_DMP_comboBox.currentText()
    def sendActiveDMP(self):
        self.active_DMP = self.selected_DMP
        self.dmp_mg.loadMotionYAML(self.active_DMP)
        '''
        if self.avoidance_enabled:
            self.ui.execute_collisionCheck_PB.setEnabled(False)
            self.ui.execute_plan_pushButton.setEnabled(True)
        else:
            self.ui.execute_collisionCheck_PB.setEnabled(True)
            self.ui.execute_plan_pushButton.setEnabled(False)
        '''
        self.ui.execute_collisionCheck_PB.setEnabled(True)
        self.ui.execute_collisionCheck_PB.setText("Generate plan")
    def setJS_init_0(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.initial_JS[0] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_1(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.initial_JS[1] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_2(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.initial_JS[2] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_3(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.initial_JS[3] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_4(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.initial_JS[4] = float(text)
        except:
            print("Wrong Type")
    def setJS_init_5(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.initial_JS[5] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_0(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.goal_JS[0] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_1(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.goal_JS[1] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_2(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.goal_JS[2] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_3(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.goal_JS[3] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_4(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.goal_JS[4] = float(text)
        except:
            print("Wrong Type")
    def setJS_goal_5(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.goal_JS[5] = float(text)
        except:
            print("Wrong Type")
    def getJS_init_robot(self):
        # joint_states = rospy.wait_for_message("/joint_states", JointState)
        self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
        joint_states = get_joints()
        self.initial_JS = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        self.ui.set_JointState_init_0_lineEdit.setText(str(self.initial_JS[0]))
        self.ui.set_JointState_init_1_lineEdit.setText(str(self.initial_JS[1]))
        self.ui.set_JointState_init_2_lineEdit.setText(str(self.initial_JS[2]))
        self.ui.set_JointState_init_3_lineEdit.setText(str(self.initial_JS[3]))
        self.ui.set_JointState_init_4_lineEdit.setText(str(self.initial_JS[4]))
        self.ui.set_JointState_init_5_lineEdit.setText(str(self.initial_JS[5]))
    def getJS_goal_robot(self):
        # joint_states = rospy.wait_for_message("/joint_states", JointState)
        self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
        joint_states = get_joints()
        self.goal_JS = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        self.ui.set_JointState_goal_0_lineEdit.setText(str(self.goal_JS[0]))
        self.ui.set_JointState_goal_1_lineEdit.setText(str(self.goal_JS[1]))
        self.ui.set_JointState_goal_2_lineEdit.setText(str(self.goal_JS[2]))
        self.ui.set_JointState_goal_3_lineEdit.setText(str(self.goal_JS[3]))
        self.ui.set_JointState_goal_4_lineEdit.setText(str(self.goal_JS[4]))
        self.ui.set_JointState_goal_5_lineEdit.setText(str(self.goal_JS[5]))

    def setCS_init_tx(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.init_CS.pose.position.x = float(text)
            # print(self.init_CS.pose)
        except:
            print("Wrong Type")
    def setCS_init_ty(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.init_CS.pose.position.y = float(text)
            # print(self.init_CS.pose)
        except:
            print("Wrong Type")
    def setCS_init_tz(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.init_CS.pose.position.z = float(text)
            # print(self.init_CS.pose)
        except:
            print("Wrong Type")
    def setCS_init_rx(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.init_CS.pose.orientation.x = float(text)
        except:
            print("Wrong Type")
    def setCS_init_ry(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.init_CS.pose.orientation.y = float(text)
        except:
            print("Wrong Type")
    def setCS_init_rz(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.init_CS.pose.orientation.z = float(text)
        except:
            print("Wrong Type")
    def setCS_init_w(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.init_CS.pose.orientation.w = float(text)
        except:
            print("Wrong Type")
    def setCS_goal_tx(self,text):
        try:
            self.goal_CS.pose.position.x = float(text)
        except:
            print("Wrong Type")
    def setCS_goal_ty(self,text):
        try:
            self.goal_CS.pose.position.y = float(text)
        except:
            print("Wrong Type")
    def setCS_goal_tz(self,text):
        try:
            self.goal_CS.pose.position.z = float(text)
        except:
            print("Wrong Type")
    def setCS_goal_rx(self,text):
        try:
            self.goal_CS.pose.orientation.x = float(text)
        except:
            print("Wrong Type")
    def setCS_goal_ry(self,text):
        try:
            self.goal_CS.pose.orientation.y = float(text)
        except:
            print("Wrong Type")
    def setCS_goal_rz(self,text):
        try:
            self.goal_CS.pose.orientation.z = float(text)
        except:
            print("Wrong Type")
    def setCS_goal_w(self,text):
        try:
            self.goal_CS.pose.orientation.w = float(text)
        except:
            print("Wrong Type")
    def getCS_init_robot(self):
        joint_states = get_joints()
        self.init_JS_1 = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        
        self.result_FK_init = self.dmp_me.fkPath(self.init_JS_1,self.linkName)
        self.ui.set_CS_init_tx_lineEdit.setText(str(self.result_FK_init.position.x))
        self.ui.set_CS_init_ty_lineEdit.setText(str(self.result_FK_init.position.y))
        self.ui.set_CS_init_tz_lineEdit.setText(str(self.result_FK_init.position.z))
        self.ui.set_CS_init_rx_lineEdit.setText(str(self.result_FK_init.orientation.x))
        self.ui.set_CS_init_ry_lineEdit.setText(str(self.result_FK_init.orientation.y))
        self.ui.set_CS_init_rz_lineEdit.setText(str(self.result_FK_init.orientation.z))
        self.ui.set_CS_init_w_lineEdit.setText(str(self.result_FK_init.orientation.w))
    def getCS_goal_robot(self):
        joint_states = get_joints()
        self.goal_JS_1 = [joint_states.position[2],joint_states.position[1],joint_states.position[0],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
        
        self.result_FK_goal = self.dmp_me.fkPath(self.goal_JS_1,self.linkName)
        self.ui.set_CS_goal_tx_lineEdit.setText(str(self.result_FK_goal.position.x))
        self.ui.set_CS_goal_ty_lineEdit.setText(str(self.result_FK_goal.position.y))
        self.ui.set_CS_goal_tz_lineEdit.setText(str(self.result_FK_goal.position.z))
        self.ui.set_CS_goal_rx_lineEdit.setText(str(self.result_FK_goal.orientation.x))
        self.ui.set_CS_goal_ry_lineEdit.setText(str(self.result_FK_goal.orientation.y))
        self.ui.set_CS_goal_rz_lineEdit.setText(str(self.result_FK_goal.orientation.z))
        self.ui.set_CS_goal_w_lineEdit.setText(str(self.result_FK_goal.orientation.w))

    def get_IK(self):
        pass

    def setTau(self,text):
        try:
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: gray")
            self.tau = float(text)*5
        except:
            print("wrong input type")
    def getDMP_plan(self):
        pass
    def load_classes(self):
        self.ui.start_loadClasses_PB.setText(str("Starting application. Please wait..."))
        QApplication.processEvents()
        self.dmp_record_JS = RecordFromJointState()
        self.dmp_mg = motionGeneration()
        self.dmp_me = motionExecution()
        self.ui.stackedWidget.setCurrentIndex(1)
        if self.rviz:
            self.ui.mainwindow.setFixedSize(2200,1300)
            self.ui.frame_rviz.setVisible(True)

    def execute_plan(self):
        print('Executing plan')
        self.ui.avoidanceCheckBox.setEnabled(False)
        if self.avoidance_enabled:
            self.dmp_me.sendAvoidanceExecution(self.initial_JS,self.goal_JS,self.tau,self.dmp_param_dt)
            '''
            init_time_WP = time.time()
            initial_pose = self.initial_JS
            final_pose = self.goal_JS
            print("Start path plan")
            init_time = time.time()
            self.path_plan = self.dmp_mg.getPlan(initial_pose,final_pose,-1,[],None,self.tau,self.dmp_param_dt)
            fin_time = time.time()
            print ("Computing path done, took: " + str(fin_time - init_time))
            # print(self.path_plan.plan.points[0].positions)
            init_time_RT = time.time()
            robot_traj = self.dmp_me.robotTrajectoryFromPlan(self.path_plan,self.dmp_me.arm)
            fin_time_RT = time.time()
            print ("Converting to Robot trajectory done, took: " + str(fin_time_RT - init_time_RT)) 
            self.dmp_me.sendTrajectory(self.path_plan, self.initial_JS, self.goal_JS)
            '''
            status = self.dmp_me.recieveExecutionStatus() # 0-stopped 1-running 2-success 3-failed
            while status < 2: # While Stopped or Running
                status = self.dmp_me.recieveExecutionStatus() 
                time.sleep(1)
            st = True if status == 2 else False # Success or Failed
        else:
            st = self.dmp_me.sendTrajectoryAction(self.path_plan,self.initial_JS,self.sim, self.safe_stop_enabled)
            if not st:
                self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: red")
                self.ui.execute_plan_pushButton.setEnabled(False)
                print('Retrieveing pose')
                time.sleep(2)
                self.getJS_init_robot() # TODO repair sim seg fault
                time.sleep(2)
                print('Pose retrieved')
        print('Execute plan state: '+ ('success' if st else 'fail'))
        self.ui.avoidanceCheckBox.setEnabled(True)

    def collision_check(self):
        init_time_WP = time.time()
        initial_pose = self.initial_JS
        final_pose = self.goal_JS
        print("Start path plan")
        init_time = time.time()
        self.path_plan = self.dmp_mg.getPlan(initial_pose,final_pose,-1,[],None,self.tau,self.dmp_param_dt)
        fin_time = time.time()
        print ("Computing path done, took: " + str(fin_time - init_time))
        # print(self.path_plan.plan.points[0].positions)
        init_time_RT = time.time()
        robot_traj = self.dmp_me.robotTrajectoryFromPlan(self.path_plan,self.dmp_me.arm)
        fin_time_RT = time.time()
        print ("Converting to Robot trajectory done, took: " + str(fin_time_RT - init_time_RT))
        print("Checking collisions")
        init_time_CC = time.time()
        validity = self.dmp_me.checkTrajectoryValidity(robot_traj, avoidance=self.avoidance_enabled)
        fin_time_CC = time.time()
        print ("Collision checks done, took: " + str(fin_time_CC - init_time_CC))
        init_time_PP = time.time()
        self.dmp_me.pathPublish(self.path_plan,self.linkName)
        fin_time_PP = time.time()
        print("Doing FK and publishing path took, "+ str(fin_time_PP - init_time_PP))
        print("The whole process took , "+ str(fin_time_PP - init_time_WP))
        # check_ik = self.dmp_me.get_IK_from_Quart(self.result_FK_goal)
        # print(check_ik)
        if validity:
            print("Trajectory valid")
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: green")
            self.ui.execute_collisionCheck_PB.setText("Generate plan")
            self.ui.execute_plan_pushButton.setEnabled(True)
        else:
            print("Trajectory invalid")
            self.ui.execute_collisionCheck_PB.setStyleSheet("background-color: red")
            self.ui.execute_collisionCheck_PB.setText("Invalid plan (Click to regenerate)")
            self.ui.execute_plan_pushButton.setEnabled(False)
            
class RViz(QWidget ):

    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath( "" )
        self.frame.initialize()
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, os.getcwd()+"/src/roy_dmp/resources/dmp_config_old.rviz" )
        self.frame.load( config )
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        self.frame.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( True)
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        h_layout = QHBoxLayout()
        layout.addLayout( h_layout )
        self.setLayout( layout )
        



def main():
    rospy.init_node("DMP_GUI", anonymous=False)
    app = QtWidgets.QApplication(sys.argv)
    application = ApplicationWindow()
    application.show()
    #rviz = RViz()
    #rviz.resize( 500, 500 )
    #rviz.show()
    sys.exit(app.exec_())
    



if __name__ == "__main__":
    main()
