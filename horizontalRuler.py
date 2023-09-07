import time
import sys
import numpy as np
import threading
import qt_utils as qtutils
import utils
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from extraqtwidgets import *
from evo_comms import *
from evo_widgets import JointController, ScalerBar
from evo_statemachine import EvoStateMachine, EvoState, QStateExt
from evo_video_service import video_service
from evo_serial_comm import serial_comm
from evo_config_service import evo_config_service
from test_code.test_pat_design import ruler_horizontal


from evo_styles import *
from joystick_controller import *
from sksurgerynditracker.nditracker import NDITracker


class tracker:
    def Connect_tracker(self):
        settings_aurora = {
            "tracker type": "aurora",
            "ports to probe": 2,
            "verbose": True,
        }
        self.em_connection = NDITracker(settings_aurora)

    def toggle_em(self):
        self.em_attached = not self.em_attached
        if self.em_attached:
            try:
                self.Connect_tracker()
                self.em_connection.start_tracking()
                self.em_button.setStyleSheet(
                    "background-color: {}; color: black;".format(self.em_attached_color))
            except Exception as error:
                print(error)
                self.em_attached = False
                print("EM not connected - Connect EM sensor and try again")
        else:
            self.em_button.setStyleSheet(
                "background-color: {}; color: black;".format(self.em_detached_color))
            self.em_connection.stop_tracking()
            self.em_connection.close()
            del self.em_connection
            print("EM Connection Closed")

    def toggle_em_data(self):  # Need to add the statements here
        self.em_data_button.setStyleSheet(
            "background-color: {}; color: black;".format(self.em_attached_color))
        return 0


class EvoApp(QMainWindow):
    def __init__(self, *args, **kwargs):
        super(EvoApp, self).__init__(*args, **kwargs)
        self.resize(QSize(1280, 720))
        # TODO: argparse
        evo = EvoMainWidget(sys.argv[1:])
        self.setCentralWidget(evo)
        self.show()


class EvoMainWidget(QWidget):
    def __init__(self, autocal, *args, **kwargs):
        super(EvoMainWidget, self).__init__(*args, **kwargs)

        # DDS Comms should all be kept at top level for cleanliness
        self.evo_comms = EvoEngGuiComms()
        time.sleep(0.1)

        # Control panel
        self.controlpanel = EvoControlPanel(self)
        self.controlpanel.publishJointAngles.connect(self.set_joint_angles)
        self.controlpanel.publishCatheterPresent.connect(
            self.set_catheter_present)
        self.controlpanel.publishScaleFactor.connect(self.set_scale_factor)

        # Config service
        config_service = evo_config_service('./Config/config.ini')
        self.config_data = config_service.read_config()

        # Serial communication
        self.serial_comm = serial_comm(self.config_data)
        msg_received_event = self.serial_comm.get_message_receieved_event()
        server_thread = threading.Thread(target=self.listen_to_serial_port)
        server_thread.daemon = True
        server_thread.start()

        msg_received_event.subscribe(
            on_next=lambda xyz: self.handle_serial_message(*xyz),
            on_error=lambda error: print(f'Error: {error}'),
            on_completed=print('Successfully Completed')
        )

        # Video
        self.video_service = video_service(self.config_data)
        self.video_label = QLabel(self)
        # self.ruler_widget.setContentsMargins(20, 20, 20, 20)

        self.video_title = QLabel('Catheter Camera:', self)
        self.video_title.setFont(QFont('Arial', 13))

        # State Machine
        self.statemachine = EvoStateMachine(self)
        self.statemachine.setMinimumWidth(500)
        self.statemachine.setContentsMargins(0, 0, 10, 0)
        self.statemachine.add_transition(self.statemachine.INIT_PRE_CAL,
                                         self.statemachine.IDU_STARTED_CALIBRATING,
                                         self.controlpanel.startCalibration)
        self.statemachine.INIT_PRE_CAL.entered.connect(
            self.controlpanel.calibration_ready)
        self.statemachine.IDU_STARTED_CALIBRATING.entered.connect(
            self.controlpanel.calibration_started)
        self.statemachine.IDU_STARTED_CALIBRATING.exited.connect(
            self.controlpanel.calibration_complete)
        self.statemachine.HOLD.entered.connect(
            self.controlpanel.zero_joint_controllers)
        self.statemachine.HOLD.entered.connect(
            lambda: self.enable_controllers(False))
        self.statemachine.HOLD.exited.connect(
            lambda: self.enable_controllers(True))
        # Connect states to publish DDS
        for state in self.statemachine.states:
            state.onentry.connect(self.set_gui_state_desired)
        self.state_received = False

        # # LOCAL TESTING PURPOSES - REMOVE STATEMACHINE START
        # self.statemachine.set_initial_state(self.statemachine.PRE_INIT)
        # self.statemachine.start()

        self.joystick_controller = JoystickController(self)
        self.joystick_controller.joystick_xy.moved.connect(
            self.set_joystick_xy)
        self.joystick_controller.joystick_xz.moved.connect(
            self.set_joystick_xz)

        # Disable joint control until state is POSITION_CONTROL
        self.enable_controllers(False)

        # E-Stop button commands to HOLD
        e_stop_button = QPushButton("TOGGLE ESTOP")
        e_stop_button.setStyleSheet("background-color: red; color: black;")
        e_stop_button.clicked.connect(self.toggle_e_stop)

        # DEV ONLY *************************

        # DEV ONLY *************************

        main_layout = QGridLayout()
        main_layout.setSpacing(0)
      
        self.ruler_horizontal = ruler_horizontal()
        self.ruler_horizontal.resize(700, 100);
        main_layout.addWidget(self.ruler_horizontal, 1, 3, 2, 1, Qt.AlignTop)

        main_layout.addWidget(self.video_label, 0, 3, 2, 1)
        main_layout.addWidget(self.video_title, 0, 3, 2, 1, Qt.AlignLeft | Qt.AlignTop)
        # main_layout.addWidget(self.ruler_widget, 0, 3, 2, 1)
        # main_layout.itemAt(0).widget().raise_()
         
        main_layout.addWidget(self.statemachine, 0, 0, 3, 1)
        main_layout.addWidget(self.controlpanel, 0, 1)
        main_layout.addWidget(self.joystick_controller, 1, 1)
        main_layout.addWidget(e_stop_button, 2, 1)
        main_layout.setAlignment(Qt.AlignTop)
        main_layout.setVerticalSpacing(50);
        self.setLayout(main_layout)

        # Start comms
        self.continuous_update_freq = 1000  # 1kHz
        self.init_continuous_thread()

        self.state_update_freq = 100  # 100 Hz
        self.init_state_thread()

        self.em_update_freq = 50  # 50 Hz
        self._em_thread = None
        self.init_em_data()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(20)  # Update frame every 30 ms (you can adjust this)

        # Flag for starting calibration automatically after a delay
        if autocal:
            # utils.async_wait(10, self.controlpanel.start_calibration)  #TODO reimplement autocalibration
            pass

    def listen_to_serial_port(self):
        self.serial_comm.start_serial_listener()

    def handle_serial_message(self, arg1, arg2, arg3, arg4, arg5):
        # print(f"Event fired with arguments: {arg1}, {arg2}, {arg3}")
        if (len(arg1) == 0 or len(arg2) == 0 or len(arg2) == 0):
            rotation_angle = 0.0
        x, y, z = float(arg1[:5]), float(arg2[:5]), float(arg3[:5])

        # rotation_angle = self.accel_to_single_angle(x,y,z)
        rotation_angle = self.accelerometer_to_angle(x, y, z)
        # if(z > 0):
        #     rotation_angle = 180 - rotation_angle

        self.rotation_angle = rotation_angle
        # completed = self.progressbar_horz['value'] >= 100 or self.progressbar_vert['value'] >= 100

        # try:
        #     self.handle_encoders_data(arg4, arg5)
        # except Exception as e:
        #     print(e)

    def handle_encoders_data(self, e1, e2):
        try:
            x = (float(e1) * self.canvas_width)/32_000
            y = (float(e2) * self.canvas_height)/32_000

            self.canvas.moveto(self.arrow_top_id, x, 530)
            self.canvas.moveto(self.arrow_bottom_id, x, 0)
            self.canvas.moveto(self.arrow_right_id, 700, y)
            self.canvas.moveto(self.arrow_left_id, 0, y)

        except Exception as e:
            print(f'handle_encoders_data: error: {e}')

    def accelerometer_to_angle(self, x, y, z):
        # Calculate pitch angle (rotation about x-axis)
        # pitch = math.degrees(math.atan2(x, math.sqrt(y**2 + z**2)))

        # Calculate roll angle (rotation about y-axis)
        roll = math.degrees(math.atan2(y, math.sqrt(x**2 + z**2)))
        if (z < 0):
            roll = 180 - roll
        # return pitch, roll
        # print(f'roll: {roll}')
        return roll

    def update_frame(self):
        # ret, frame = self.cap.read()
        frame = self.video_service.read_frame()
        if frame is not None:
            height, width, channel = frame.shape
            bytes_per_line = 3 * width
            frame = self.video_service.rotate_frame(frame, self.rotation_angle)
            q_image = QImage(frame.data, width, height, bytes_per_line,
                             QImage.Format_RGB888)  # .rgbSwapped()
            pixmap = QPixmap.fromImage(q_image)
            self.video_label.setPixmap(pixmap)

    def toggle_e_stop(self):
        """ Sends HOLD command to KM
        """
        self.evo_comms.state_desired = EvoState.HOLD.value
        self.evo_comms.publish_EngGuiToKinManagerState()

    def set_scale_factor(self, scale_factor):
        self.evo_comms.scale_factor = scale_factor
        self.evo_comms.publish_EngGuiToKinManagerState()

    def set_joint_angles(self, angles):
        self.evo_comms.joints_desired = angles

    def set_joystick_xy(self, x, y):
        self.evo_comms.joystick_inputs[0] = x
        self.evo_comms.joystick_inputs[1] = y

    def set_joystick_xz(self, x, z):
        self.evo_comms.joystick_inputs[2] = x
        self.evo_comms.joystick_inputs[3] = z

    def set_gui_state_desired(self, state: QStateExt):
        self.evo_comms.state_desired = state.property("idx")
        self.evo_comms.publish_EngGuiToKinManagerState()

    def set_catheter_present(self, cath_is_present):
        self.evo_comms.catheter_present = cath_is_present
        self.evo_comms.publish_EngGuiToKinManagerState()

    def init_continuous_thread(self):
        self._continuous_thread = threading.Thread(
            target=self.poll_continuous, args=())
        self._continuous_thread.daemon = True
        self._continuous_thread.start()

    def init_state_thread(self):
        self._state_thread = threading.Thread(target=self.poll_state, args=())
        self._state_thread.daemon = True
        self._state_thread.start()

    def poll_continuous(self):
        """ Pub/sub for EG-KM DDS continuous struct
        """
        while True:
            self.evo_comms.publish_EngGuiToKinManagerContinuous()
            self.evo_comms.read_KinManagerToEngGuiContinuous()
            self.update_continuous_actual()
            time.sleep(1 / self.continuous_update_freq)

    def poll_state(self):
        """ Pub/sub for EG-KM DDS state struct
        """
        while True:
            self.evo_comms.read_KinManagerToEngGuiState()
            if self.evo_comms.KinManagerToEngGuiState_changed:
                self.update_states_actual()
            time.sleep(1 / self.state_update_freq)

    def update_states_actual(self):
        self.evo_comms.init_KinManagerToEngGuiState()
        self.controlpanel.update_states_actual(
            self.evo_comms.KinManagerToEngGuiState)
        curr_state = self.statemachine.get_state(
            int(self.evo_comms.state_actual))
        if not self.state_received:  # Initialize statemachine when state_actual received from KM, currently needs restart to reinit
            self.statemachine.set_initial_state(curr_state)
            self.statemachine.start()
            self.state_received = True
        self.statemachine.request_state(
            self.evo_comms.KinManagerToEngGuiState['state_actual'])

    def init_em_data(self):
        self._em_thread = threading.Thread(target=self.poll_em_data, args=())
        self._em_thread.daemon = True
        self._em_thread.start()

    def poll_em_data(self):
        while True:
            try:
                self.controlpanel.em_tracker.em_connection

            except:
                NameError

            else:
                em_data = self.controlpanel.em_tracker.em_connection.get_frame()
                Transform = em_data[3][0]
                em_data_x = Transform[0, 3]
                em_data_y = Transform[1, 3]
                em_data_z = Transform[2, 3]

                em_coords = (em_data_x, em_data_y, em_data_z)

                self.controlpanel.poll_em_data(em_coords)

            time.sleep(1 / self.em_update_freq)

    def init_em_data(self):
        self._em_thread = threading.Thread(target=self.poll_em_data, args=())
        self._em_thread.daemon = True
        self._em_thread.start()

    def poll_em_data(self):
        while True:
            try:
                self.controlpanel.em_tracker.em_connection

            except:
                NameError

            else:
                em_data = self.controlpanel.em_tracker.em_connection.get_frame()
                Transform = em_data[3][0]
                em_data_x = Transform[0, 3]
                em_data_y = Transform[1, 3]
                em_data_z = Transform[2, 3]

                em_coords = (em_data_x, em_data_y, em_data_z)

                self.controlpanel.poll_em_data(em_coords)

            time.sleep(1 / self.em_update_freq)

    def update_continuous_actual(self):
        self.evo_comms.timestamp_eg_to_km = int(
            self.evo_comms.timestamp_km_to_eg)
        self.evo_comms.init_KinManagerToEngGuiContinuous()
        self.controlpanel.update_continuous_actual(
            self.evo_comms.KinManagerToEngGuiContinuous)

    def enable_controllers(self, enabled):
        self.controlpanel.enable_controllers(enabled)
        self.joystick_controller.enable_joysticks(enabled)


class EvoControlPanel(QWidget):
    publishJointAngles = pyqtSignal(list)  # list[4]: joint angles
    publishScaleFactor = pyqtSignal(float)
    publishCatheterPresent = pyqtSignal(bool)
    startCalibration = pyqtSignal()

    def __init__(self, *args, **kwargs):
        super(EvoControlPanel, self).__init__(*args, **kwargs)
        self.joint_control = QHBoxLayout()
        self.joint_controllers = [JointController("Motor 0"), JointController(
            "Motor 1"), JointController("Motor 2"), JointController("Motor 3")]

        for jc in self.joint_controllers:
            jc.angleChanged.connect(self.publish_joint_angles)
            self.joint_control.addWidget(jc)

        self.scaler_control = QHBoxLayout()
        self.scaler_controller = ScalerBar("Scale factor")
        self.scaler_controller.setMaximumWidth(400)
        self.scaler_controller.scaleFactorChanged.connect(
            self.publish_scale_factor)

        self.calibrate_IDU_button = QPushButton("Calibrate IDU")
        # TODO: We'll want to make sure the user can't spam the calibrate button until it either fails or succeeds in the future
        self.calibrate_IDU_button.clicked.connect(self.startCalibration.emit)

        self.actual_state = QLabel(EvoState.PRE_INIT.name)
        self.actual_state.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.actual_state.setMinimumWidth(200)

        self.actual_e_stop_state = QLabel("e_stop_actual: False")
        self.actual_e_stop_state.setSizePolicy(
            QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.actual_e_stop_state.setMinimumWidth(200)

        self.actual_cath_state = QLabel("cath_is_homed: False")
        self.actual_cath_state.setSizePolicy(
            QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.actual_cath_state.setMinimumWidth(200)

        self.actual_joints = QLabel("joints_actual: 0, 0, 0, 0")
        self.actual_joints.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.actual_joints.setMinimumWidth(200)

        self.actual_position = QLabel("position_actual: 0, 0 ,0")
        self.actual_position.setSizePolicy(
            QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.actual_position.setMinimumWidth(200)

        self.actual_timestamp = QLabel("timestamp_km_to_eg: 0")
        self.actual_timestamp.setSizePolicy(
            QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.actual_timestamp.setMinimumWidth(200)

        self.em_tracker_data = QLabel("EM Data: 0, 0, 0")
        self.em_tracker_data.setSizePolicy(
            QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.em_tracker_data.setMinimumWidth(200)

        self.em_tracker_data = QLabel("EM Data: 0, 0, 0")
        self.em_tracker_data.setSizePolicy(
            QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.em_tracker_data.setMinimumWidth(200)

        self.live_data = QVBoxLayout()
        self.live_data.addWidget(self.actual_state)
        self.live_data.addWidget(self.actual_e_stop_state)
        self.live_data.addWidget(self.actual_cath_state)
        self.live_data.addWidget(self.actual_joints)
        self.live_data.addWidget(self.actual_position)
        self.live_data.addWidget(self.actual_timestamp)
        self.live_data.addWidget(self.em_tracker_data)

        self.cath_button = QPushButton("Catheter Attached")
        self.cath_button.setMaximumWidth(400)
        self.cath_detached_color = "#AD0000"
        self.cath_attached_color = "#2ED700"
        self.cath_button.setStyleSheet(
            "background-color: {}; color: black;".format(self.cath_detached_color))
        self.cath_present = False
        self.cath_button.clicked.connect(self.toggle_cath)

        self.em_tracker = tracker()

        self.em_tracker.em_button = QPushButton("EM Sensor Attached")
        self.em_tracker.em_detached_color = "#AD0000"
        self.em_tracker.em_attached_color = "#2ED700"
        self.em_tracker.em_button.setStyleSheet(
            "background-color: {}; color: black;".format(self.em_tracker.em_detached_color))
        self.em_tracker.em_attached = False
        self.em_tracker.em_button.clicked.connect(self.em_tracker.toggle_em)

        self.em_tracker.em_data_button = QPushButton("Collect EM Data")
        self.em_tracker.em_wait_color = "#AD0000"
        self.em_tracker.em_collect_color = "#2ED700"
        self.em_tracker.em_data_button.setStyleSheet(
            "background-color: {}; color: black;".format(self.em_tracker.em_wait_color))
        self.em_tracker.em_collect = False
        self.em_tracker.em_data_button.clicked.connect(
            self.em_tracker.toggle_em_data)

        main_layout = QGridLayout(self)
        main_layout.addLayout(self.live_data, 1, 0)
        main_layout.addLayout(self.joint_control, 1, 1)
        main_layout.addWidget(self.cath_button, 1, 2)
        main_layout.addWidget(self.em_tracker.em_button, 2, 2)
        main_layout.addWidget(self.calibrate_IDU_button, 2, 1)
        main_layout.addWidget(self.scaler_controller, 0, 2)

        self.setLayout(main_layout)

    def calibration_ready(self):
        self.calibrate_IDU_button.setEnabled(True)
        self.calibrate_IDU_button.setStyleSheet(
            "background-color: rgb{}; color: black;".format(EvoStyles.enabled_color))

    def calibration_started(self):
        self.calibrate_IDU_button.setEnabled(False)
        self.calibrate_IDU_button.setText("Calibrating...")
        self.calibrate_IDU_button.setStyleSheet(
            "background-color: rgb{}; color: grey;".format(EvoStyles.inprogress_color))

    def calibration_complete(self):
        self.calibrate_IDU_button.setText("IDU Calibration Complete")
        self.calibrate_IDU_button.setStyleSheet(
            "color: rgb{};".format(EvoStyles.enabled_color))

    def update_states_actual(self, KinManagerToEngGuiState):
        """ Sets live data text display to reflect states actuals values 

        Args:
            KinManagerToEngGuiState (dict): Dict from comms containing states actuals data from KM
        """
        state_actual = KinManagerToEngGuiState['state_actual']
        e_stop_actual = KinManagerToEngGuiState['e_stop_actual']
        cath_is_homed = KinManagerToEngGuiState['cath_is_homed']
        self.actual_state.setText(EvoState(state_actual).name)
        self.actual_e_stop_state.setText(
            str("cath_is_homed: " + str(cath_is_homed)))
        self.actual_cath_state.setText(
            str("e_stop_actual: " + str(e_stop_actual)))

    def update_continuous_actual(self, KinManagerToEngGuiContinuous):
        """ Sets live data text display to reflect continuous actuals values

        Args:
            KinManagerToEngGuiContinuous (dict): Dict from comms containing continuous actuals data from KM
        """
        joints_actual = KinManagerToEngGuiContinuous['joints_actual']
        position_actual = KinManagerToEngGuiContinuous['position_actual']
        timestamp_km_to_eg = KinManagerToEngGuiContinuous['timestamp_km_to_eg']
        self.actual_joints.setText(
            str("joints_actual: \n" + str(np.around(joints_actual, 2))))
        self.actual_position.setText(
            str("position_actual: \n" + str(np.around(position_actual, 2))))
        self.actual_timestamp.setText(
            str("timestamp_km_to_eg: " + str(timestamp_km_to_eg)))

    def poll_em_data(self, em_coords):
        self.em_tracker_data.setText(
            "EM Data: \n" + str(np.around(em_coords, 2)))

    def zero_joint_controllers(self):
        for jc in self.joint_controllers:
            jc.zero()

    def enable_controllers(self, is_enabled):
        for jc in self.joint_controllers:
            jc.minus_button.setEnabled(is_enabled)
            jc.plus_button.setEnabled(is_enabled)
            jc.slider.setEnabled(is_enabled)
        self.calibrate_IDU_button.setEnabled(is_enabled)

    def toggle_cath(self):
        self.cath_present = not self.cath_present
        if self.cath_present:
            self.cath_button.setStyleSheet(
                "background-color: {}; color: black;".format(self.cath_attached_color))
            self.publishCatheterPresent.emit(True)
        else:
            self.cath_button.setStyleSheet(
                "background-color: {}; color: black;".format(self.cath_detached_color))
            self.publishCatheterPresent.emit(False)

    def publish_joint_angles(self):
        self.publishJointAngles.emit([jc.angle()
                                     for jc in self.joint_controllers])

    def publish_scale_factor(self):
        self.publishScaleFactor.emit(self.scaler_controller.scale_factor())


if __name__ == "__main__":
    app = QApplication(sys.argv)
    qtutils.set_darkmode_palette(app)
    app.setApplicationName("EVO GUI")

    window = EvoApp()

    sys.exit(app.exec())
