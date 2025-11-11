import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import serial
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')

        # — read values only from parameters — (all can be injected via YAML)
        self.declare_parameter(
            'serial_ports',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='List of serial port names (must not be empty)'
            )
        )
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('num_sensor', 1)            # How many sensors you want (used for horizontal concatenation)
        self.declare_parameter('height', 20)
        self.declare_parameter('width', 8)
        self.declare_parameter('rate_hz', 10.0)            # Publishing frequency
        self.declare_parameter('topic', '/sensor/matrix_sensor')

        # Read parameters
        serial_ports_param = self.get_parameter('serial_ports').get_parameter_value().string_array_value
        serial_ports = list(serial_ports_param)

        if len(serial_ports) == 0:
            self.get_logger().error(
                "❌ Parameter 'serial_ports' is required but not provided in YAML!"
            )
            raise RuntimeError("Missing required parameter: serial_ports")

        baud_rate = int(self.get_parameter('baud_rate').get_parameter_value().integer_value)
        self.num_sensor = int(self.get_parameter('num_sensor').get_parameter_value().integer_value)
        self.height = int(self.get_parameter('height').get_parameter_value().integer_value)
        self.width = int(self.get_parameter('width').get_parameter_value().integer_value)
        rate_hz = float(self.get_parameter('rate_hz').get_parameter_value().double_value)
        topic = self.get_parameter('topic').get_parameter_value().string_value

        # Port selection: if serial_ports (array) is provided, create sensors according to the array;
        # otherwise, only create a single-port sensor
        if len(serial_ports) > 0:
            self.ports = serial_ports
            print("serial ports: {}".format(serial_ports))
        else:
            print("no serial ports error")

        # Actual number of sensors to be concatenated: min(num_sensor, len(ports))
        self.num_sensor = len(serial_ports)

        # Create publisher and timer
        self.pub = self.create_publisher(Image, topic, 10)
        period = 1.0 / max(rate_hz, 0.1)
        self.timer = self.create_timer(period, self.timer_cb)
        self.frame = 0

        # Create sensor instances
        self.sensors = []
        for i in range(self.num_sensor):
            p = self.ports[i]
            fs = ForceSensor(
                port=p,
                baud_rate=baud_rate,
                num_rows=self.height,
                num_cols=self.width,
            )
            self.sensors.append(fs)
            self.get_logger().info(f'Opened sensor[{i}] on {p}')

        self.get_logger().info(
            f'sensor_publisher started: ports={self.ports[:self.num_sensor]}, '
            f'size={self.height}x{self.width}, num_sensor={self.num_sensor}, rate={rate_hz}Hz'
        )

    def timer_cb(self):
        # Read data frames according to the number of ports; if no frame yet, use the cached one (initialized to zeros)
        mats = []
        for fs in self.sensors:
            fs.find_frame()               # Block until one frame is received (your original logic)
            mats.append(fs.get_data())    # uint16 (H, W)

        # Concatenate horizontally: (H, W * num_sensor)
        full_u16 = np.hstack(mats) if len(mats) > 1 else mats[0]
        print(mats)

        # Publish as 32SC1 (according to your previous setup)
        full_i32 = full_u16.astype(np.int32, copy=False)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'
        msg.height = self.height
        msg.width = self.width * self.num_sensor
        msg.encoding = '32SC1'
        msg.is_bigendian = int(sys.byteorder == 'big')
        msg.step = msg.width * 4
        msg.data = full_i32.tobytes()

        self.pub.publish(msg)
        self.frame += 1


class ForceSensor:
    def __init__(self,
                 port: str,
                 baud_rate: int = 921600,
                 num_rows: int = 20,
                 num_cols: int = 8,
                 timeout: float = 0.01):
        self.port = port
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.BYTES_PER_PIXEL = 2
        self.FRAME_PAYLOAD_SIZE = num_rows * num_cols * self.BYTES_PER_PIXEL
        self.HEADER = bytes([0xFF, 0xFF])
        self.TOTAL_FRAME_SIZE = 2 + self.FRAME_PAYLOAD_SIZE

        self.ser = serial.Serial(port, baud_rate, timeout=timeout)
        self.data = np.zeros((num_rows, num_cols), dtype=np.uint16)

    def find_frame(self) -> np.ndarray:
        """
        Continuously read from the serial port until a complete frame is found,
        then return the parsed matrix.
        """
        buffer = b''
        while True:
            chunk = self.ser.read(512)
            if not chunk:
                continue

            buffer += chunk
            start = buffer.find(self.HEADER)
            if start != -1 and len(buffer) >= start + self.TOTAL_FRAME_SIZE:
                frame_bytes = buffer[start+2 : start+2 + self.FRAME_PAYLOAD_SIZE]
                buffer = buffer[start + self.TOTAL_FRAME_SIZE:]
                self.data = np.frombuffer(frame_bytes, dtype='>u2').reshape(
                    (self.num_rows, self.num_cols)
                )
                return self.data

            # Prevent the buffer from growing indefinitely
            if len(buffer) > 1024:
                buffer = buffer[-256:]

    def get_data(self) -> np.ndarray:
        return self.data


def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
