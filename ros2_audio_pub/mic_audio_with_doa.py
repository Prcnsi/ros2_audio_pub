import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from AudioWithDOA.msg import AudioWithDOA

import subprocess
import numpy as np
import usb.core


class MicAudioWithDOAPublisher(Node):
    def __init__(self):
        super().__init__('mic_audio_with_doa_publisher')

        self.publisher_ = self.create_publisher(AudioWithDOA, 'mic_audio_doa', 10)
        self.chunk_size = 441  # 44100Hz / 100Hz = 441 samples per chunk

        # Start arecord subprocess
        self.proc = subprocess.Popen(
            ['arecord', '-f', 'S16_LE', '-r', '44100', '-c', '1', '-t', 'raw'],
            stdout=subprocess.PIPE
        )
        self.get_logger().info('Started arecord for audio capture.')

        # Find ReSpeaker HID device
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if self.dev is None:
            self.get_logger().error('ReSpeaker device not found.')
            raise RuntimeError('ReSpeaker USB Mic Array not found')

        # 100Hz timer
        self.create_timer(1.0 / 100, self.timer_callback)

    def timer_callback(self):
        try:
            raw = self.proc.stdout.read(self.chunk_size * 2)  # 2 bytes per sample
            if not raw:
                self.get_logger().warn("No audio data received.")
                return

            # Convert raw audio to Float32
            audio = np.frombuffer(raw, dtype=np.int16).astype(np.float32) / 32768.0

            # Read DOA from HID
            try:
                doa_val = int(self.dev.ctrl_transfer(0xC0, 0x01, 0, 0, 4)[0])
            except Exception as e:
                self.get_logger().warn(f"DOA read error: {e}")
                doa_val = -1

            # Construct and publish message
            msg = AudioWithDOA()
            msg.audio = Float32MultiArray(data=audio.tolist())
            msg.doa = Int32(data=doa_val)
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Unhandled error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MicAudioWithDOAPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.proc.terminate()
        node.destroy_node()
        rclpy.shutdown()
