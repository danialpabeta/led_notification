import RPi.GPIO as GPIO
import rclpy
import time
import subprocess
from rclpy.node import Node
from darknet_ros_msgs.msg import BoundingBoxes

class LedSensor(Node):
    def __init__(self):
        super().__init__('led_sensor')

        # Inisialisasi GPIO
        GPIO.setmode(GPIO.BCM)
        self.led_pin1 = 13  # LED merah 
        self.led_pin2 = 12  # LED hijau
        GPIO.setup(self.led_pin1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.led_pin2, GPIO.OUT, initial=GPIO.LOW)

        # Subscriber ke darknet_ros/bounding_boxes
        self.subscriber_darknet = self.create_subscription(
            BoundingBoxes,
            '/robot_5/darknet_ros/bounding_boxes',
            self.bbox_callback,
            1
        )

        # Timer untuk cek apakah kamera masih aktif
        self.last_detection_time = time.time()
        self.timer = self.create_timer(2.0, self.check_camera_status)  # Cek setiap 2 detik

        # Timer untuk LED blink
        self.blink_timer = None
        self.blink_status = False  # Status ON/OFF LED kuning saat blink

    def bbox_callback(self, msg):
        #"""Dipanggil saat kamera mendeteksi objek."""
        self.last_detection_time = time.time()  # Update waktu terakhir deteksi

        bola_terdeteksi = False

        for bbox in msg.bounding_boxes:
            if bbox.class_id == "Ball":  # Jika objek yang terdeteksi adalah bola
                bola_terdeteksi = True
                break  # Keluar dari loop karena bola sudah ditemukan

        if bola_terdeteksi:
            GPIO.output(self.led_pin1, GPIO.LOW)  # Matikan LED merah
            GPIO.output(self.led_pin2, GPIO.HIGH)  # Nyalakan LED hijau
            self.get_logger().info("Bola terdeteksi! LED hijau ON")

            # Hentikan blink jika sedang berjalan
            if self.blink_timer:
                self.blink_timer.cancel()
                self.blink_timer = None
                GPIO.output(self.led_pin2, GPIO.LOW)  # Pastikan LED kuning mati
        else:
            GPIO.output(self.led_pin2, GPIO.LOW)  # Matikan LED merah
            GPIO.output(self.led_pin1, GPIO.HIGH)  # Nyalakan LED kuning
            self.get_logger().info("Tidak ada bola! LED merah ON")

    def check_camera_status(self):
        output = subprocess.check_output(['sudo', 'iwgetid']).decode()
        #"""Dijalankan setiap 2 detik untuk mengecek apakah kamera masih aktif."""
        if time.time() - self.last_detection_time > 2.0 #or output.split('"')[1] != "Polibatam":  # Jika lebih dari 2 detik tanpa deteksi
            self.get_logger().info("Kamera mati atau tidak mengirim data atau wifi tidak sesuai! LED merah BLINKING")

            # Aktifkan timer untuk blink jika belum berjalan
            GPIO.output(self.led_pin2, GPIO.LOW)
            GPIO.output(self.led_pin1, GPIO.LOW)
            #if self.blink_timer is None:
            #    self.blink_timer = self.create_timer(0.5, self.blink_led)

   # def blink_led(self):
        #"""Fungsi untuk membuat LED kuning berkedip setiap 0.5 detik."""
     #   self.blink_status = not self.blink_status  # Toggle status LED
     #   GPIO.output(self.led_pin1, GPIO.HIGH if self.blink_status else GPIO.LOW)
    

def main(args=None):
    rclpy.init(args=args)
    node = LedSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()  # Membersihkan GPIO saat program selesai

if __name__ == '__main__':
    main()

