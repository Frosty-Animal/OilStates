
from multiprocessing import Process, Manager
import time
import random
import os

class MotorSimulator:
    def __init__(self, width=10, height=10):
        self.width = width
        self.height = height
        self.manager = Manager()
        self.motor1_x = self.manager.Value('i', 0)
        self.motor2_y = self.manager.Value('i', 0)
        self.motor3_z = self.manager.Value('c', b'u')  # 'u' = up, 'd' = down
        self.target = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))

    def display_grid(self):
        os.system('cls' if os.name == 'nt' else 'clear')
        for y in range(self.height):
            row = ''
            for x in range(self.width):
                if (x, y) == (self.motor1_x.value, self.motor2_y.value):
                    row += '[M]'
                elif (x, y) == self.target:
                    row += '[T]'
                else:
                    row += '[ ]'
            print(row)
        print(f"\nMotor3 Z-axis: {'DOWN' if self.motor3_z.value == b'd' else 'UP'}")

    def move_motor1_to_x(self, target_x):
        while self.motor1_x.value != target_x:
            if self.motor1_x.value < target_x:
                self.motor1_x.value += 1
            else:
                self.motor1_x.value -= 1
            self.display_grid()
            time.sleep(0.2)

    def move_motor2_to_y(self, target_y):
        while self.motor2_y.value != target_y:
            if self.motor2_y.value < target_y:
                self.motor2_y.value += 1
            else:
                self.motor2_y.value -= 1
            self.display_grid()
            time.sleep(0.2)

    def motor3_pickup(self):
        self.motor3_z.value = b'd'
        self.display_grid()
        time.sleep(1)
        self.motor3_z.value = b'u'
        self.display_grid()

    def home_all_motors(self):
        p1 = Process(target=self.move_motor1_to_x, args=(0,))
        p2 = Process(target=self.move_motor2_to_y, args=(0,))
        p1.start()
        p2.start()
        p1.join()
        p2.join()
        print("\nAll motors homed.")

    def simulate_pick_and_place(self):
        print(f"Target is at {self.target}\n")

        # Move to target
        p1 = Process(target=self.move_motor1_to_x, args=(self.target[0],))
        p2 = Process(target=self.move_motor2_to_y, args=(self.target[1],))
        p1.start()
        p2.start()
        p1.join()
        p2.join()

        # Pickup
        p3 = Process(target=self.motor3_pickup)
        p3.start()
        p3.join()

        # Return to origin
        p4 = Process(target=self.move_motor1_to_x, args=(0,))
        p5 = Process(target=self.move_motor2_to_y, args=(0,))
        p4.start()
        p5.start()
        p4.join()
        p5.join()

        # Place
        p6 = Process(target=self.motor3_pickup)
        p6.start()
        p6.join()

if __name__ == "__main__":
    simulator = MotorSimulator()
    simulator.home_all_motors()
    simulator.simulate_pick_and_place()
