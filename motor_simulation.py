import time
import random
import os
#this current model aims to simulate controlling motors via step-wise concurrency opposed to true concurrency.
#this approach is take to avoid complex programming that may overwork the rpi cpu and maintain parrallel movement

# --- Motor movement and pickup functions ---
def move_motor_to_target(motor1_x, motor2_y, motor3_z, target, target_x, target_y):
    while (motor1_x, motor2_y) != (target_x, target_y):
        moved = False

        if motor1_x != target_x:
            motor1_x += 1 if motor1_x < target_x else -1
            moved = True

        if motor2_y != target_y:
            motor2_y += 1 if motor2_y < target_y else -1
            moved = True

        if moved:
            display_grid(motor1_x, motor2_y, motor3_z, target)
    
    return motor1_x, motor2_y

def motor3_pickup(motor1_x, motor2_y, motor3_z, target):
    motor3_z = 'd'
    display_grid(motor1_x, motor2_y, motor3_z, target)
    motor3_z = 'u'
    display_grid(motor1_x, motor2_y, motor3_z, target)
    return motor3_z

# --- Display grid and motor status ---
def display_grid(motor1_x, motor2_y, motor3_z, target):
    grid_height = 10
    grid_width = 10

    COLOR_RESET = "\033[0m"
    COLOR_MOTOR = "\033[1;32m"  # Bright Green
    COLOR_TARGET = "\033[1;34m"  # Bright Blue
    COLOR_DOWN = "\033[1;31m"    # Red
    COLOR_UP = "\033[1;33m"      # Yellow

    print("\n" + "="*40)

    for j in range(grid_height):
        row = ''
        for i in range(grid_width):
            if (i, j) == (motor1_x, motor2_y):
                row += f"{COLOR_MOTOR}[M]{COLOR_RESET}"
            elif (i, j) == target:
                row += f"{COLOR_TARGET}[T]{COLOR_RESET}"
            else:
                row += '[ ]'
        print(row)

    z_status = f"{COLOR_DOWN}DOWN{COLOR_RESET}" if motor3_z == 'd' else f"{COLOR_UP}UP{COLOR_RESET}"
    print(f"\nMotor Position: X={motor1_x}  Y={motor2_y}  Z={z_status}\n")

    input("Press Enter to step to next move...")

# --- MotorSimulator class ---
class MotorSimulator:
    def __init__(self, width=10, height=10):
        self.width = width
        self.height = height
        self.motor1_x = 0
        self.motor2_y = 0
        self.motor3_z = 'u'  # 'u' = up, 'd' = down

    def move_to_target_and_pick(self, target):
        print(f"\nNew target at {target}\n")
        self.motor1_x, self.motor2_y = move_motor_to_target(
            self.motor1_x, self.motor2_y, self.motor3_z, target, target[0], target[1]
        )
        self.motor3_z = motor3_pickup(self.motor1_x, self.motor2_y, self.motor3_z, target)

    def return_home_and_place(self):
        self.motor1_x, self.motor2_y = move_motor_to_target(
            self.motor1_x, self.motor2_y, self.motor3_z, (0, 0), 0, 0
        )
        self.motor3_z = motor3_pickup(self.motor1_x, self.motor2_y, self.motor3_z, (0, 0))

    def simulate_multiple_pick_and_place(self, num_targets=3):
        for _ in range(num_targets):
            target = (random.randint(0, self.width - 1), random.randint(0, self.height - 1))
            self.move_to_target_and_pick(target)
            self.return_home_and_place()

# --- Entry Point ---
if __name__ == "__main__":
    simulator = MotorSimulator()
    simulator.simulate_multiple_pick_and_place(num_targets=5)
