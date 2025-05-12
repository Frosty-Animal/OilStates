import time, random, os

# --- Bresenham → unit‐step deltas ---
def bresenham_steps(dx, dy):
    sx, sy = (1 if dx>=0 else -1), (1 if dy>=0 else -1)
    dx_a, dy_a = abs(dx), abs(dy)
    err = dx_a - dy_a
    x = y = 0
    path = []
    while (x, y) != (dx, dy):
        e2 = err * 2
        step_x = step_y = 0
        if e2 > -dy_a:
            err -= dy_a
            x += sx
            step_x = sx
        if e2 < dx_a:
            err += dx_a
            y += sy
            step_y = sy
        path.append((step_x, step_y))
    return path

# --- Trapezoidal timing ---
def trapezoidal_durations(n, v_max, a):
    if n==0: return []
    # accel‐phase count
    n_a = int(v_max**2/(2*a))
    if 2*n_a > n:
        n_a = n//2
        v_peak = (a*n)**0.5
    else:
        v_peak = v_max
    n_c = n - 2*n_a
    dts = []
    for i in range(1, n_a+1):
        dts.append(1/((2*a*i)**0.5))
    dts += [1/v_peak]*n_c
    for i in range(n_a,0,-1):
        dts.append(1/((2*a*i)**0.5))
    return dts

# --- Display ---
def display_grid(x,y,z,target):
    os.system('cls' if os.name=='nt' else 'clear')
    H,W=10,10; C={'R':'\033[1;31m','G':'\033[1;32m','B':'\033[1;34m','Y':'\033[1;33m','0':'\033[0m'}
    for j in range(H):
        row=''
        for i in range(W):
            if (i,j)==(x,y):    row+=f"{C['G']}[M]{C['0']}"
            elif (i,j)==target: row+=f"{C['B']}[T]{C['0']}"
            else:               row+='[ ]'
        print(row)
    zs = f"{C['R']}DOWN" if z=='d' else f"{C['Y']}UP "
    print(f"\nX={x}  Y={y}  Z={zs}{C['0']}\n")

# --- Motor & pickup ---
def move_motor_to_target(x,y,tx,ty, v_max=5.0, accel=20.0):
    path = bresenham_steps(tx-x, ty-y)
    dts  = trapezoidal_durations(len(path), v_max, accel)
    for (dx,dy), dt in zip(path, dts):
        x += dx; y += dy
        display_grid(x,y,None,(tx,ty))
        time.sleep(dt)
    return x,y

def motor3_pickup(x,y,z,target):
    z='d'; display_grid(x,y,z,target); time.sleep(0.3)
    z='u'; display_grid(x,y,z,target); time.sleep(0.3)
    return z

# --- Simulator ---
class MotorSimulator:
    def __init__(self):
        self.x=0; self.y=0; self.z='u'
    def move_and_pick(self,t):
        self.x,self.y = move_motor_to_target(self.x,self.y,*t, v_max=8, accel=30)
        self.z = motor3_pickup(self.x,self.y,self.z,t)
    def return_home(self):
        self.x,self.y = move_motor_to_target(self.x,self.y,0,0, v_max=8, accel=30)
        self.z = motor3_pickup(self.x,self.y,self.z,(0,0))
    def run(self,n=5):
        for _ in range(n):
            tgt=(random.randrange(10), random.randrange(10))
            self.move_and_pick(tgt)
            self.return_home()

if __name__=="__main__":
    MotorSimulator().run()
