import socket
import numpy as np
from scipy.spatial.transform import Rotation as Rot


"""utility functions"""

# open a socket connection to the low-level controller
def connect2robot(PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('localhost', PORT))
    s.listen()
    conn, addr = s.accept()
    return conn

# send a joint velocity command to the low level controller
def send2robot(conn, qdot):
    max_scale = 1.0
    qdot = np.asarray(qdot)
    scale = np.linalg.norm(qdot)
    if scale > max_scale:
        qdot = [qdot[i] * max_scale / scale for i in range(7)]
    qdot += np.asarray([1e-5]*7)
    send_msg = np.array2string(qdot, precision=5, separator=',', suppress_small=True)[1:-1]
    send_msg = "s," + send_msg + ","
    conn.send(send_msg.encode())

# read in the state information sent back by low level controller
def listen2robot(conn):
    state_length = 7 + 7 + 7 + 42
    state_message = str(conn.recv(state_length*10+3))[2:-1]
    print(state_message)
    state_str = list(state_message.split(","))
    for idx in range(len(state_str)):
        if state_str[idx] == "s":
            state_str = state_str[idx+1:idx+1+state_length]
            break
    try:
        state = [float(item) for item in state_str]
    except ValueError:
        return (0, None, None, None, None)
    print(state)
    q = np.asarray(state[0:7])
    qdot = np.asarray(state[7:14])
    tau = np.asarray(state[14:21])
    J = state[21:]
    Jacobian = np.array([J[0:6],J[6:12],J[12:18],J[18:24],J[24:30],J[30:36],J[36:42]]).T
    return (1, q, qdot, tau, Jacobian)

# forward kinematics of robot, outputs position and euler angles
def joint2pose(q):
    def RotX(q):
        return np.array([[1, 0, 0, 0], [0, np.cos(q), -np.sin(q), 0], [0, np.sin(q), np.cos(q), 0], [0, 0, 0, 1]])
    def RotZ(q):
        return np.array([[np.cos(q), -np.sin(q), 0, 0], [np.sin(q), np.cos(q), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    def TransX(q, x, y, z):
        return np.array([[1, 0, 0, x], [0, np.cos(q), -np.sin(q), y], [0, np.sin(q), np.cos(q), z], [0, 0, 0, 1]])
    def TransZ(q, x, y, z):
        return np.array([[np.cos(q), -np.sin(q), 0, x], [np.sin(q), np.cos(q), 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
    H1 = TransZ(q[0], 0, 0, 0.333)
    H2 = np.dot(RotX(-np.pi/2), RotZ(q[1]))
    H3 = np.dot(TransX(np.pi/2, 0, -0.316, 0), RotZ(q[2]))
    H4 = np.dot(TransX(np.pi/2, 0.0825, 0, 0), RotZ(q[3]))
    H5 = np.dot(TransX(-np.pi/2, -0.0825, 0.384, 0), RotZ(q[4]))
    H6 = np.dot(RotX(np.pi/2), RotZ(q[5]))
    H7 = np.dot(TransX(np.pi/2, 0.088, 0, 0), RotZ(q[6]))
    H_panda_hand = TransZ(-np.pi/4, 0, 0, 0.2105)
    H = np.linalg.multi_dot([H1, H2, H3, H4, H5, H6, H7, H_panda_hand])
    p, R = H[:,3][:3], H[0:3, 0:3]
    return np.asarray(p), np.asarray(Rot.from_matrix(R).as_euler('xyz'))

# read full state info
def readState(conn):
    while True:
        (success, q, qdot, tau, J) = listen2robot(conn)
        if success:
            break
    x, R = joint2pose(q)
    xdot = J @ qdot
    return q, qdot, x, R, xdot, tau, J

# convert end-effector velocity command to joint velocity command
def xdot2qdot(xdot, Jacobian):
    Jacobian_inv = np.linalg.pinv(Jacobian)
    return Jacobian_inv @ np.asarray(xdot)


"""example script"""

def main():
    print("Started")
    conn = connect2robot(8080)
    print("I connected to the low-level controller")

    for idx in range(100):
        q, qdot, x, R, xdot, tau, J = readState(conn)
        print("I read the robot state")

        """ start of main part you will customize """

        xdot = [0] * 6
        # xdot[3] = 1.0  # telling the robot to move in +y axis
        xdot[4] = -0.5
        print(xdot)
        print("I picked my desired end-effector action")

        """ end of main part you will customize"""

        qdot = xdot2qdot(xdot, J)
        print("I converted that action to a joint velocity command")

        send2robot(conn, qdot)
        print("I sent that velocity command to the robot")


if __name__ == "__main__":
    main()
