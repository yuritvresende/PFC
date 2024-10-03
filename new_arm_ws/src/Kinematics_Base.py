import numpy as np
from numpy import pi, cos, sin, array, dot, eye, round, zeros, linalg, hstack
from scipy.linalg import pinv

# Definindo limites das juntas (upper limit - UL e lower limit - LL)
j1UL = (pi/180)*165; j1LL = -(pi/180)*165
j2UL = (pi/180)*110; j2LL = -(pi/180)*110
j3UL = (pi/180)*70; j3LL = -(pi/180)*110
j4UL = (pi/180)*160; j4LL = -(pi/180)*160
j5UL = (pi/180)*120; j5LL = -(pi/180)*120
j6UL = (pi/180)*400; j6LL = -(pi/180)*400

# Lista com os limites de cada junta
joint_limits = [
    (j1LL, j1UL),
    (j2LL, j2UL),
    (j3LL, j3UL),
    (j4LL, j4UL),
    (j5LL, j5UL),
    (j6LL, j6UL)
]

# Função que gera a matriz de transformação DH
def dh_matrix(theta, d, a, alpha):
    return array([
        [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta)],
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

def dh_matrixJ2(theta, d, a, alpha):
    return array([
        [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta)],
        [-cos(theta), sin(theta)*cos(alpha), -sin(theta)*sin(alpha), -a*cos(theta)],
        [0, sin(alpha), cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Parâmetros DH do manipulador robótico
dh_params = [
    {'d': 0.29, 'a': 0, 'alpha': -pi/2},
    {'d': 0, 'a': 0.27, 'alpha': 0},
    {'d': 0, 'a': 0.07, 'alpha': -pi/2},
    {'d': 0.302, 'a': 0, 'alpha': pi/2},
    {'d': 0, 'a': 0, 'alpha': -pi/2},
    {'d': 0.072, 'a': 0, 'alpha': 0}
]

# Função de cinemática direta
def forward_kinematics(q):
    T = eye(4)  # Matriz identidade 4x4
    for i, params in enumerate(dh_params):
        theta = q[i]
        if i != 1:
            A_i = dh_matrix(theta, params['d'], params['a'], params['alpha'])
        else:
            A_i = dh_matrixJ2(theta, params['d'], params['a'], params['alpha'])
        T = dot(T, A_i)
    position = T[:3, 3]  # Posição (x, y, z)
    return position

# Função para calcular a Jacobiana numericamente
def jacobian(q, delta=1e-6):
    J = zeros((3, len(q)))  # 3 para posição
    current_pos = forward_kinematics(q)
    
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += delta
        new_pos = forward_kinematics(dq)
        
        # Diferença na posição
        J[:3, i] = (new_pos - current_pos) / delta
    
    return J

# Função que verifica os limites das juntas
def check_joint_limits(q):
    assert len(q) == len(joint_limits), "O número de juntas não corresponde ao número de limites."
    
    for i, angle in enumerate(q):
        lower_limit, upper_limit = joint_limits[i]
        if angle < lower_limit:
            q[i] = lower_limit
        elif angle > upper_limit:
            q[i] = upper_limit
    return q

# Função de cinemática inversa que ajusta a posição
def inverse_kinematics(target_pos, initial_q, max_iterations=3000, tolerance=1e-6):
    q = initial_q.copy()
    for iteration in range(max_iterations):
        current_pos = forward_kinematics(q)
        
        pos_error = target_pos - current_pos
        
        if linalg.norm(pos_error) > tolerance:
            # Se a posição estiver fora do limite de tolerância, corrige a posição
            error = hstack([pos_error]) 
        else:
            return q
            
        # Atualiza as juntas
        J = jacobian(q)
        dq = dot(pinv(J), error)
        q += dq
        q = check_joint_limits(q)
    
    return q  # Retorna a última tentativa mesmo que não tenha convergido completamente

# Função de teste para a cinemática direta
def test_forward_kinematics(q):
    position = forward_kinematics(q)
    return round(position, 6)

# Função para formatar a string do comando ROS2
def format_ros2_action_command(joint_angles, speed=1.0):
    command = f'ros2 action send_goal -f /MoveJ ros2_data/action/MoveJ ' \
              f'\"{{goal: {{joint1: {joint_angles[0]:.2f}, joint2: {joint_angles[1]:.2f}, ' \
              f'joint3: {joint_angles[2]:.2f}, joint4: {joint_angles[3]:.2f}, ' \
              f'joint5: {joint_angles[4]:.2f}, joint6: {joint_angles[5]:.2f}}}, ' \
              f'speed: {speed}}}\"'
    return command

# Função de teste que verifica soluções com alinhamento ao plano XY
def test_inverse_kinematics(target_position, initial_q):
    solution = inverse_kinematics(target_position, initial_q)
    
    print(f'Solução encontrada: {round((180/pi)*solution, 6)}')
    pos = test_forward_kinematics(solution)
    print(f'Posição calculada (X, Y, Z): {pos}')
    error = round(target_position - pos, 6)
    print(f'Erro final: {error}')
        
    # Imprime o comando formatado para ROS2
    command = format_ros2_action_command(round((180/pi)*solution, 4))
    print(f'Comando ROS2 para enviar esta solução:\n{command}')

# Testando com uma posição alvo
initial_q = [0, 0, 0, 0, 0, 0]
target_position = array([0.4, -0.3, 0.2])

test_inverse_kinematics(target_position, initial_q)
