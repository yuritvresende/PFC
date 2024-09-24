import numpy as np
from numpy import pi, cos, sin, array, dot, eye, round, zeros, linalg, hstack
from scipy.linalg import pinv

# Definindo limites das juntas (upper limit - UL e lower limit - LL)
j1UL = 165; j1LL = -165
j2UL = 110; j2LL = -110
j3UL = 70; j3LL = -110
j4UL = 160; j4LL = -160
j5UL = 120; j5LL = -120
j6UL = 400; j6LL = -400

# Lista com os limites de cada junta
joint_limits = (pi/180)*[
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
    R = T[:3, :3]  # Matriz de rotação
    return position, R

# Função para calcular a Jacobiana numericamente
def jacobian(q, delta=1e-6):
    J = zeros((6, len(q)))  # 3 para posição, 3 para orientação
    current_pos, current_rot = forward_kinematics(q)
    
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += delta
        new_pos, new_rot = forward_kinematics(dq)
        
        # Diferença na posição
        J[:3, i] = (new_pos[0] - current_pos[0]) / delta
        
        # Diferença na orientação
        rot_diff = dot(new_rot, current_rot.T)  # Matriz de rotação relativa
        J[3:, i] = array([rot_diff[2, 1], rot_diff[0, 2], rot_diff[1, 0]]) / delta  # Extrai ângulos de Euler aproximados
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

# Função de cinemática inversa que também considera orientação
def inverse_kinematics(target_pos, target_orientation, initial_q, max_iterations=1000, tolerance=1e-6):
    q = initial_q.copy()
    for iteration in range(max_iterations):
        current_pos, current_rot = forward_kinematics(q)
        
        # Erro de posição
        pos_error = target_pos - current_pos
        
        # Erro de orientação (usando ângulos de Euler)
        rot_error = dot(current_rot.T, target_orientation)  # Matriz de rotação relativa
        euler_error = array([rot_error[2, 1], rot_error[0, 2], rot_error[1, 0]])  # Aproximação com ângulos
        
        # Combinando erro de posição e orientação
        error = hstack([pos_error, euler_error])
        
        if linalg.norm(error) < tolerance:
            return q
        
        J = jacobian(q)
        dq = dot(pinv(J), error)
        q += dq
        q = check_joint_limits(q)
    
    return None  # Retorna None se não convergir

# Função de teste para a cinemática direta
def test_forward_kinematics(q):
    position, _ = forward_kinematics(q)
    return round(position, 4)

# Função para formatar a string do comando ROS2
def format_ros2_action_command(joint_angles, speed=1.0):
    command = f'ros2 action send_goal -f /MoveJ ros2_data/action/MoveJ ' \
              f'\"{{goal: {{joint1: {joint_angles[0]:.4f}, joint2: {joint_angles[1]:.4f}, ' \
              f'joint3: {joint_angles[2]:.4f}, joint4: {joint_angles[3]:.4f}, ' \
              f'joint5: {joint_angles[4]:.4f}, joint6: {joint_angles[5]:.4f}}}, ' \
              f'speed: {speed}}}\"'
    return command

# Função de teste que verifica soluções com orientação
def test_inverse_kinematics_with_orientation(target_position, target_orientation, initial_q):
    solution = inverse_kinematics(target_position, target_orientation, initial_q)
    
    if solution is not None:
        print(f'Solução encontrada: {round((180/pi)*solution, 4)}')
        pos = test_forward_kinematics(solution)
        print(f'Posição calculada (X, Y, Z): {pos}')
        error = round(target_position - pos, 4)
        print(f'Erro final: {error}')
        
        # Imprime o comando formatado para ROS2
        command = format_ros2_action_command(solution)
        print(f'Comando ROS2 para enviar esta solução:\n{command}')
    else:
        print("Solução não convergiu.")

# Testando com uma posição alvo e uma orientação alvo
initial_q = [0, 0, 0, 0, 0, 0]
target_position = array([0.5, 0.3, 0.75])
# Orientação desejada: paralelo ao plano XY e girado por pi/4 ao redor do eixo Z
target_orientation = array([
    [cos(pi/4), -sin(pi/4), 0],
    [sin(pi/4),  cos(pi/4), 0],
    [0, 0, 1]
])

test_inverse_kinematics_with_orientation(target_position, target_orientation, initial_q)
