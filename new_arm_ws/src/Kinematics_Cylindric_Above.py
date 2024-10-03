import numpy as np
from numpy import pi, cos, sin, array, dot, eye, round, zeros, linalg, hstack
from math import sqrt, asin, acos, atan2
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

# Função que gera a matriz de transformação DH especialmente para a extremidade do segundo elo
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
        # Composição de todas as matrizes de transformação homogêneas de cada elo
        T = dot(T, A_i)
    position = T[:3, 3]  # Posição (x, y, z)
    R = T[:3, :3]  # Matriz de rotação que indica as novas orientações dos eixos do efetuador final em relação aos eixos originais
    return position, R

# Função para calcular a Jacobiana numericamente
def jacobian(q, delta=1e-6):
    J = zeros((6, len(q)))  # 3 para posição, 3 para orientação
    current_pos, current_rot = forward_kinematics(q)
    
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += delta
        new_pos, new_rot = forward_kinematics(dq)
        # Diferença na posição calculada para um dq na variação dos ângulos
        J[:3, i] = (new_pos - current_pos)/delta
        # Diferença na orientação calculada para um dq na variação dos ângulos
        rot_diff = dot(new_rot, current_rot.T)  # Matriz de rotação relativa
        J[3:, i] = array([rot_diff[2, 1], rot_diff[0, 2], rot_diff[1, 0]]) / delta  # Extrai ângulos de Euler aproximados a partir de row, pitch e yaw
    return J

# Função que verifica os limites das juntas
def check_joint_limits(q):
    assert len(q) == len(joint_limits), "O número de juntas não corresponde ao número de limites."
    # Mudar depois para algo mais cômodo, isso é improviso
    for i, angle in enumerate(q):
        lower_limit, upper_limit = joint_limits[i]
        if angle < lower_limit:
            q[i] = lower_limit
        elif angle > upper_limit:
            q[i] = upper_limit
    return q

# Necessidade para acertar corretamente os parâmetros radiais
def first_steps(target_position):
    # Função para pegar a orientação radial do alvo em relação ao eixo Z
    orientation = (180/pi)*atan2(target_position[1], target_position[0])
    if orientation > 180:
        orientation -= 360
    elif orientation < -180:
        orientation += 360
    height = target_position[2]

    return orientation, height

# Função de cinemática inversa
def inverse_kinematics(target_position, initial_angles, max_iterations=10000, tolerance=1e-6, orientation_tolerance=1e-4):
    q = initial_angles.copy()
    orientation, height = first_steps(target_position)
    # Fixando a rotação da junta 1 para a orientação do alvo, o problema é simplificado
    q[0] = (pi/180)*orientation
    # Fixando a rotação da junta 6 para 45° para a garra se alinhar corretamente
    q[5] = pi/4
    for iteration in range(max_iterations):
        current_position, current_rot = forward_kinematics(q)

        # Aqui deve ser avaliado, mantendo-se a orientação, se os valores de y e de z estão sendo alcançados
        # Além disso, é necessário excluir todas as componentes em X e em Y do eixo Z' do efetuador final
        height_and_y_error = [(target_position[1] - current_position[1]), (height - current_position[2])]

        rot_error_y = current_rot[1, 2]  # Pega a componente Y da terceira coluna da matriz de rotação
        rot_error_x = current_rot[0, 2]  # Pega a componente X da terceira coluna da matriz de rotação
        
        if linalg.norm(height_and_y_error) > tolerance or abs(rot_error_x) > orientation_tolerance or abs(rot_error_y) > orientation_tolerance:
            error = hstack([zeros(1), height_and_y_error, [-rot_error_x, -rot_error_y, 0]])
        else:
            return q 
        # Atualizar as juntas
        J = jacobian(q)
        dq = dot(pinv(J), error)
        for i in range(6):
            if i == 0:
                # Junta 1 nunca atualizada
                q[i] = q[i]
            elif i == 3:
                # Junta 4 também não precisa ser atualizada para a movimentação em coordenadas cilíndricas, mesmo para não afetar a junta 5
                q[i] = q[i]
            elif i == 4:
                # A junta 5 preferencialmente deve ser movimentada de forma a somar 90° com as juntas 2 e 3, apontando para baixo
                q[i] = pi/2 -(q[1] + q[2]) 
            elif i == 5:
                # A rotação da junta 6 é indiferente nesse caso, apenas faz a garra rotacionar em torno do próprio eixo
                q[i] = q[i]
            else:
                q[i] += dq[i] 
        q = check_joint_limits(q)
    return q  # Retorna a última tentativa mesmo que não tenha convergido completamente

# Função de teste para a cinemática direta
def test_forward_kinematics(q):
    position, _ = forward_kinematics(q)
    return round(position, 6)

# Função para formatar a string do comando ROS2
def format_ros2_action_command(joint_angles, speed=1.0):
    command = f'ros2 action send_goal -f /MoveJ ros2_data/action/MoveJ ' \
              f'\"{{goal: {{joint1: {joint_angles[0]:.2f}, joint2: {joint_angles[1]:.2f}, ' \
              f'joint3: {joint_angles[2]:.2f}, joint4: {joint_angles[3]:.2f}, ' \
              f'joint5: {joint_angles[4]:.2f}, joint6: {joint_angles[5]:.2f}}}, ' \
              f'speed: {speed}}}\"'
    return command

# Função de teste que verifica soluções com perpendicularidade ao eixo Z
def get_to_target(target_position, initial_angles, orientation_tolerance=1e-4):
    solution = inverse_kinematics(target_position, initial_angles, orientation_tolerance=orientation_tolerance)
    _, R = forward_kinematics(solution)
    new_z = [R[0,2], R[1,2], R[2,2]]
    if solution is not None:
        pos = test_forward_kinematics(solution)
        print(f'Posição calculada (X, Y, Z) do efetuador final: {pos}')
        print(f'Orientação do eixo do efetuador final decomposta em (X, Y, Z): {new_z}')
        for i in range(6):
            solution[i] = round((180/pi)*solution[i], 6)
        # Imprime o comando formatado para ROS2
        print(f'Solução encontrada: {round(solution, 6)}')
        command = format_ros2_action_command(solution)
        print(f'Comando ROS2 para enviar esta solução:\n{command}')
    else:
        print("Solução não convergiu.")

testHeight = 0.21
testRange = 0.42

initial_angles = [0, 0, 0, 0, 0, 0]
target_position = array([testRange, testRange, testHeight])
get_to_target(target_position, initial_angles)
