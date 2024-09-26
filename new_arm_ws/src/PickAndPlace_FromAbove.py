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

# Essa função foi adicionada para poder verificar o quanto é necessário girar a junta 6 para a garra apanhar a caixa por sua menor dimensão
def gripper_orientation(box_size):
    if box_size[0] > box_size[1]:
        return -180
    else:
        return -90

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

# Definição do comando para spawnar uma caixa a partir da posição definida
def print_box(target_position):
    command = f'ros2 run ros2_grasping spawn_object.py --package "ros2_grasping" --urdf "box.urdf" --name "box" ' \
              f'--x {target_position[0]:.2f} --y {target_position[1]:.2f} --z {target_position[2]:.2f}'
    return command

# Função de teste que verifica soluções com perpendicularidade ao eixo plano XY
def get_to_target(target_position, initial_angles, orientation_tolerance=1e-4):
    solution = inverse_kinematics(target_position, initial_angles, orientation_tolerance=orientation_tolerance)
    _, R = forward_kinematics(solution)
    new_z = [R[0,2], R[1,2], R[2,2]]
    if solution is not None:
        pos = test_forward_kinematics(solution)
        solve = solution
        for i in range(6):
            solve[i] = round((180/pi)*solution[i], 6)
    else:
        print("Solução não convergiu.")
    return solution

# A sequência abaixo executa repetidas vezes as funções de movimento com a finalidade de lançar no terminal todos os comandos necessários
def pick_and_place(initial_angles, box_size, target_position, final_position):
    # Primeiramente, é necessário spawnar a caixa
    command1 = print_box(target_position)
    print(f'Comando ROS2 para spawnar a caixa:\n{command1}')
    
    # Após isso, a coordenada inicial de movimento leva em consideração colocar a garra suficientemente acima da caixa
    initial_movement = target_position + gripper_length + [0, 0, 2*box_size[2]]
    new_init = get_to_target(initial_movement, initial_angles, orientation_tolerance=1e-4)
    print(f'\nSolução encontrada para pairar sobre a caixa: {round(new_init, 6)}')
    command2 = format_ros2_action_command(new_init)
    print(f'Comando para pairar sobre a caixa:\n{command2}')
   
    # A partir da função definida lá em cima, aqui se faz o ajuste na rotação da garra após identificar o menor lado da caixa
    correct_j6 = gripper_orientation(box_size)
    print(f'\nA garra deve se orientar para {correct_j6}° e está em {new_init[0]}° \n')
    correction = new_init[0] - correct_j6 - 180
    print(f'A garra deve girar um total de {correction}°\n')
    new_init[5] += correction    

    # Aqui o comando para corrigir a orientação sem mover as outras juntas do braço
    print(f'Solução encontrada para corrigir a orientação da garra: {round(new_init, 6)}') 
    command3 = format_ros2_action_command(new_init)
    print(f'Comando ROS2 para corrigir a orientação da garra:\n{command3}')
    
    # Com o ajuste feito, agora é necessário encaixar a garra no corpo da caixa com um movimento quase linear, por isso se varia apenas a altura do alvo
    pick_position = get_to_target(initial_movement - [0, 0, 1.5*box_size[2]], new_init, orientation_tolerance=1e-4)
    pick_position[5] = new_init[5]
    print(f'\nSolução encontrada para encaixar a caixa: {round(new_init, 6)}') 
    command4 = format_ros2_action_command(pick_position)
    print(f'Comando ROS2 para encaixar a caixa:\n{command4}')
    
    # A verificar por enquanto, mas esse comando mantém a caixa e a ponta da garra conectados em um terminal que ficará em atividade constante
    print('\nComando ROS2 para fechar a garra:')
    print('ros2 action send_goal -f /Attacher ros2_grasping/action/Attacher "{object: \'box\', endeffector: \'EE_egp64\'}"')
    
    # Soma de todas as etapas até aqui para facilitar a instrução no terminal
    print(f'\nComando pick completo:') 
    print(f'{command1} && {command2} && {command3} && {command4} && ros2 action send_goal -f /Attacher ros2_grasping/action/Attacher "{{object: \'box\', endeffector: \'EE_egp64\'}}"')
    
    # Para levantar o objeto, retorna-se uma posição
    print('\nPara afastar, utiliza-se o movimento inverso ao de se aproximar, mas em outro terminal') 
    print(f'Comando ROS2 para se afastar carregando o bloco:\n{command3}')
    
    # Para evitar a possibilidade de esbarrar a caixa no chão, retorna-se primeiro à posição inicial como passo intermediário
    inter_command = format_ros2_action_command([0, 0, 0, 0, 90, 45])
    print(f'\nComando ROS2 para chegar a uma posição neutra:\n{inter_command}')

    # Agora, semelhantemente ao caso do pick, o alvo da garra será estar um pouco acima da posição em que a caixa deve ficar
    first_goal = final_position + [0, 0, 2*box_size[2]] + gripper_length
    place_position = get_to_target(first_goal, new_init, orientation_tolerance=1e-4)
    correction = place_position[0] - correct_j6 - 180
    place_position[5] += correction
    print(f'\nSolução encontrada para aproximar o alvo do local de colocar: {round(place_position, 6)}') 
    command5 = format_ros2_action_command(place_position)
    print(f'Comando ROS2 para iniciar a descida no local alvo:\n{command5}')

    # Essa será, portanto, a posição para deixar a caixa no chão
    leave_position = get_to_target(first_goal - [0, 0, 1.5*box_size[2]], place_position, orientation_tolerance=1e-4)
    leave_position[5] = place_position[5]
    print(f'\nSolução encontrada para deixar a caixa: {round(leave_position, 6)}') 
    command6 = format_ros2_action_command(leave_position)
    print(f'Comando ROS2 para deixar a caixa:\n{command6}')

    # Soma de todas as etapas até aqui
    print(f'\nComando place completo:') 
    print(f'{command3} && {inter_command} && {command5} && {command6}')

    # Após posicionar a caixa em seu local de destino, deve-se retornar ao terminal que estava ativo mantendo a conexão e encerrar sua atividade para continuar
    print('\nPara afastar, utiliza-se o movimento inverso ao de se aproximar') 
    print(f'Comando ROS2 para se afastar depois de soltar o bloco:\n{command5}')

    print('\nNesse meio termo volte ao outro terminal e encerre a atividade da garra')
   
    print(f'\nComando ROS2 para chegar a uma posição neutra:\n{inter_command}')
   
    print(f'\nComando de encerramento completo:') 
    print(f'{command5} && {inter_command}')

initial_angles = [0, 0, 0, 0, 0, 0]
box_size = [0.05, 0.03, 0.08]
gripper_length = [0, 0, 0.171971]
target_position = array([0.2, -0.3, box_size[2]/2])
final_position = array([0.2, 0.3, box_size[2]/2])
pick_and_place(initial_angles, box_size, target_position, final_position)

