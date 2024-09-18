import numpy as np
from scipy.linalg import pinv

# Função para calcular a matriz de transformação homogênea usando os parâmetros de Denavit-Hartenberg
def dh_matrix(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Parâmetros DH para o braço robótico (valores constantes, exceto theta)
dh_params = [
    {'d': 0.29, 'a': 0, 'alpha': -np.pi/2},
    {'d': 0, 'a': 0.27, 'alpha': 0},
    {'d': 0, 'a': 0.07, 'alpha': -np.pi/2},
    {'d': 0.302, 'a': 0, 'alpha': np.pi/2},
    {'d': 0, 'a': 0, 'alpha': -np.pi/2},
    {'d': 0.072, 'a': 0, 'alpha': 0}
]

# Função para calcular a posição do efetuador final com base nos ângulos das juntas
def forward_kinematics(q):
    T = np.eye(4)  # Matriz identidade 4x4
    for i, params in enumerate(dh_params):
        theta = q[i]  # Ângulo da junta atual
        A_i = dh_matrix(theta, params['d'], params['a'], params['alpha'])  # Matriz de transformação
        T = np.dot(T, A_i)  # Multiplica as matrizes de transformação
    return T[:3, 3]  # Retorna a posição final do efetuador (X, Y, Z)

# Função para calcular o Jacobiano
def jacobian(q, delta=1e-6):
    J = np.zeros((3, len(q)))  # Inicializa a matriz Jacobiana com zeros
    current_pos = forward_kinematics(q)  # Posição atual do efetuador
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += delta  # Pequena variação no ângulo da junta i
        new_pos = forward_kinematics(dq)
        J[:, i] = (new_pos - current_pos) / delta  # Diferenciação numérica
    return J

# Função para resolver a cinemática inversa
def inverse_kinematics(target_pos, initial_q, max_iterations=100, tolerance=1e-6):
    q = initial_q.copy()
    for iteration in range(max_iterations):
        current_pos = forward_kinematics(q)
        error = target_pos - current_pos  # Diferença entre a posição atual e a posição desejada
        if np.linalg.norm(error) < tolerance:  # Se o erro for menor que a tolerância
            print(f'Solução encontrada após {iteration+1} iterações.')
            return q
        J = jacobian(q)
        dq = np.dot(pinv(J), error)  # Ajusta os ângulos das juntas usando o Jacobiano pseudo-inverso
        q += dq  # Atualiza os ângulos das juntas
    print('A solução não convergiu.')
    return q

# Função de teste para calcular a posição XYZ dada uma lista de ângulos das juntas
def test_forward_kinematics(q):
    position = forward_kinematics(q)
    position_rounded = np.round(position, 4)  # Arredonda a posição para 4 casas decimais
    print(f'Posição calculada do efetuador final (X, Y, Z): {position_rounded}')
    return position_rounded

# Função que testa a cinemática inversa e depois verifica a cinemática direta
def test_inverse_and_forward(target_position, initial_angles):
    # Passo 1: Calcula os ângulos das juntas que alcançam a posição alvo
    calculated_angles = inverse_kinematics(target_position, initial_angles)
    calculated_angles_rounded = np.round(calculated_angles, 4)  # Arredonda os ângulos para 4 casas decimais
    print(f'Ângulos calculados pela cinemática inversa: {calculated_angles_rounded}')
    
    # Passo 2: Usa os ângulos encontrados para calcular a posição do efetuador final (cinemática direta)
    final_position = test_forward_kinematics(calculated_angles)
    
    # Passo 3: Exibe a diferença entre a posição desejada e a posição alcançada
    error = np.round(target_position - final_position, 4)  # Arredonda o erro para 4 casas decimais
    print(f'Diferença entre a posição desejada e a posição calculada: {error}')

# Exemplo de uso da função que testa a cinemática inversa e depois a direta
target_position = np.array([0.3, 0.2, 0.5])  # Posição alvo (X, Y, Z)
initial_angles = [0, -np.pi/4, 0, 0, 0, 0]  # Estimativa inicial dos ângulos das juntas
test_inverse_and_forward(target_position, initial_angles)
