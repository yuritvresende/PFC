import numpy as np
from scipy.linalg import pinv

# Definindo limites das juntas (upper limit - UL e lower limit - LL)
j1UL = 165; j1LL = -165
j2UL = 110; j2LL = -110
j3UL = 70; j3LL = -110
j4UL = 160; j4LL = -160
j5UL = 120; j5LL = -120
j6UL = 400; j6LL = -400

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
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Parâmetros DH do manipulador robótico
dh_params = [
    {'d': 0.29, 'a': 0, 'alpha': -np.pi/2},
    {'d': 0, 'a': 0.27, 'alpha': 0},
    {'d': 0, 'a': 0.07, 'alpha': -np.pi/2},
    {'d': 0.302, 'a': 0, 'alpha': np.pi/2},
    {'d': 0, 'a': 0, 'alpha': -np.pi/2},
    {'d': 0.072, 'a': 0, 'alpha': 0}
]

# Função de cinemática direta que calcula a posição do efetuador final
def forward_kinematics(q):
    T = np.eye(4)  # Matriz identidade 4x4
    for i, params in enumerate(dh_params):
        theta = q[i]
        # Calcula a matriz de transformação de cada junta e acumula em T
        A_i = dh_matrix(theta, params['d'], params['a'], params['alpha']) 
        T = np.dot(T, A_i)
    return T[:3, 3]  # Retorna apenas a posição (x, y, z) do efetuador final

# Função que calcula a matriz Jacobiana numericamente
def jacobian(q, delta=1e-6):
    J = np.zeros((3, len(q)))  # Inicializa a matriz Jacobiana
    current_pos = forward_kinematics(q)  # Posição atual do efetuador final
    for i in range(len(q)):
        dq = q.copy()
        dq[i] += delta  # Perturba o ângulo da junta i
        new_pos = forward_kinematics(dq)  # Calcula a nova posição
        # Calcula a derivada numérica (variação na posição / variação no ângulo)
        J[:, i] = (new_pos - current_pos) / delta
    return J

# Função que verifica os limites das juntas e ajusta os ângulos se necessário
def check_joint_limits(q):
    """Verifica se os ângulos estão dentro dos limites e ajusta, se necessário."""
    for i, angle in enumerate(q):
        lower_limit, upper_limit = joint_limits[i]  # Limites da junta i
        if angle < lower_limit:
            print(f"Ângulo da junta {i+1} ({angle}) abaixo do limite inferior ({lower_limit}). Ajustando...")
            q[i] = lower_limit  # Ajusta para o limite inferior
        elif angle > upper_limit:
            print(f"Ângulo da junta {i+1} ({angle}) acima do limite superior ({upper_limit}). Ajustando...")
            q[i] = upper_limit  # Ajusta para o limite superior
    return q

# Função de cinemática inversa que encontra os ângulos das juntas para alcançar uma posição alvo
def inverse_kinematics(target_pos, initial_q, max_iterations=100, tolerance=1e-6):
    q = initial_q.copy()  # Copia os ângulos iniciais
    for iteration in range(max_iterations):
        current_pos = forward_kinematics(q)  # Calcula a posição atual do efetuador final
        error = target_pos - current_pos  # Calcula o erro (diferença entre posição atual e alvo)
        
        # Verifica se o erro está dentro da tolerância
        if np.linalg.norm(error) < tolerance:
            print(f'Solução encontrada após {iteration+1} iterações.')
            return q
        
        # Calcula o Jacobiano e a variação nos ângulos
        J = jacobian(q)
        dq = np.dot(pinv(J), error)  # Ajusta os ângulos das juntas usando o Jacobiano pseudo-inverso 
        q += dq  # Atualiza os ângulos das juntas
        
        # Verifica se os ângulos estão dentro dos limites
        q = check_joint_limits(q)

    print('A solução não convergiu dentro dos limites.')
    return q  # Retorna os ângulos ajustados, mesmo sem convergência

# Função de teste para verificar a cinemática direta
def test_forward_kinematics(q):
    position = forward_kinematics(q)  # Calcula a posição do efetuador final
    position_rounded = np.round(position, 4)  # Arredonda para 4 casas decimais
    print(f'Posição calculada do efetuador final (X, Y, Z): {position_rounded}')
    return position_rounded

# Função de teste que executa a cinemática inversa e verifica a posição final
def test_inverse_and_forward(target_position, initial_angles):
    calculated_angles = inverse_kinematics(target_position, initial_angles)  # Calcula os ângulos
    calculated_angles_rounded = np.round(calculated_angles, 4)  # Arredonda os ângulos
    print(f'Ângulos calculados pela cinemática inversa: {calculated_angles_rounded}')
    
    final_position = test_forward_kinematics(calculated_angles)  # Calcula a posição final
    error = np.round(target_position - final_position, 4)  # Calcula o erro final
    print(f'Diferença entre a posição desejada e a posição calculada: {error}')

# Posição alvo e ângulos iniciais
target_position = np.array([0.3, 0.2, 0.5])
initial_angles = [0, 0, 0, 0, 0, 0]

# Executa o teste de cinemática inversa e direta
test_inverse_and_forward(target_position, initial_angles)
