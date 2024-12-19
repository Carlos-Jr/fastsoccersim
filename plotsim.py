import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Função para carregar o CSV e processar os dados
def plot_movements(file_path):
    # Carrega o CSV
    data = pd.read_csv(file_path)
    robot_data = data[data['object'] == 'robot']
    ball_data = data[data['object'] == 'ball']
    
    plt.figure(figsize=(10, 8))
    plt.scatter(robot_data['x'], robot_data['y'], label='Robot Path', c=np.arange(len(robot_data['x'])), cmap='winter')
    plt.plot(ball_data['x'], ball_data['y'], label='Ball Path', color='orange')
    
    plt.title('Robot and Ball Movements')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    plt.show()

file_path = 'log.csv'
plot_movements(file_path)
