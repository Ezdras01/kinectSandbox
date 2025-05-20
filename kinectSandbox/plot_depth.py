import matplotlib.pyplot as plt
import csv
import numpy as np

def plot_depth_matrix_with_markers(filename="depth_matrix.csv"):
    depth_matrix = []
    try:
        with open(filename, 'r') as csvfile:
            reader = csv.reader(csvfile)
            for row in reader:
                try:
                    depth_matrix.append([int(val) for val in row])
                except ValueError:
                    print(f"Advertencia: Se ignoró una línea no numérica: {row}")
    except FileNotFoundError:
        print(f"Error: No se encontró el archivo '{filename}'. Asegúrate de que el archivo exista.")
        return

    if depth_matrix:
        depth_array = np.array(depth_matrix)

        # Encontrar las coordenadas del valor máximo
        max_val = np.max(depth_array)
        max_coords = np.where(depth_array == max_val)
        max_y, max_x = max_coords[0][0], max_coords[1][0]

        # Encontrar las coordenadas del valor mínimo
        min_val = np.min(depth_array)
        min_coords = np.where(depth_array == min_val)
        min_y, min_x = min_coords[0][0], min_coords[1][0]

        plt.figure(figsize=(10, 8))
        plt.imshow(depth_array, cmap='jet_r')
        plt.colorbar(label='Profundidad (mm)')
        plt.title('Mapa de Profundidad con Máximo y Mínimo Marcados')
        plt.xlabel('Ancho (píxeles)')
        plt.ylabel('Alto (píxeles)')
        # plt.gca().invert_yaxis()

        # Marcar el valor máximo con un círculo rojo
        plt.plot(max_x, max_y, 'ro', markersize=8, label=f'Máximo: {max_val} mm')

        # Marcar el valor mínimo con un círculo azul
        plt.plot(min_x, min_y, 'bo', markersize=8, label=f'Mínimo: {min_val} mm')

        plt.legend()
        plt.tight_layout()
        plt.show()
    else:
        print("No se encontraron datos de profundidad para graficar.")

if __name__ == "__main__":
    plot_depth_matrix_with_markers()