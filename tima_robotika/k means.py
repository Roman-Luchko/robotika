import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans
from matplotlib.colors import ListedColormap
import itertools

# 1. Генерация 30 случайных точек
np.random.seed(42)
X = np.random.uniform(0, 12, size=(13, 2))  # x в интервале [0, 12], y в интервале [0, 19]

# 2. Кластеризация на 4 кластера
n_clusters = 3
kmeans = KMeans(n_clusters=n_clusters, random_state=0)
kmeans.fit(X)
labels = kmeans.predict(X)
centroids = kmeans.cluster_centers_

# Центр карты
center = np.array([6, 9.5])  # Центр карты, координаты (6, 9.5)

# 3. Сетка для визуализации кластеров
h = 0.05
x_min, x_max = X[, 0].min() - 1, X[, 0].max() + 1
y_min, y_max = X[, 1].min() - 1, X[, 1].max() + 1
xx, yy = np.meshgrid(np.arange(x_min, x_max, h),
                     np.arange(y_min, y_max, h))
Z = kmeans.predict(np.c_[xx.ravel(), yy.ravel()])
Z = Z.reshape(xx.shape)

# 4. Матрица расстояний между центрами кластеров и центром карты
def euclidean(p1, p2)
    return np.linalg.norm(p1 - p2)

N = len(centroids)
distance_matrix = np.zeros((N + 1, N + 1))  # добавим 1 для центра карты
for i in range(N)
    for j in range(N)
        if i != j
            distance_matrix[i][j] = euclidean(centroids[i], centroids[j])

# Добавляем расстояния от центра карты до всех центров кластеров
for i in range(N)
    distance_matrix[N][i] = euclidean(center, centroids[i])
    distance_matrix[i][N] = distance_matrix[N][i]

# 5. Решение TSP с центром карты
cities = list(range(N))  # Убираем центр карты из обычных точек
cities = [N] + cities + [N]  # Добавляем центр в начале и в конце

best_path = None
best_cost = float('inf')

# Перебираем все возможные маршруты
for perm in itertools.permutations(cities[1N+1])  # фиксируем центр
    path = [N] + list(perm) + [N]  # центр в начале и в конце
    cost = sum(distance_matrix[path[i]][path[i+1]] for i in range(len(path) - 1))
    if cost  best_cost
        best_cost = cost
        best_path = path

# 6. Визуализация
plt.figure(figsize=(10, 8))

# Визуализация кластеров
plt.subplot(211)  # Делаем первый график на верхней половине
h = 0.05
x_min, x_max = X[, 0].min() - 1, X[, 0].max() + 1
y_min, y_max = X[, 1].min() - 1, X[, 1].max() + 1
xx, yy = np.meshgrid(np.arange(x_min, x_max, h),
                     np.arange(y_min, y_max, h))
Z = kmeans.predict(np.c_[xx.ravel(), yy.ravel()])
Z = Z.reshape(xx.shape)

# Задний фон с цветами для кластеров
cmap_background = ListedColormap(['#FFCCCC', '#CCFFCC', '#CCCCFF', '#FFFFCC', '#FFCCFF'])
plt.contourf(xx, yy, Z, cmap=cmap_background, alpha=0.3)

# Точки кластеров
plt.scatter(X[, 0], X[, 1], c=labels, cmap=cmap_background, s=120, edgecolors='k')

# Центры кластеров
plt.scatter(centroids[, 0], centroids[, 1], c='black', s=250, marker='X', label='Центры кластеров')

# Центр карты
plt.scatter(center[0], center[1], c='red', s=200, marker='X', label='Центр карты')

# Названия для точек
for i, (x, y) in enumerate(X)
    plt.text(x + 0.3, y, f'B{i+1}', fontsize=10)

plt.title(Кластеризация 13 случайных точек с центром карты)
plt.xlabel(X координата)
plt.ylabel(Y координата)
plt.legend()
plt.grid(True)

# Добавляем отступ
plt.tight_layout()

# Показываем первый график
plt.show()

# 7. Визуализация маршрута с направленными стрелками (по линиям маршрута)
plt.figure(figsize=(10, 8))

# Рисуем сам маршрут между точками, добавляя стрелки
for i in range(len(best_path) - 1)
    p1 = centroids[best_path[i]] if best_path[i] != N else center
    p2 = centroids[best_path[i + 1]] if best_path[i + 1] != N else center

    # Используем plt.arrow() для отображения стрелки на линии между точками
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]

    # Уменьшаем размеры стрелок, можно регулировать коэффициент 0.1
    plt.arrow(p1[0], p1[1], dx  0.5, dy  0.5, head_width=0.3, head_length=0.5, fc='blue', ec='blue', length_includes_head=True)

# Рисуем центры кластеров
for i, (x, y) in enumerate(centroids)
    plt.scatter(x, y, s=200, color='orange', edgecolors='k', zorder=3)
    plt.text(x + 0.3, y, f'C{i+1}', fontsize=12)

# Рисуем центр карты
plt.scatter(center[0], center[1], s=200, color='red', edgecolors='k', zorder=3)
plt.text(center[0] + 0.3, center[1], 'Центр', fontsize=12)

plt.title(Оптимальный маршрут с указанием направления (стрелки по маршруту))
plt.xlabel(X координата)
plt.ylabel(Y координата)
plt.grid(True)

# Отступ между графиками
plt.tight_layout()

# Показываем второй график
plt.show()
