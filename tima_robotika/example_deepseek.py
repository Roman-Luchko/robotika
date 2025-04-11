import numpy as np
from sklearn.cluster import KMeans
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt

class UAVDataCollection:
    def __init__(self, area_size=2000, num_sensors=100, uav_coverage=200, uav_altitude=100):
        """
        Инициализация системы сбора данных с помощью БПЛА
        
        Параметры:
        - area_size: Размер зоны наблюдения в метрах (квадрат)
        - num_sensors: Количество наземных сенсорных устройств (SD)
        - uav_coverage: Радиус покрытия БПЛА в позиции зависания
        - uav_altitude: Высота работы БПЛА
        """
        self.area_size = area_size
        self.num_sensors = num_sensors
        self.uav_coverage = uav_coverage
        self.uav_altitude = uav_altitude
        
        # Генерация случайных позиций сенсоров
        self.sensor_positions = np.random.rand(num_sensors, 2) * area_size
        self.dcc_position = np.array([area_size/2, area_size/2])  # Центр сбора данных (DCC) в центре
        
        # Параметры из статьи
        self.uav_speed = 8  # м/с
        self.data_per_sensor = 0.5  # Гбит
        self.battery_capacity = 5000  # мАч
        
        # Параметры модели энергопотребления
        self.p_hover = 200  # Мощность при зависании (Вт)
        self.p_fly = 300    # Мощность при полете (Вт)
        
        # Параметры связи
        self.bandwidth = 10e6  # 10 МГц
        self.noise_power = 1e-9  # 1 нВт
        self.tx_power = 100  # 100 мВт
        
    def calculate_snr(self, distance):
        """
        Расчет SNR между БПЛА и наземным сенсором
        На основе уравнений (3)-(8) из статьи
        """
        # Упрощенная модель потерь на трассе
        fc = 2.4e9  # Частота несущей (2.4 ГГц)
        c = 3e8     # Скорость света
        path_loss_exp = 2.2  # Показатель потерь на трассе
        
        # Потери в свободном пространстве
        fspl = (4 * np.pi * fc * distance / c)**2
        
        # Дополнительные факторы потерь (упрощенно)
        los_prob = 1 / (1 + 10 * np.exp(-0.6 * (np.arcsin(self.uav_altitude/distance) * 180/np.pi - 10)))
        nlos_prob = 1 - los_prob
        
        # Средние потери на трассе
        apl = fspl * (los_prob * 1 + nlos_prob * 3)  # NLoS имеет в 3 раза большие потери
        
        snr = (self.tx_power * 1e-3) / (apl * self.noise_power)
        return snr
    
    def calculate_transmission_rate(self, distance, num_sensors_in_coverage):
        """
        Расчет скорости передачи данных от сенсора к БПЛА
        На основе уравнений (1)-(2) из статьи
        """
        snr = self.calculate_snr(distance)
        bandwidth_per_sensor = self.bandwidth / num_sensors_in_coverage
        rate = bandwidth_per_sensor * np.log2(1 + snr)
        return rate
    
    def calculate_transmission_time(self, distance, num_sensors_in_coverage):
        """
        Расчет времени передачи данных для сенсора
        На основе уравнения (1) из статьи
        """
        rate = self.calculate_transmission_rate(distance, num_sensors_in_coverage)
        return (self.data_per_sensor * 1e9) / rate  # Конвертация Гбит в биты
    
    def find_hover_positions_cboc(self):
        """
        Алгоритм центроидного покрытия с перекрытием (CBOC) для поиска позиций зависания БПЛА
        На основе Алгоритма 1 из статьи
        """
        uncovered_sensors = self.sensor_positions.copy()
        hover_positions = []
        covered_sensors = []
        
        while len(uncovered_sensors) > 0:
            best_position = None
            best_coverage = 0
            best_covered_indices = []
            
            # Пробуем каждый непокрытый сенсор как потенциальный центр
            for i, sensor in enumerate(uncovered_sensors):
                # Расчет расстояния до всех других сенсоров
                distances = np.linalg.norm(uncovered_sensors - sensor, axis=1)
                in_coverage = distances <= self.uav_coverage
                
                # Подсчет количества сенсоров в зоне покрытия
                coverage_count = np.sum(in_coverage)
                
                if coverage_count > best_coverage:
                    best_coverage = coverage_count
                    best_covered_indices = np.where(in_coverage)[0]
                    best_position = sensor
            
            if best_position is None:
                break  # Невозможно обеспечить большее покрытие
            
            # Расчет центроида покрытых сенсоров как новой позиции зависания
            if len(best_covered_indices) > 0:
                centroid = np.mean(uncovered_sensors[best_covered_indices], axis=0)
                hover_positions.append(centroid)
                
                # Удаление покрытых сенсоров
                uncovered_sensors = np.delete(uncovered_sensors, best_covered_indices, axis=0)
                
                # Запись, какие сенсоры покрыты этой позицией зависания
                covered_sensors.append(best_covered_indices)
        
        self.hover_positions = np.array(hover_positions)
        self.covered_sensors = covered_sensors
        return self.hover_positions
    
    def cluster_data_collection_areas(self, num_uavs):
        """
        Алгоритм кластеризации зон сбора данных
        На основе Алгоритма 2 из статьи
        """
        # Использование K-средних для распределения позиций зависания между БПЛА
        kmeans = KMeans(n_clusters=num_uavs, random_state=0).fit(self.hover_positions)
        self.uav_clusters = kmeans.labels_
        self.cluster_centers = kmeans.cluster_centers_
        
        # Распределение сенсоров по кластерам на основе того, какая позиция зависания их покрывает
        self.sensor_clusters = -1 * np.ones(self.num_sensors, dtype=int)
        for cluster_idx in range(num_uavs):
            hover_in_cluster = np.where(self.uav_clusters == cluster_idx)[0]
            for hover_idx in hover_in_cluster:
                for sensor_idx in self.covered_sensors[hover_idx]:
                    self.sensor_clusters[sensor_idx] = cluster_idx
        
        return self.uav_clusters, self.cluster_centers
    
    def plan_flight_paths(self):
        """
        Алгоритм предотвращения обратного хода при планировании маршрута
        На основе Алгоритма 3 из статьи
        """
        num_uavs = len(np.unique(self.uav_clusters))
        self.flight_paths = []
        self.flight_distances = np.zeros(num_uavs)
        
        for uav_idx in range(num_uavs):
            # Получение позиций зависания, назначенных этому БПЛА
            hover_indices = np.where(self.uav_clusters == uav_idx)[0]
            hover_positions = self.hover_positions[hover_indices]
            
            if len(hover_positions) == 0:
                self.flight_paths.append([])
                continue
            
            # Начало от DCC
            current_pos = self.dcc_position
            path = [current_pos.copy()]
            remaining_positions = hover_positions.copy()
            total_distance = 0
            
            while len(remaining_positions) > 0:
                # Нахождение ближайшей позиции зависания
                distances = np.linalg.norm(remaining_positions - current_pos, axis=1)
                closest_idx = np.argmin(distances)
                closest_pos = remaining_positions[closest_idx]
                
                # Добавление в маршрут и обновление расстояния
                path.append(closest_pos.copy())
                total_distance += distances[closest_idx]
                
                # Переход к следующей позиции
                current_pos = closest_pos
                remaining_positions = np.delete(remaining_positions, closest_idx, axis=0)
            
            # Возврат к DCC
            path.append(self.dcc_position.copy())
            total_distance += np.linalg.norm(current_pos - self.dcc_position)
            
            self.flight_paths.append(np.array(path))
            self.flight_distances[uav_idx] = total_distance
        
        return self.flight_paths
    
    def calculate_energy_consumption(self):
        """
        Расчет энергопотребления для всех БПЛА
        На основе уравнений (11)-(14) из статьи
        """
        num_uavs = len(np.unique(self.uav_clusters))
        self.hover_times = np.zeros(num_uavs)
        self.energy_consumption = np.zeros(num_uavs)
        
        for uav_idx in range(num_uavs):
            hover_indices = np.where(self.uav_clusters == uav_idx)[0]
            
            # Расчет времени зависания для каждой позиции
            for hover_idx in hover_indices:
                # Получение сенсоров, покрываемых этой позицией зависания
                sensor_indices = self.covered_sensors[hover_idx]
                
                if len(sensor_indices) == 0:
                    continue
                
                # Расчет расстояния до каждого сенсора
                hover_pos = self.hover_positions[hover_idx]
                sensor_positions = self.sensor_positions[sensor_indices]
                distances = np.sqrt(
                    (sensor_positions[:, 0] - hover_pos[0])**2 + 
                    (sensor_positions[:, 1] - hover_pos[1])**2 + 
                    self.uav_altitude**2
                )
                
                # Время передачи - максимум из всех сенсоров в зоне покрытия
                transmission_times = [
                    self.calculate_transmission_time(d, len(sensor_indices)) 
                    for d in distances
                ]
                hover_time = np.max(transmission_times)
                self.hover_times[uav_idx] += hover_time
            
            # Расчет энергопотребления
            hover_energy = self.p_hover * self.hover_times[uav_idx] / 3600  # Конвертация в Вт·ч
            flight_energy = self.p_fly * (self.flight_distances[uav_idx] / self.uav_speed) / 3600
            self.energy_consumption[uav_idx] = hover_energy + flight_energy
        
        return self.energy_consumption
    
    def run_full_simulation(self, num_uavs=3):
        """Запуск полной симуляции"""
        # Шаг 1: Поиск оптимальных позиций зависания
        self.find_hover_positions_cboc()
        
        # Шаг 2: Кластеризация зон сбора данных
        self.cluster_data_collection_areas(num_uavs)
        
        # Шаг 3: Планирование маршрутов полета
        self.plan_flight_paths()
        
        # Шаг 4: Расчет энергопотребления
        self.calculate_energy_consumption()
        
        # Расчет общего времени выполнения задачи
        flight_times = self.flight_distances / self.uav_speed
        self.task_completion_time = np.max(self.hover_times + flight_times)
        
        return {
            'num_hover_positions': len(self.hover_positions),
            'task_completion_time': self.task_completion_time,
            'total_energy': np.sum(self.energy_consumption),
            'flight_distances': self.flight_distances
        }
    
    def visualize(self):
        """Визуализация позиций сенсоров, позиций зависания и маршрутов полета"""
        plt.figure(figsize=(10, 10))
        
        # Отображение сенсоров
        plt.scatter(self.sensor_positions[:, 0], self.sensor_positions[:, 1], 
                   c='blue', label='Сенсоры', alpha=0.6)
        
        # Отображение DCC
        plt.scatter(self.dcc_position[0], self.dcc_position[1], 
                   c='red', marker='s', s=100, label='Центр сбора данных (DCC)')
        
        if hasattr(self, 'hover_positions'):
            # Отображение позиций зависания и зон покрытия
            colors = plt.cm.get_cmap('tab10', len(np.unique(self.uav_clusters)))
            
            for i, pos in enumerate(self.hover_positions):
                cluster = self.uav_clusters[i]
                color = colors(cluster)
                
                # Позиция зависания
                plt.scatter(pos[0], pos[1], c=[color], marker='D')
                
                # Зона покрытия
                circle = plt.Circle((pos[0], pos[1]), self.uav_coverage, 
                                  color=color, alpha=0.1)
                plt.gca().add_patch(circle)
            
            # Отображение маршрутов полета
            if hasattr(self, 'flight_paths'):
                for i, path in enumerate(self.flight_paths):
                    if len(path) > 0:
                        color = colors(i)
                        plt.plot(path[:, 0], path[:, 1], c=color, linestyle='-', 
                                marker='o', markersize=4, label=f'БПЛА {i+1}')
        
        plt.xlim(0, self.area_size)
        plt.ylim(0, self.area_size)
        plt.xlabel('Координата X (м)')
        plt.ylabel('Координата Y (м)')
        plt.title('Сбор данных с помощью БПЛА в беспроводной сенсорной сети')
        plt.legend()
        plt.grid(True)
        plt.show()

# Пример использования
if __name__ == "__main__":
    # Создание среды симуляции
    sim = UAVDataCollection(area_size=2000, num_sensors=100)
    
    # Запуск полной симуляции с 3 БПЛА
    results = sim.run_full_simulation(num_uavs=3)
    
    # Вывод результатов
    print("Результаты симуляции:")
    print(f"Количество позиций зависания: {results['num_hover_positions']}")
    print(f"Время выполнения задачи: {results['task_completion_time']:.2f} секунд")
    print(f"Общее энергопотребление: {results['total_energy']:.2f} Вт·ч")
    print(f"Дистанции полета: {results['flight_distances']} метров")
    
    # Визуализация результатов
    sim.visualize()
