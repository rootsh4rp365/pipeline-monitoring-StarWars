# НТО 2026 по Летающей робототехнике - 2 командный этап - Команда StarWars

> Нахождение координат врезок нефтепровода в системе aruco_map

[![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen.svg)]()
[![Python](https://img.shields.io/badge/python-3-blue.svg)]()
[![Opencv](https://img.shields.io/badge/python-opencv-darkblue.svg)]()
[![Flask](https://img.shields.io/badge/python-Flask-lightblue.svg)]()

---

## Системные требования

- Образ ВМ Clover
- git
- Python
- Flask

---

## Установка

### Шаг 1: Клонирование репозитория
```
cd ~/catkin_ws/src

git clone https://github.com/rootsh4rp365/pipeline-monitoring-StarWars.git
```

### Шаг 2: изначальная настройка
```
cd ~/catkin_ws/src/pipeline-monitoring-StarWars/src

./setup.sh
```

### Шаг 3: Установка зависимостей
```
cd ~/catkin_ws/src/pipeline-monitoring-StarWars

pip3 install -r requirements.txt
```

---

## Быстрый запуск

### 1-е Окно терминала: запуск симулятора
```
roslaunch pipeline_monitoring_starwars simulator.launch
```

### 2-е Окно терминала: запуск скрипта навигации
```
roslaunch pipeline_monitoring_starwars scanner.launch
```

### 3-е Окно терминала: запуск веб-приложения
```
rosrun pipeline_monitoring_starwars web.py
```

### Открыть веб-интерфейс
В браузере перейти по адресу 
- http://localhost:5555


## Использование доп. инструментов / Настройка

### 1. Генерация мира
```
rosrun pipeline_monitoring_starwars generate_world.py
```

### 2. Изменение параметров миссии
#### Отредактировать ~/catkin_ws/src/pipeline-monitoring-StarWars/launch/scanner.launch:
#### Параметры:
- "flight_height" - высота полета (м)
- "scan_speed" - скорость полета (м/с)
- "navigation_threshold" - трешхолд при навигации по координатам (м)
        
- "grid_width" - высота сетки aruco маркеров
- "grid_height" - ширина сетки aruco маркеров
- "grid_spacing" - расстояние между центрами aruco маркеров (м)
        
- "detection_threshold" - трешхолд площади врезок
- "uniqueness_radius" - минимальное расстояние между центрами врезок

### Видео прохождения миссии
[![Mission video](https://img.youtube.com/vi/95Rbv68nJGY/maxresdefault.jpg)](https://youtube.com/watch?v=95Rbv68nJGY)