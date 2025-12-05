#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
CATKIN_WS="${HOME}/catkin_ws"

echo -e "Проверка окружения..."

if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/noetic/setup.bash
fi

if [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
    source "${CATKIN_WS}/devel/setup.bash"
fi

echo -e "ROS: $ROS_DISTRO"
echo -e "Проект: $PROJECT_DIR"
echo ""

echo -e "Проверка структуры проекта..."
if [ ! -f "${PROJECT_DIR}/CMakeLists.txt" ] || [ ! -f "${PROJECT_DIR}/package.xml" ]; then
    echo -e "Ошибка: CMakeLists.txt или package.xml не найдены"
    exit 1
fi
echo -e "Структура проекта валидна"
echo ""

mkdir -p "${PROJECT_DIR}/worlds"
mkdir -p "${PROJECT_DIR}/map"

echo -e "Загрузка необходимых моделей..."
mkdir -p "${HOME}/.gazebo/models"
cp -r "${PROJECT_DIR}/resources/." "${HOME}/.gazebo/models/"
echo ""

echo -e "Сборка ROS пакета..."
cd "$CATKIN_WS"
catkin_make
echo ""
source "${CATKIN_WS}/devel/setup.bash"
echo -e "Пакет собран"
echo ""

echo -e "Установка прав доступа..."
chmod -R +x ${SCRIPT_DIR}
chmod -R +x ${PROJECT_DIR}/web
echo ""

echo -e "Генерация мира..."
PYTHONIOENCODING=utf-8 python3 "${SCRIPT_DIR}/generate_world.py"
echo ""

echo -e "Для запуска симуляции:"
echo -e "  roslaunch pipeline_monitoring_starwars simulator.launch"
echo ""
echo -e "Для запуска навигации:"
echo -e "  roslaunch pipeline_monitoring_starwars scanner.launch"
echo ""
echo -e "Для запуска Web приложения:"
echo -e "  rosrun pipeline_monitoring_starwars web.py"
echo -e "  В браузере: localhost:5555"
echo ""
