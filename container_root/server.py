# /root/flask_server.py
from flask import Flask, jsonify, request
import subprocess
import threading
import time
import os
import uuid
import signal
import sys
from datetime import datetime
from functools import partial

app = Flask(__name__)

# Глобальное хранилище задач: task_id -> {proc, stdout, stderr, status, cwd}
tasks = {}

# Словарь для отслеживания последних задач каждого типа
last_tasks_by_type = {}

# Флаг для отслеживания состояния сервера
server_shutting_down = False

def _run_task(task_id, cmd, cwd):
    """Фоновая функция для запуска команды и сбора вывода"""
    try:
        # Проверяем, не завершается ли сервер
        if server_shutting_down:
            tasks[task_id]['status'] = 'cancelled'
            tasks[task_id]['error'] = 'Server is shutting down'
            return

        # Запускаем процесс
        proc = subprocess.Popen(
            cmd,
            shell=True,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,  # построчный буфер
            universal_newlines=True,
            start_new_session=True
        )

        tasks[task_id]['proc'] = proc
        tasks[task_id]['start_time'] = datetime.utcnow().isoformat()
        
        # Update the last task for this type
        task_type = tasks[task_id]['type']
        last_tasks_by_type[task_type] = task_id

        stdout_lines = []
        stderr_lines = []

        # Читаем stdout в реальном времени
        def read_stdout():
            for line in iter(proc.stdout.readline, ''):
                if server_shutting_down:
                    break
                stdout_lines.append(line)
                tasks[task_id]['stdout'] = ''.join(stdout_lines[-100:])  # последние 100 строк

        def read_stderr():
            for line in iter(proc.stderr.readline, ''):
                if server_shutting_down:
                    break
                stderr_lines.append(line)
                tasks[task_id]['stderr'] = ''.join(stderr_lines[-100:])

        t_out = threading.Thread(target=read_stdout, daemon=True)
        t_err = threading.Thread(target=read_stderr, daemon=True)
        t_out.start()
        t_err.start()

        # Ждём завершения или до завершения сервера
        while proc.poll() is None:
            if server_shutting_down:
                # Прерываем процесс при завершении сервера
                try:
                    # Убиваем всю группу процессов
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    try:
                        proc.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except:
                    pass  # Процесс уже мог завершиться
                break
            time.sleep(0.1)  # Небольшая задержка чтобы не нагружать CPU

        if not server_shutting_down:
            tasks[task_id]['returncode'] = proc.returncode
            tasks[task_id]['status'] = 'finished' if proc.returncode == 0 else 'failed'
            tasks[task_id]['end_time'] = datetime.utcnow().isoformat()

    except Exception as e:
        if not server_shutting_down:
            tasks[task_id]['status'] = 'error'
            tasks[task_id]['error'] = str(e)

@app.route('/health')
def health():
    return jsonify(status="ok")

@app.route('/start', methods=['POST'])
def start_task():
    data = request.json
    task_type = data.get('type')

    # Определяем команду и рабочую директорию
    if task_type == 'slam':
        cmd = "ros2 launch orb_slam3_ros2_wrapper unirobot_mono.launch.py"
        cwd = "/root/colcon_ws"
    elif task_type == 'image_pub':
        video_name = data.get('video', 'Mill-video')
        cmd = f"python3 image_publish.py {video_name}/"
        cwd = "/root/Database"
    elif task_type == 'relay':
        cmd = "python3 relay.py"
        cwd = "/root/TerraSLAM_relay"
    elif task_type == 'callback':
        cmd = "python counter_callback.py"
        cwd = "."  # current directory where the server is running
    else:
        return jsonify(error="Unknown task type"), 400

    # Создаём задачу
    task_id = str(uuid.uuid4())
    tasks[task_id] = {
        'type': task_type,
        'cmd': cmd,
        'cwd': cwd,
        'status': 'starting',
        'stdout': '',
        'stderr': ''
    }

    # Update the last task for this type immediately
    last_tasks_by_type[task_type] = task_id

    # Запускаем в фоне
    thread = threading.Thread(target=_run_task, args=(task_id, cmd, cwd))
    thread.daemon = True
    thread.start()

    return jsonify(task_id=task_id, status="started")

@app.route('/status/<task_id_or_type>')
def get_status(task_id_or_type):
    # Check if the parameter is a task type (like 'callback', 'slam', etc.) or a specific task ID
    if task_id_or_type in ['slam', 'image_pub', 'relay', 'callback']:
        # Get the last task ID for this type
        last_task_id = last_tasks_by_type.get(task_id_or_type)
        if last_task_id is None or last_task_id not in tasks:
            return jsonify(error="No task of this type found"), 404
        
        task = tasks[last_task_id]
        
        # Check if the process is still running
        proc = task.get('proc')
        if proc and proc.poll() is None:
            task['status'] = 'running'
        elif task['status'] == 'starting' and proc is not None:
            # Double-check the process status if it's still starting
            if proc.poll() is None:
                task['status'] = 'running'
            else:
                # Process has finished, but we might not have updated the status yet
                if hasattr(proc, 'returncode') and proc.returncode is not None:
                    task['returncode'] = proc.returncode
                    task['status'] = 'finished' if proc.returncode == 0 else 'failed'
                    task['end_time'] = datetime.utcnow().isoformat()
        
        # Create a copy of the task dict without the non-serializable proc object
        task_copy = {k: v for k, v in task.items() if k != 'proc'}
        return jsonify(task_copy)
    else:
        # Original behavior: treat as a specific task ID
        task = tasks.get(task_id_or_type)
        if not task:
            return jsonify(error="Task not found"), 404

        # Проверяем, жив ли процесс
        proc = task.get('proc')
        if proc and proc.poll() is None:
            task['status'] = 'running'

        # Create a copy of the task dict without the non-serializable proc object
        task_copy = {k: v for k, v in task.items() if k != 'proc'}
        return jsonify(task_copy)

@app.route('/list')
def list_tasks():
    # Возвращаем только метаданные (без логов и proc объекта)
    summary = {
        tid: {k: v for k, v in t.items() if k not in ('stdout', 'stderr', 'proc')}
        for tid, t in tasks.items()
    }
    return jsonify(summary)

@app.route('/stop/<task_id_or_type>', methods=['POST'])
def stop_task(task_id_or_type):
    # Check if the parameter is a task type (like 'callback', 'slam', etc.) or a specific task ID
    if task_id_or_type in ['slam', 'image_pub', 'relay', 'callback']:
        # Get the last task ID for this type
        last_task_id = last_tasks_by_type.get(task_id_or_type)
        if last_task_id is None or last_task_id not in tasks:
            return jsonify(error="No task of this type found"), 404
        
        task = tasks[last_task_id]
        
        # Check if the process is still running
        proc = task.get('proc')
        if proc and proc.poll() is None:
            task['status'] = 'running'
        elif task['status'] != 'running':
            return jsonify(error="No running task of this type found"), 404
        
        if proc:
            try:
                # Убиваем всю группу процессов
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                
                task['status'] = 'stopped'
                task['end_time'] = datetime.utcnow().isoformat()
                return jsonify(message=f"Task {last_task_id} of type {task_id_or_type} stopped successfully")
            except Exception as e:
                return jsonify(error=f"Failed to stop task: {str(e)}"), 500
        else:
            return jsonify(error="Task has no associated process"), 400
    else:
        # Original behavior: treat as a specific task ID
        task = tasks.get(task_id_or_type)
        if not task:
            return jsonify(error="Task not found"), 404
        
        proc = task.get('proc')
        if proc:
            try:
                # Убиваем всю группу процессов
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                
                task['status'] = 'stopped'
                task['end_time'] = datetime.utcnow().isoformat()
                return jsonify(message=f"Task {task_id_or_type} stopped successfully")
            except Exception as e:
                return jsonify(error=f"Failed to stop task: {str(e)}"), 500
        else:
            return jsonify(error="Task has no associated process"), 400

def shutdown_handler(signum, frame):
    """Обработчик сигналов завершения для корректного завершения работы"""
    global server_shutting_down
    print(f"Received signal {signum}, initiating graceful shutdown...")
    
    # Устанавливаем флаг завершения
    server_shutting_down = True
    
    # Завершаем все активные процессы
    for task_id, task_data in tasks.items():
        if task_data.get('status') in ['running', 'starting'] and 'proc' in task_data:
            proc = task_data['proc']
            if proc and proc.poll() is None:  # Процесс всё ещё запущен
                print(f"Terminating process for task {task_id}")
                try:
                    # Убиваем всю группу процессов
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    try:
                        proc.wait(timeout=5)  # Ждем завершения до 5 секунд
                    except subprocess.TimeoutExpired:
                        print(f"Process for task {task_id} did not terminate gracefully, killing it...")
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)  # Принудительно убиваем процесс если он не завершился
                except Exception as e:
                    print(f"Error terminating process for task {task_id}: {e}")
    
    print("All processes terminated, shutting down server...")
    sys.exit(0)

def register_shutdown_handlers():
    """Регистрация обработчиков сигналов завершения"""
    signal.signal(signal.SIGTERM, shutdown_handler)
    signal.signal(signal.SIGINT, shutdown_handler)  # Обработка Ctrl+C
    if hasattr(signal, 'SIGQUIT'):
        signal.signal(signal.SIGQUIT, shutdown_handler)

if __name__ == '__main__':
    # Регистрируем обработчики сигналов
    register_shutdown_handlers()
    
    # Убедись, что нужные папки существуют
    os.makedirs("/root/Database", exist_ok=True)
    os.makedirs("/root/TerraSLAM_relay", exist_ok=True)

    app.run(host='0.0.0.0', port=5000, threaded=True)
