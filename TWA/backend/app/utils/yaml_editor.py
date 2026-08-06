# utils/yaml_editor.py
import re
import os
import tempfile
import shutil
from typing import Optional

def update_slam_yaml(
    yaml_path: str,
    save_filename: Optional[str] = None,
    load_filename: Optional[str] = None
) -> bool:
    """
    Обновляет real.yaml для ORB-SLAM3:
    - Удаляет все существующие System.SaveAtlasToFile и System.LoadAtlasFromFile
    - Добавляет System.SaveAtlasToFile, если передан save_filename
    - Добавляет System.LoadAtlasFromFile, если передан load_filename
    
    Args:
        yaml_path: Полный путь к файлу (внутри контейнера бэкенда)
        save_filename: Новое имя для сохранения (полный путь внутри контейнера SLAM)
        load_filename: Путь к файлу для загрузки (полный путь внутри контейнера SLAM)
    
    Returns:
        bool: True если успешно
    """
    if not os.path.exists(yaml_path):
        print(f"❌ YAML file not found: {yaml_path}")
        return False
    
    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        # 1. Удаляем все вхождения Save и Load (включая закомментированные)
        new_lines = []
        pattern = re.compile(r'^\s*#?\s*System\.(SaveAtlasToFile|LoadAtlasFromFile)\s*:')
        for line in lines:
            if not pattern.match(line):
                new_lines.append(line)
        
        content = "".join(new_lines)
        
        # 2. Подготавливаем строки для вставки
        to_insert = ""
        if save_filename:
            to_insert += f'System.SaveAtlasToFile: "{save_filename}"\n'
        if load_filename:
            to_insert += f'System.LoadAtlasFromFile: "{load_filename}"\n'
        
        if to_insert:
            # Ищем место для вставки перед секцией Camera Parameters
            # Сначала ищем заголовок Camera Parameters
            header_match = re.search(r'^#+\s*Camera Parameters', content, flags=re.MULTILINE)
            if header_match:
                # Пытаемся найти разделительную линию перед заголовком
                # Ищем последнюю линию из решеток и дефисов перед заголовком
                before_header = content[:header_match.start()]
                separator_match = list(re.finditer(r'^#+\s*-+\s*$', before_header, flags=re.MULTILINE))
                
                if separator_match:
                    insert_pos = separator_match[-1].start()
                else:
                    insert_pos = header_match.start()
                
                content = content[:insert_pos] + to_insert + content[insert_pos:]
            else:
                # Если секция не найдена, просто добавляем в конец
                if not content.endswith('\n'):
                    content += '\n'
                content += to_insert
        
        # 3. Атомарная запись
        dir_name = os.path.dirname(yaml_path)
        with tempfile.NamedTemporaryFile(
            mode='w', 
            dir=dir_name, 
            delete=False, 
            suffix='.tmp',
            encoding='utf-8'
        ) as tmp:
            tmp.write(content)
            tmp_path = tmp.name
        
        shutil.move(tmp_path, yaml_path)
        os.chmod(yaml_path, 0o666)
        print(f"✅ Updated YAML: {yaml_path}")
        if save_filename:
            print(f"   → SaveAtlasToFile: \"{save_filename}\"")
        if load_filename:
            print(f"   → LoadAtlasFromFile: \"{load_filename}\"")
        return True
        
    except Exception as e:
        print(f"❌ YAML update failed: {e}")
        if 'tmp_path' in locals() and os.path.exists(tmp_path):
            os.remove(tmp_path)
        return False
