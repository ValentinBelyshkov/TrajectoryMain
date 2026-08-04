import json, os, glob

for path in glob.glob('/opt/main/Trajectory/Database/projects/*/metadata.json'):
    with open(path, 'r') as f:
        data = json.load(f)
    changed = False
    for key in ['video_filename', 'frames_path']:
        val = data.get(key)
        if val and isinstance(val, str):
            if '/app/trajectory-db/' in val:
                data[key] = val.replace('/app/trajectory-db/', '/opt/main/Trajectory/Database/')
                changed = True
            elif '/home/orb/Database/' in val:
                data[key] = val.replace('/home/orb/Database/', '/opt/main/Trajectory/Database/')
                changed = True
    if changed:
        with open(path, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print('Fixed:', path)
