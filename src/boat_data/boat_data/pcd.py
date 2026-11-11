"""Portable PCD output from ROS PointCloud2; source fields/axes are preserved."""
import re
import struct
from pathlib import Path

TYPES = {1: ('b',1,'I'), 2: ('B',1,'U'), 3: ('h',2,'I'), 4: ('H',2,'U'),
         5: ('i',4,'I'), 6: ('I',4,'U'), 7: ('f',4,'F'), 8: ('d',8,'F')}


def write_pcd(msg, path):
    if msg.point_step <= 0 or msg.row_step < msg.width*msg.point_step or len(msg.data) != msg.row_step*msg.height:
        raise ValueError('Invalid cloud layout')
    fields = list(msg.fields)
    if not {'x','y','z'}.issubset({f.name for f in fields}):
        raise ValueError('PCD requires XYZ')
    names = [f.name for f in fields]
    if len(set(names)) != len(names):
        raise ValueError('Duplicate fields')
    formats = []
    for f in fields:
        if not re.fullmatch('[A-Za-z][A-Za-z0-9_]*', f.name) or f.datatype not in TYPES or not 1 <= f.count <= 16:
            raise ValueError('Unsupported PCD field')
        code,size,_ = TYPES[f.datatype]
        if f.offset < 0 or f.offset+size*f.count > msg.point_step:
            raise ValueError('Field exceeds point stride')
        formats.append(struct.Struct(('>' if msg.is_bigendian else '<')+code*f.count))
    header = '\n'.join([
        '# .PCD v0.7 - original sensor coordinates, no motion compensation', 'VERSION 0.7',
        'FIELDS '+' '.join(names),
        'SIZE '+' '.join(str(TYPES[f.datatype][1]) for f in fields),
        'TYPE '+' '.join(TYPES[f.datatype][2] for f in fields),
        'COUNT '+' '.join(str(f.count) for f in fields),
        f'WIDTH {msg.width}',f'HEIGHT {msg.height}', 'VIEWPOINT 0 0 0 1 0 0 0',
        f'POINTS {msg.width*msg.height}', 'DATA ascii'])+'\n'
    with Path(path).open('w',encoding='ascii') as stream:
        stream.write(header)
        for row in range(msg.height):
            for col in range(msg.width):
                offset = row*msg.row_step+col*msg.point_step
                values=[]
                for field,fmt in zip(fields,formats):
                    values.extend(fmt.unpack_from(msg.data,offset+field.offset))
                stream.write(' '.join(map(str,values))+'\n')
