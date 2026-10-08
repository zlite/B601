"""Read Creality's stock assembly, preserving repeated component instances."""
import os
os.environ.setdefault('XDG_CACHE_HOME','/tmp/b601-cad-cache')
import hashlib
import json
import re
from pathlib import Path
import cadquery as cq

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'Ender3.STEP'
COMMIT='88c7758cea9d0d00a54fdb238bedb3b33425f409'


class InstanceAssembly(cq.Assembly):
    def add(self,*args,**kwargs):
        name=kwargs.get('name')
        if name and name in self.objects:
            i=2
            while f'{name}__instance{i}' in self.objects:i+=1
            kwargs['name']=f'{name}__instance{i}'
        return super().add(*args,**kwargs)


def product_names(source):
    """Recover readable product labels behind this STEP's NAUO instance IDs."""
    entities = dict(re.findall(r'#(\d+)\s*=\s*(.*?);',
                              source.read_text(errors='replace'), re.S))
    labels = {}
    for value in entities.values():
        if not value.lstrip().startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
            continue
        strings = re.findall(r"'((?:[^']|'')*)'", value)
        refs = re.findall(r'#(\d+)', value)
        if not strings or len(refs) < 2:
            continue
        definition = entities[refs[1]]
        formation = entities[re.findall(r'#(\d+)', definition)[0]]
        product = entities[re.findall(r'#(\d+)', formation)[0]]
        names = re.findall(r"'((?:[^']|'')*)'", product)
        labels[strings[0]] = names[1].replace("''", "'")
    return labels


def main():
    labels = product_names(SOURCE)
    assembly=InstanceAssembly.load(str(SOURCE),unit='MM')
    rows=[]
    # Iteration supplies each leaf with its accumulated assembly location.
    for shape,name,location,color in assembly:
        world=shape.moved(location);b=world.BoundingBox()
        instance = name.rsplit('/', 1)[-1].split('__instance')[0]
        rows.append({'name':name,'product_name':labels.get(instance, instance),
                     'solids':len(world.Solids()),
                     'min_assembly_mm':[b.xmin,b.ymin,b.zmin],
                     'max_assembly_mm':[b.xmax,b.ymax,b.zmax],
                     'volume_mm3':world.Volume()})
    out={'source_url':f'https://github.com/Creality3DPrinting/Ender-3/blob/{COMMIT}/Ender-3%20Mechanical/STP/Ender3.STEP',
         'commit':COMMIT,'sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
         'objects':rows,'motion_ready':False,'registered':False,
         'scope':'Stock assembly only; moving components need separate registration and custom syringe drive is absent'}
    (HERE/'inspection.json').write_text(json.dumps(out,indent=2)+'\n')
    print('Extracted',len(rows),'located assembly components',flush=True)
    for row in rows:
        if any(word in row['name'].lower() for word in ['profile','bed','frame','plate','lcd','x axis']):
            print(row,flush=True)


if __name__=='__main__':main()
