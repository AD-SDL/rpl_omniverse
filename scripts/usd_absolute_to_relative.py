from pathlib import Path
from time import time
import sys

def find_relative_path(fname_usd, fname_ref, base='rpl_omniverse'):
    parts_usd = fname_usd.resolve().parts
    parts_ref = fname_ref.parts

    parts_usd = list(parts_usd[parts_usd.index(base)+1:-1])
    parts_ref = list(parts_ref[parts_ref.index(base)+1:-1])

    while len(parts_usd) and len(parts_ref) and parts_usd[0] == parts_ref[0]:
        parts_usd.pop(0)
        parts_ref.pop(0)

    p = str(Path('../' * len(parts_usd), *parts_ref, fname_ref.name))
    if not p.startswith('.'):
        p = './' + p

    return p

def process_file(fname_usd):
    changed = False

    base = 'rpl_omniverse'
    offset = len(base)

    out_lines = []
    with open(fname_usd) as f:
        for line in f:
            if '@' in line and base in line:
                a, fname_ref, b = line.split('@')

                fname_ref = fname_ref.removeprefix('file:')
                fname_ref = find_relative_path(Path(fname_usd), Path(fname_ref), base=base)

                line = '@'.join((a, fname_ref, b))

                changed = True

            out_lines.append(line)

    if changed:
        out_lines = ''.join(out_lines)
        with open(fname_usd, 'w') as f:
            f.write(out_lines)

    return changed

def main():
    S = time()

    filter_ = sys.argv[1:]

    for fname_usd in Path('../usd/').rglob('*.usda'):
        if len(filter_) and str(fname_usd.with_suffix('').name) not in filter_:
            continue

        if process_file(fname_usd):
            print('Changed:', fname_usd)

    E = time()
    print(f'Finished all in {round(E-S, 2)}s')

if __name__ == '__main__':
    main()
