#!/usr/bin/env python3
"""Audit the supplied sheet DXF; units alone are deliberately not used as scale."""
import collections
import hashlib
import json
from pathlib import Path
import ezdxf
from ezdxf import bbox

ROOT = Path(__file__).resolve().parents[1]


def analyze(path):
    doc = ezdxf.readfile(path)
    dims = []
    for e in doc.modelspace().query('DIMENSION'):
        raw = float(e.get_measurement())
        factor = e.override().get('dimlfac', 1.0)
        dims.append(dict(handle=e.dxf.handle, raw=raw, dimlfac=factor,
                         displayed_mm=raw * factor,
                         cached_mm=e.dxf.get('actual_measurement'),
                         text=e.dxf.get('text', '')))
    layers = {}
    for layer in doc.layers:
        es = [e for e in doc.modelspace() if e.dxf.layer == layer.dxf.name]
        box = bbox.extents(es, fast=True)
        layers[layer.dxf.name] = dict(count=len(es),
            types=dict(collections.Counter(e.dxftype() for e in es)),
            bounds=[list(box.extmin), list(box.extmax)] if box.has_data else None)
    return dict(source=str(path.name), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        header={k: str(doc.header.get(k)) for k in
                ['$INSUNITS', '$MEASUREMENT', '$LUNITS', '$LTSCALE', '$DIMSCALE', '$DIMLFAC']},
        layers=layers, dimensions=dims,
        inserts=[dict(name=e.dxf.name, layer=e.dxf.layer, insert=list(e.dxf.insert),
                      scale=[e.dxf.xscale, e.dxf.yscale, e.dxf.zscale], rotation=e.dxf.rotation)
                 for e in doc.modelspace().query('INSERT')],
        conclusion={'meters_per_drawing_unit': 0.03, 'nominal_width_m': 10.0,
                    'nominal_height_m': 6.0, 'drawing_scale': '1:30',
                    'evidence': 'DIMENSION 4DE / 4EC: 333.333333*30=10000 mm; 200*30=6000 mm; PDF agrees',
                    'physical_as_built': 'not measured', 'dwg': 'retained, not independently decoded'})


if __name__ == '__main__':
    result = analyze(ROOT / 'assets/original/2026Track 도면 시트1.dxf')
    out = ROOT / 'reports/track_audit.json'
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(out)
