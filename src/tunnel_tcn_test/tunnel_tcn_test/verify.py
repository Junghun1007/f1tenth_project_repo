"""Replay bundled reference sequences through the actual selected backend."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from .features import FeatureStream, EventDecoder, softmax
from .backend import TensorRTBackend, OnnxBackend


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['tensorrt', 'onnx'], default='tensorrt')
    parser.add_argument('--cache-dir', default='~/.cache/tunnel_tcn_test')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    except ImportError:
        share = Path(__file__).resolve().parent.parent
    else:
        try:
            share = Path(get_package_share_directory('tunnel_tcn_test'))
        except PackageNotFoundError:
            share = Path(__file__).resolve().parent.parent
    models = share / 'models'
    provenance = json.loads((models / 'provenance.json').read_text())
    model = models / 'brightness31_partial_int8.onnx'
    if hashlib.sha256(model.read_bytes()).hexdigest() != provenance['onnx_sha256']:
        raise ValueError('ONNX checksum mismatch')
    golden = models / 'golden_replay.npz'
    if hashlib.sha256(golden.read_bytes()).hexdigest() != provenance['golden_sha256']:
        raise ValueError('Replay checksum mismatch')
    backend = TensorRTBackend(model, args.cache_dir, provenance['onnx_sha256']) if args.backend == 'tensorrt' else OnnxBackend(model)
    rows = []
    try:
        with np.load(golden) as data:
            for index, item in enumerate(provenance['replay_items']):
                stream = FeatureStream(warmup_seconds=0.)
                decoder, reference = EventDecoder(), EventDecoder()
                actual_events, expected_events, errors, times = [], [], [], []
                statics, expected = data[f'static_{index}'], data[f'probabilities_{index}']
                for frame, values in enumerate(statics):
                    timestamp = frame / item['fps']
                    context = stream.push_static(values, timestamp)
                    started = time.perf_counter()
                    result = softmax(backend.infer(context))
                    times.append((time.perf_counter() - started) * 1000)
                    errors.append(float(np.max(np.abs(result - expected[frame]))))
                    for d, p, events in [(decoder, result, actual_events), (reference, expected[frame], expected_events)]:
                        event = d.step(p, timestamp)
                        if event:
                            events.append((frame, event['type']))
                rows.append({'id': item['id'], 'split': item['split'], 'frames': len(statics),
                             'max_probability_error': max(errors), 'event_frames_equal': actual_events == expected_events,
                             'actual_events': actual_events, 'expected_events': expected_events,
                             'inference_plus_softmax_p50_ms': float(np.median(times))})
    finally:
        backend.close()
    success = all(r['max_probability_error'] <= .005 and r['event_frames_equal'] for r in rows)
    report = {'backend': args.backend, 'passed': success, 'items': rows,
              'scope': 'Frozen static features -> timestamp context -> model -> decoder, including startup padding. Live runtime has a separate 2.5 s warmup.',
              'jetson_verified': args.backend == 'tensorrt'}
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.output:
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    if not success:
        raise SystemExit('Replay differs from reference: inspect the report before using this engine')


if __name__ == '__main__':
    main()
