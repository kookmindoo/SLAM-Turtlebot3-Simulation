"""Convenience CLI for rosbag2 recordings using repository defaults."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path

import yaml


def _load_topics(config: Path) -> list[str]:
    data = yaml.safe_load(config.read_text())
    params = data.get('tb3_data_tools', {}).get('ros__parameters', {})
    topics = params.get('topics', [])
    return [entry.split('|')[0] for entry in topics]


def main() -> None:
    parser = argparse.ArgumentParser(description='Launch ros2 bag record with TurtleBot3 defaults.')
    parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1] / 'config' / 'recorder_topics.yaml')
    parser.add_argument('--bag-path', type=Path, default=Path('/tmp/tb3_dataset'))
    parser.add_argument('--storage-id', default='sqlite3')
    parser.add_argument('--max-cache-size', default='100', help='ros2 bag record --max-cache-size value (MB).')
    args = parser.parse_args()

    topics = _load_topics(args.config)
    cmd = [
        'ros2', 'bag', 'record',
        '--storage', args.storage_id,
        '--max-cache-size', args.max_cache_size,
        '--output', str(args.bag_path),
    ] + topics
    print('Executing: ' + ' '.join(cmd))
    subprocess.run(cmd, check=True)


if __name__ == '__main__':
    main()
