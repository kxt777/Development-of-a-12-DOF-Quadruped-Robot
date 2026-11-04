"""Copy recorded messages up to a measured receive-time cutoff; source stays intact."""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import rosbag2_py

p = argparse.ArgumentParser()
p.add_argument('source', type=Path)
p.add_argument('output', type=Path)
p.add_argument('--cutoff-ns', type=int, required=True)
args = p.parse_args()
if args.output.exists():
    raise FileExistsError(args.output)
source_meta = args.source / 'metadata.yaml'
before = hashlib.sha256(source_meta.read_bytes()).hexdigest()
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(args.source), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr','cdr'))
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=str(args.output), storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr','cdr'))
for topic in reader.get_all_topics_and_types():
    writer.create_topic(topic)
counts = collections.Counter()
first = last = None
while reader.has_next():
    topic, data, stamp = reader.read_next()
    if stamp <= args.cutoff_ns:
        writer.write(topic, data, stamp)
        counts[topic] += 1
        first = stamp if first is None else min(first, stamp)
        last = stamp if last is None else max(last, stamp)
del writer
assert hashlib.sha256(source_meta.read_bytes()).hexdigest() == before
result = dict(source=str(args.source), output=str(args.output),
              cutoff_receive_time_ns=args.cutoff_ns, first_receive_ns=first,
              last_receive_ns=last, duration_s=(last-first)/1e9 if first else 0,
              counts=dict(counts), source_metadata_sha256=before,
              method='Original serialized messages and timestamps copied; no interpolation or post-gap data')
(args.output.parent/(args.output.name+'_trim.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
