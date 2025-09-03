import argparse
import pandas as pd
import os

class Message:
    def __init__(self, line: str):
        line = line.replace(',', '')
        self.id = int(line.split(' ')[1])
        self.distance = float(line.split(' ')[3])

    def to_dict(self):
        return {
            'id': self.id,
            'distance': self.distance
        }

    def __str__(self):
        return f"{self.id}, {self.distance}"

if __name__ == '__main__':

    argparser = argparse.ArgumentParser(description='Convert raw data file to table format.')
    argparser.add_argument('--input', type=str, required=True, help='Input raw data file path.')
    argparser.add_argument('--output', type=str, required=True, help='Output tables directory.')
    argparser.add_argument('--real_values_file', type=str, help='File with real values for each id. Format: id,distance')
    args = argparser.parse_args()

    lines: list[str] = []
    with open(args.input, 'r') as f:
        lines = f.readlines()

    messages: list[Message] = []
    for line in lines:
        messages.append(Message(line))

    # make directory for all output files
    if not os.path.exists(args.output):
        os.makedirs(args.output)
        print(f"Created directory {args.output}")
    print(f"Saving to {args.output}")
    # find unique ids
    ids = set()
    for message in messages:
        ids.add(message.id)

    real_values = {}
    if args.real_values_file is not None:
        with open(args.real_values_file, 'r') as f:
            lines = f.readlines()

        for line in lines:
            id, distance = line.split(',')
            id = int(id)
            distance = float(distance)
            real_values[id] = distance
        print(f"Loaded real values for {len(real_values)} ids: {real_values}")

    # save statistics for all ids
    stats = []
    for id in ids:
        messages_for_id = [m.to_dict() for m in messages if m.id == id]

        with open(os.path.join(args.output, f"{id}.csv"), 'w') as f:
            print(f"Saving {len(messages_for_id)} messages for id {id}")
            df = pd.DataFrame(messages_for_id, columns=['id', 'distance'])
            df.to_csv(f, index=False)
        stats.append({
            'id': id,
            'count': len(messages_for_id),
            'mean': sum([m['distance'] for m in messages_for_id]) / len(messages_for_id),
            'min': min([m['distance'] for m in messages_for_id]),
            'max': max([m['distance'] for m in messages_for_id]),
            'std': pd.Series([m['distance'] for m in messages_for_id]).std(),
            'median': pd.Series([m['distance'] for m in messages_for_id]).median(),
        })

        if id in real_values:
            stats[-1]['real_value'] = real_values[id]
            stats[-1]['error'] = stats[-1]['mean'] - real_values[id]
            stats[-1]['error_pct'] = (stats[-1]['mean'] - real_values[id]) / real_values[id] * 100

    df_stats = pd.DataFrame(stats)
    df_stats.to_csv(os.path.join(args.output, "stats.csv"), index=False)
