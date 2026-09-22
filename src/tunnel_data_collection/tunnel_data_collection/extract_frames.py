"""Extract lossless PNG RGB/BEV datasets from a tunnel rosbag2 session."""

import argparse
import csv
from pathlib import Path
import sys

import cv2

from .image_utils import FrameRecord, decode_image, pair_frames


def parse_arguments(arguments=None):
    parser = argparse.ArgumentParser(
        description="Extract /camera/image_rect and /camera/image_bev as PNG frames."
    )
    parser.add_argument("bag", type=Path, help="rosbag2 session directory")
    parser.add_argument("--output", type=Path, help="new output dataset directory")
    parser.add_argument("--rgb-topic", default="/camera/image_rect")
    parser.add_argument("--bev-topic", default="/camera/image_bev")
    parser.add_argument(
        "--every-nth", type=int, default=1, help="keep every Nth message per topic"
    )
    parser.add_argument(
        "--max-pair-delta-ms",
        type=float,
        default=20.0,
        help="maximum RGB/BEV header timestamp difference in paired_frames.csv",
    )
    return parser.parse_args(arguments)


def _write_manifest(path, records):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "index",
                "header_stamp_ns",
                "bag_receive_stamp_ns",
                "filename",
                "width",
                "height",
                "encoding",
            ]
        )
        for frame in records:
            writer.writerow(
                [
                    frame.index,
                    frame.header_ns,
                    frame.bag_ns,
                    frame.filename,
                    frame.width,
                    frame.height,
                    frame.encoding,
                ]
            )


def extract(arguments=None):
    options = parse_arguments(arguments)
    if options.every_nth <= 0:
        raise ValueError("--every-nth must be positive")
    if options.max_pair_delta_ms < 0:
        raise ValueError("--max-pair-delta-ms must be nonnegative")
    bag = options.bag.resolve()
    if not bag.is_dir() or not (bag / "metadata.yaml").is_file():
        raise ValueError(f"not a rosbag2 directory: {bag}")
    output = (
        options.output.resolve()
        if options.output
        else bag.with_name(f"{bag.name}_frames")
    )
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"output directory is not empty: {output}")
    rgb_directory = output / "rgb"
    bev_directory = output / "bev"
    rgb_directory.mkdir(parents=True, exist_ok=True)
    bev_directory.mkdir(parents=True, exist_ok=True)

    # Imports remain here so image conversion/unit tests do not require a sourced ROS shell.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    wanted = (options.rgb_topic, options.bev_topic)
    for topic in wanted:
        if topic_types.get(topic) != "sensor_msgs/msg/Image":
            raise ValueError(f"bag has no sensor_msgs/msg/Image topic {topic}")
    image_type = get_message("sensor_msgs/msg/Image")
    records = {options.rgb_topic: [], options.bev_topic: []}
    received = {options.rgb_topic: 0, options.bev_topic: 0}
    failures = []
    while reader.has_next():
        topic, serialized, bag_ns = reader.read_next()
        if topic not in records:
            continue
        sequence = received[topic]
        received[topic] += 1
        if sequence % options.every_nth:
            continue
        message = deserialize_message(serialized, image_type)
        header_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        directory = rgb_directory if topic == options.rgb_topic else bev_directory
        index = len(records[topic])
        filename = f"{index:08d}_{header_ns}.png"
        try:
            image = decode_image(
                data=message.data,
                width=int(message.width),
                height=int(message.height),
                step=int(message.step),
                encoding=message.encoding,
            )
            if not cv2.imwrite(str(directory / filename), image):
                raise RuntimeError("cv2.imwrite returned false")
        except (ValueError, RuntimeError, cv2.error) as error:
            failures.append(f"{topic} message {sequence}: {error}")
            continue
        records[topic].append(
            FrameRecord(
                index=index,
                header_ns=header_ns,
                bag_ns=int(bag_ns),
                filename=filename,
                width=int(message.width),
                height=int(message.height),
                encoding=message.encoding,
            )
        )

    rgb_records = records[options.rgb_topic]
    bev_records = records[options.bev_topic]
    _write_manifest(output / "rgb_frames.csv", rgb_records)
    _write_manifest(output / "bev_frames.csv", bev_records)
    pairs = pair_frames(
        rgb_records,
        bev_records,
        round(options.max_pair_delta_ms * 1_000_000),
    )
    with (output / "paired_frames.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "pair_index",
                "rgb_filename",
                "bev_filename",
                "rgb_header_stamp_ns",
                "bev_header_stamp_ns",
                "delta_ns",
            ]
        )
        for index, (rgb_frame, bev_frame, delta) in enumerate(pairs):
            writer.writerow(
                [
                    index,
                    rgb_frame.filename,
                    bev_frame.filename,
                    rgb_frame.header_ns,
                    bev_frame.header_ns,
                    delta,
                ]
            )
    if failures:
        (output / "extraction_errors.txt").write_text(
            "\n".join(failures) + "\n", encoding="utf-8"
        )
    return output, len(rgb_records), len(bev_records), len(pairs), len(failures)


def main(arguments=None):
    try:
        output, rgb_count, bev_count, pair_count, failure_count = extract(arguments)
    except (FileExistsError, OSError, ValueError, RuntimeError) as error:
        print(f"Frame extraction failed: {error}", file=sys.stderr)
        return 1
    print(
        f"Extracted RGB={rgb_count}, BEV={bev_count}, paired={pair_count}, "
        f"errors={failure_count} to {output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
