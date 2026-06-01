"""Plotting code for recorded bags."""

from finger_interfaces.msg import MotorFeedback

import os

import csv

import glob

import matplotlib.pyplot as plt

import numpy as np

from rclpy.serialization import deserialize_message

import rosbag2_py

DRAKE_JOINT_TORQUES = False

DATA_FOLDER = 'src/robotic-finger-sim/finger_recorder/may28-2026/test9/'

FILE = 'finger_bag_20260528_175306_0.mcap'

JOINT_LABELS = ['splay [deg]', 'mcp_flex [deg]', 'pip/dip_flex [deg]']

def save_data_to_csv(data, joint_labels, output_file='plotted_data.csv'):
    """Save the loaded data to a CSV file."""
    # Flatten the data structure into a single dataframe-like format
    all_times = set()
    data_dict = {}
    
    for group, series_dict in data.items():
        for series, values in series_dict.items():
            if not values:
                continue
            for t, positions in values:
                all_times.add(t)
                for i, label in enumerate(joint_labels):
                    key = f"{group}_{series}_{label}"
                    if key not in data_dict:
                        data_dict[key] = {}
                    data_dict[key][t] = np.rad2deg(positions[i])
    
    if not all_times:
        print("No data to save")
        return
    
    sorted_times = sorted(all_times)
    
    with open(output_file, 'w', newline='') as csvfile:
        fieldnames = ['Time (s)'] + sorted(data_dict.keys())
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        for t in sorted_times:
            row = {'Time (s)': f"{t:.6f}"}
            for col in fieldnames[1:]:
                row[col] = f"{data_dict[col].get(t, '')}"  # Empty string if time not in this series
            writer.writerow(row)
    
    print(f"Data saved to {output_file}")

if DRAKE_JOINT_TORQUES:
    TOPICS = {
        'motor_pos_actual_feedback':   ('motor_pos', 'actual'),
        'motor_pos_setpoint_feedback': ('motor_pos', 'setpoint'),
        'motor_pos_drake_feedback': ('motor_pos', 'drake'),
        'actual/joint_feedback':       ('joint_angle', 'actual'),
        'setpoint/joint_feedback':     ('joint_angle', 'setpoint'),
        'drake/joint_feedback':     ('joint_angle', 'drake'),
        'joint_torque_drake_feedback':     ('joint_torque', 'drake'),
    }

    data = {
        'motor_pos':   {'actual': [], 'setpoint': [], 'drake': []},
        'joint_angle': {'actual': [], 'setpoint': [], 'drake': []},
        'joint_torque': {'drake': []},
    }

    reader = rosbag2_py.SequentialReader()

    if FILE == 'latest':
        reader.open(
            rosbag2_py.StorageOptions(
                uri=max(glob.glob(DATA_FOLDER + '*')),
                storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
    else:
        reader.open(
            rosbag2_py.StorageOptions(
                uri=DATA_FOLDER + FILE,
                storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))

    while reader.has_next():
        topic, raw, t = reader.read_next()
        if topic in TOPICS:
            group, series = TOPICS[topic]  # must unpack tuple
            msg = deserialize_message(raw, MotorFeedback)
            positions = list(msg.motor_positions)
            if len(positions) == len(JOINT_LABELS):
                data[group][series].append((t / 1e9, positions))

    print("Data loaded:")
    for group, series_dict in data.items():
        for series, values in series_dict.items():
            print(f"  {group}/{series}: {len(values)} points")

    # Save data to CSV
    save_data_to_csv(data, JOINT_LABELS, DATA_FOLDER + 'plotted_data.csv')

    fig, axes = plt.subplots(len(JOINT_LABELS), len(data), figsize=(14, 8), sharex=True)

    for col, (group, title) in enumerate([('motor_pos', 'Motor Position'),
                                        ('joint_angle', 'Joint Angle'),
                                        ('joint_torque', 'Joint Torque')]):

        for i, label in enumerate(JOINT_LABELS):
            ax = axes[i, col]
            for series_name, color in [('actual', 'tab:blue'), ('setpoint', 'tab:red'), ('drake', 'tab:green')]:
                if not data[group].get(series_name):
                    continue
                filtered = [(t, v) for t, v in data[group][series_name]]
                ts, vals_raw = zip(*filtered)
                vals = np.array(vals_raw)
                ax.plot(ts, np.rad2deg(vals[:, i]), label=series_name, color=color,
                        linestyle='-' if (series_name == 'actual') else '--')
            if col == 0:
                ax.set_ylabel(label)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            if i == 0:
                ax.set_title(title)
        axes[-1, col].set_xlabel('Time (s)')

    fig.suptitle('Motor Position & Joint Angle: Setpoint vs Actual')
    plt.tight_layout()
    plt.show()

else:
    TOPICS = {
        'motor_pos_actual_feedback':   ('motor_pos', 'actual'),
        'motor_pos_setpoint_feedback': ('motor_pos', 'setpoint'),
        'motor_pos_drake_feedback': ('motor_pos', 'drake'),
        'actual/joint_feedback':       ('joint_angle', 'actual'),
        'setpoint/joint_feedback':     ('joint_angle', 'setpoint'),
        'drake/joint_feedback':     ('joint_angle', 'drake'),
    }

    data = {
        'motor_pos':   {'actual': [], 'setpoint': [], 'drake': []},
        'joint_angle': {'actual': [], 'setpoint': [], 'drake': []},
    }

    reader = rosbag2_py.SequentialReader()

    if FILE == 'latest':
        reader.open(
            rosbag2_py.StorageOptions(
                uri=max(glob.glob(DATA_FOLDER + '*')),
                storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
    else:
        reader.open(
            rosbag2_py.StorageOptions(
                uri=DATA_FOLDER + FILE,
                storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))

    while reader.has_next():
        topic, raw, t = reader.read_next()
        if topic in TOPICS:
            group, series = TOPICS[topic]  # must unpack tuple
            msg = deserialize_message(raw, MotorFeedback)
            positions = list(msg.motor_positions)
            if len(positions) == len(JOINT_LABELS):
                data[group][series].append((t / 1e9, positions))

    print("Data loaded:")
    for group, series_dict in data.items():
        for series, values in series_dict.items():
            print(f"  {group}/{series}: {len(values)} points")

    # Save data to CSV
    save_data_to_csv(data, JOINT_LABELS, DATA_FOLDER + 'plotted_data.csv')

    fig, axes = plt.subplots(len(JOINT_LABELS), len(data), figsize=(14, 8), sharex=True)

    for col, (group, title) in enumerate([('motor_pos', 'Motor Position'),
                                        ('joint_angle', 'Joint Angle')]):

        for i, label in enumerate(JOINT_LABELS):
            ax = axes[i, col]
            for series_name, color in [('actual', 'tab:blue'), ('setpoint', 'tab:red'), ('drake', 'tab:green')]:
                if not data[group].get(series_name):
                    continue
                filtered = [(t, v) for t, v in data[group][series_name]]
                ts, vals_raw = zip(*filtered)
                vals = np.array(vals_raw)
                ax.plot(ts, np.rad2deg(vals[:, i]), label=series_name, color=color,
                        linestyle='-' if (series_name == 'actual') else '--')
            if col == 0:
                ax.set_ylabel(label)
            ax.legend(loc='upper right')
            ax.grid(True, alpha=0.3)
            if i == 0:
                ax.set_title(title)
        axes[-1, col].set_xlabel('Time (s)')

    fig.suptitle('Motor Position & Joint Angle: Setpoint vs Actual')
    plt.tight_layout()
    plt.show()
