import datetime
import rosbag
import pandas as pd
import matplotlib.pyplot as plt

def load_rosbag(bag_path, topic):
    bag = rosbag.Bag(bag_path)

    data = {'timestamp': [], 'latitude': [], 'longitude': [], 'altitude': [], 'easting': [], 'northing': []}

    for _, msg, t in bag.read_messages(topics=[topic]):
        data['timestamp'].append(t.to_sec())
        data['latitude'].append(msg.latitude)
        data['longitude'].append(msg.longitude)
        data['altitude'].append(msg.altitude)
        data['easting'].append(msg.utm_easting)
        data['northing'].append(msg.utm_northing)

    bag.close()

    return pd.DataFrame(data)

def analyze_moving_data(df, location_type):
    # Convert timestamp to NumPy array
    df['timestamp'] = df['timestamp'].apply(lambda x: datetime.datetime.fromtimestamp(x))
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    df['timestamp'] = df['timestamp'].apply(lambda x: x.to_pydatetime())

    # Convert columns to NumPy arrays
    easting = df['easting'].to_numpy()
    northing = df['northing'].to_numpy()
    altitude = df['altitude'].to_numpy()

    # Convert timestamp to NumPy array
    timestamps = df['timestamp'].values

    print("Timestamps:", timestamps)  # Check timestamps

    # Moving data scatterplot
    plt.figure()
    plt.scatter(easting, northing, label=f'{location_type} Data', marker='o')
    plt.title(f'Moving {location_type} Data')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()

    print("Scatter plot created")

    # Moving data altitude plot
    plt.figure()
    plt.plot(timestamps, altitude, label=f'{location_type} Data', marker='o')
    plt.title(f'Moving {location_type} Data Altitude vs. Time')
    plt.xlabel('Timestamp')
    plt.ylabel('Altitude (meters)')
    plt.legend()

    print("Altitude plot created")

if __name__ == '__main__':
    # Load ROS bag data
    walking_df = load_rosbag('/home/karthikkoduru1/catkin_ws/src/gps_driver/src/data/walking1.bag', '/chatter')

    print("DataFrame shape:", walking_df.shape)  # Check DataFrame shape

    # Analyze moving data
    analyze_moving_data(walking_df, 'Walking')

    print("Plots created")

    # Show the plots
    plt.show()
