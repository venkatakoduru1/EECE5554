import datetime
import rosbag
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def load_rosbag(bag_path, topic):
    bag = rosbag.Bag(bag_path)

    data = {'timestamp': [], 'latitude': [], 'longitude': [], 'altitude': [], 'easting': [], 'northing': []}

    initial_time = None  # Store the initial timestamp

    for _, msg, t in bag.read_messages(topics=[topic]):
        if initial_time is None:
            initial_time = t.to_sec()  # Store the initial timestamp

        data['timestamp'].append(t.to_sec() - initial_time)  # Calculate time difference in seconds
        data['latitude'].append(msg.latitude)
        data['longitude'].append(msg.longitude)
        data['altitude'].append(msg.altitude)
        data['easting'].append(msg.utm_easting)
        data['northing'].append(msg.utm_northing)

    bag.close()

    return pd.DataFrame(data)


def analyze_stationary_data(open_df, occluded_df):
    # Convert 'timestamp' column to NumPy array
    open_timestamp = open_df['timestamp'].to_numpy()
    occluded_timestamp = occluded_df['timestamp'].to_numpy()

    # Convert 'altitude' column to NumPy array
    open_altitude = open_df['altitude'].to_numpy()
    occluded_altitude = occluded_df['altitude'].to_numpy()

    # Convert 'easting' and 'northing' columns to NumPy arrays
    open_easting = open_df['easting'].to_numpy()
    open_northing = open_df['northing'].to_numpy()
    occluded_easting = occluded_df['easting'].to_numpy()
    occluded_northing = occluded_df['northing'].to_numpy()

    # Calculate centroids for Open data
    open_centroid = {
        'easting': np.mean(open_easting),
        'northing': np.mean(open_northing)
    }

    # Calculate centroids for Occluded data
    occluded_centroid = {
        'easting': np.mean(occluded_easting),
        'northing': np.mean(occluded_northing)
    }

    # Plot northing vs. easting scatterplots for Open data
    plt.figure()
    plt.scatter(open_easting, open_northing, label='Open Spot', marker='o', color='blue')
    plt.scatter(open_centroid['easting'], open_centroid['northing'], label='Open Centroid', marker='x', color='green')
    plt.title('Stationary Northing vs. Easting Scatterplots - Open Data')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()

    # Plot northing vs. easting scatterplots for Occluded data
    plt.figure()
    plt.scatter(occluded_easting, occluded_northing, label='Occluded Spot', marker='o', color='red')
    plt.scatter(occluded_centroid['easting'], occluded_centroid['northing'], label='Occluded Centroid', marker='x', color='orange')
    plt.title('Stationary Northing vs. Easting Scatterplots - Occluded Data')
    plt.xlabel('Easting (meters)')
    plt.ylabel('Northing (meters)')
    plt.legend()

    # Calculate Euclidean distance from each point to the centroid for Open data
    open_distances = np.sqrt((open_easting - open_centroid['easting'])**2 + (open_northing - open_centroid['northing'])**2)

    # Calculate Euclidean distance from each point to the centroid for Occluded data
    occluded_distances = np.sqrt((occluded_easting - occluded_centroid['easting'])**2 + (occluded_northing - occluded_centroid['northing'])**2)


    distance1=np.mean(open_distances)
    distance2=np.mean(occluded_distances)

    print("error for open data",distance1)
    print("error for occluded data",distance2)

      
    # Plot histogram of distances for Open data
    plt.figure()
    plt.hist(open_distances, bins=20, alpha=0.5, label='Open Spot - Distance from Centroid', color='blue')
    #plt.hist(occluded_distances, bins=20, alpha=0.5, label='Occluded Spot - Distance from Centroid', color='red')
    plt.title('Euclidean Distance from Centroid Histogram')
    plt.xlabel('Distance')
    plt.ylabel('Frequency')
    plt.legend()

    # Plot histogram of distances for Occluded data
    plt.figure()
    #plt.hist(open_distances, bins=20, alpha=0.5, label='Open Spot - Distance from Centroid', color='blue')
    plt.hist(occluded_distances, bins=20, alpha=0.5, label='Occluded Spot - Distance from Centroid', color='red')
    plt.title('Euclidean Distance from Centroid Histogram')
    plt.xlabel('Distance')
    plt.ylabel('Frequency')
    plt.legend()

    # Plot altitude vs. time plot for both Open data
    plt.figure()
    plt.plot(open_timestamp, open_altitude, label='Open Spot', marker='o', color='blue')
    plt.title('Stationary Altitude vs. Time Plot - Open Data')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Altitude (meters)')
    plt.legend()

    # Plot altitude vs. time plot for both Occluded data
    plt.figure()
    plt.plot(occluded_timestamp, occluded_altitude, label='Occluded Spot', marker='x', color='red')
    plt.title('Stationary Altitude vs. Time Plot - Occluded Data')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Altitude (meters)')
    plt.legend()

    plt.show()


if __name__ == '__main__':
    open_spot_df = load_rosbag('/home/karthikkoduru1/lab2_bagfiles/openRTK.bag', '/gps')
    occluded_spot_df = load_rosbag('/home/karthikkoduru1/lab2_bagfiles/occludedRTK.bag', '/gps')

    analyze_stationary_data(open_spot_df, occluded_spot_df)
