import os
import pandas as pd
import matplotlib.pyplot as plt
import roslib.packages

# Function to get the path to the ROS package
def get_ros_package_path(package_name):
    try:
        package_path = roslib.packages.get_pkg_dir(package_name)
        print(f"Path to package {package_name}: {package_path}")  # Debug output
        return package_path
    except roslib.packages.InvalidROSPkgException:
        print(f"Error: ROS package {package_name} was not found.")  # Debug output
        return None

# Package name
ros_package_name = "tars_robot"
data_subfolder = "data"

# Path to the "tars_robot" ROS package
ros_package_path = get_ros_package_path(ros_package_name)

# Check if the path was found
if ros_package_path is not None:
    # Add path to the data folder
    main_folder = os.path.join(ros_package_path, data_subfolder)
    
    # Function to search all subfolders for CSV files and create plots
    def search_subfolders_and_create_plots(main_folder):
        for root, dirs, files in os.walk(main_folder):
            print(f"Searching folder: {root}")  # Debug output
            for file in files:
                if file.endswith(".csv"):
                    csv_path = os.path.join(root, file)
                    plot_path = os.path.splitext(csv_path)[0] + ".png"
                    
                    # Skip if a plot already exists
                    if os.path.exists(plot_path):
                        continue
                    
                    try:
                        # Read the CSV file
                        df = pd.read_csv(csv_path)
                        
                        # Check if the necessary columns are present
                        if 'Distance(mm)' in df.columns and 'AbsoluteForce' in df.columns:
                            # Create the plot
                            print(f"Creating plot for {file}")  # Debug output
                            plt.figure(figsize=(10, 6))
                            plt.plot(df['Distance(mm)'], df['AbsoluteForce'])
                            plt.xlabel('Distance [mm]')
                            plt.ylabel('AbsoluteForce [N]')
                            plt.title(f'Plot for {file}')
                            plt.legend()
                            plt.grid(True)
                            
                            # Save the plot as an image
                            plt.savefig(plot_path)
                            print(f"Plot saved as {plot_path}")  # Debug output
                            plt.close()
                        else:
                            print(f"Required columns are missing in {csv_path}")  # Debug output
                    except Exception as e:
                        print(f"Error processing {csv_path}: {e}")

    # Call the function with the path to the ROS package
    search_subfolders_and_create_plots(main_folder)
else:
    print(f"Path for ROS package {ros_package_name} could not be found.")
