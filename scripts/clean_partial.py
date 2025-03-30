#!/usr/bin/env python3
import os
import shutil
import xml.etree.ElementTree as ET
import argparse


def get_folders_above_threshold(directory, threshold):
    # List to hold the folder names
    folder_names = []

    # Walk through the directory
    for folder_name in os.listdir(directory):
        folder_path = os.path.join(directory, folder_name)

        # Check if it's a directory and the folder name starts with an integer followed by an underscore
        if os.path.isdir(folder_path):
            parts = folder_name.split("_")
            if parts[0].isdigit():  # Ensure the first part is a number
                number = int(parts[0])
                # Check if the folder number is above the threshold
                if number >= threshold:
                    folder_names.append(folder_name)

    return folder_names


def get_package_name(package_xml_path):
    try:
        tree = ET.parse(package_xml_path)
        root = tree.getroot()
        package_name = root.find("./name")
        return package_name.text if package_name is not None else None
    except ET.ParseError:
        return None


def get_package_names_from_dirs(base_dirs):
    package_dirs = []

    for base_dir in base_dirs:
        for root, _, files in os.walk(base_dir):
            if "package.xml" in files:
                package_xml_path = os.path.join(root, "package.xml")
                package_name = get_package_name(package_xml_path)
                if package_name:
                    package_dirs.append(package_name)

    return package_dirs


def delete_matching_dirs(target_dirs, package_names):
    for target_dir in target_dirs:
        for root, dirs, _ in os.walk(target_dir, topdown=False):
            for dir_name in dirs:
                if dir_name in package_names:
                    dir_path = os.path.join(root, dir_name)
                    print(f"Deleting: {dir_path}")
                    try:
                        if os.path.islink(dir_path):
                            os.unlink(dir_path)  # Remove symbolic link
                        else:
                            shutil.rmtree(dir_path)
                    except Exception as e:
                        print(f"Failed to delete {dir_path}: {e}")


DIR_IGNORE_LIST = ["02_V5", "09_External"]
PKG_IGNORE_LIST = ["btcpp_ros2_samples"]
DELETION_DIRS = ["build", "install", "log"]

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Delete packages above a certain level in the folder heirarchy."
    )
    parser.add_argument(
        "-y",
        action="store_true",
        help="Delete directories without asking for confirmation",
    )
    parser.add_argument(
        "--threshold",
        "-t",
        required=True,
        type=int,
        default=float("inf"),
        help="Level to begin deletion. e.x. 3 will delete all packages including and above 03_ROS",
    )
    args = parser.parse_args()

    # Get the value of repo base
    VEXU_HOME_DIR = os.environ.get("VEXU_HOME")

    if VEXU_HOME_DIR is None:
        print("Error: VEXU_HOME environment variable is not set!")
        exit(1)

    for dir in DIR_IGNORE_LIST:
        print("Ignoring " + dir + " directory.")
    print("")

    DELETION_DIRS = [VEXU_HOME_DIR + "/" + item for item in DELETION_DIRS]

    sendit = False

    # Get filtered subdirectory names
    search_dirs = get_folders_above_threshold(VEXU_HOME_DIR, args.threshold)
    search_dirs = [item for item in search_dirs if item not in DIR_IGNORE_LIST]

    # Get filtered pkg names
    package_names = get_package_names_from_dirs(search_dirs)
    package_names = [item for item in package_names if item not in PKG_IGNORE_LIST]

    print("Going to delete the following packages:")
    print(package_names)
    print("")

    if not sendit:
        user_input = input("Continue? Y/n: ")
        if user_input != "Y":
            print("Aborting...")
            exit(0)

    delete_matching_dirs(DELETION_DIRS, package_names)
