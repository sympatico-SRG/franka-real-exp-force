from scripts.check_data_server_resources import find_camera_owners


def test_finds_realsense_publisher_process() -> None:
    process_table = (
        "101 /opt/ros/jazzy/lib/franka_realsense_camera_publisher/"
        "realsense_camera_publisher --ros-args\n"
    )

    assert find_camera_owners(process_table) == [process_table.strip()]


def test_finds_stale_realsense_launch_parent() -> None:
    process_table = (
        "202 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch "
        "franka_realsense_camera_publisher cameras.launch.py\n"
    )

    assert find_camera_owners(process_table) == [process_table.strip()]


def test_ignores_unrelated_processes() -> None:
    assert find_camera_owners("303 python data_collection/lerobot_collection.py\n") == []
