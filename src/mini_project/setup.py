from setuptools import find_packages, setup

package_name = 'mini_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hv-05',
    maintainer_email='jungsub27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'capture_comp_image = mini_project.capture_comp_image:main',
            'capture_image = mini_project.capture_image:main',
            'depth_checker_mouse_click = mini_project.depth_checker_mouse_click:main',
            'yolo_detection = mini_project.yolo_detection:main',
            'record_bag = mini_project.record_bag:main',
            'bag_to_video = mini_project.bag_to_video:main',
            'video_to_frames = mini_project.video_to_frames:main',
            'depth_floor_ransac = mini_project.depth_floor_ransac:main',
            'align_check = mini_project.align_check:main',
            'webcam_calib = mini_project.webcam_calib:main',
            'mission = mini_project.mission:main',
            'track_report = mini_project.track_report:main',
        ],
    },
)
