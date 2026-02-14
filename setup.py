from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # Construct the full local path
            local_path = os.path.join(path, filename)
            # Construct the destination path (share/package_name/path)
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [local_path]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.*'))),
        (os.path.join('share', package_name, 'nodes'), glob(os.path.join('nodes', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.*'))),
        *package_files('models'),
        # (os.path.join('share', package_name, 'models/circular_maze'), glob(os.path.join('models/circular_maze', '*.*'))),
        # (os.path.join('share', package_name, 'models/circular_maze/meshes'), glob(os.path.join('models/circular_maze/meshes', '*.*'))),
        # (os.path.join('share', package_name, 'models/square_maze'), glob(os.path.join('models/square_maze', '*.*'))),
        # (os.path.join('share', package_name, 'models/square_maze/meshes'), glob(os.path.join('models/square_maze/meshes', '*.*'))),
        # (os.path.join('share', package_name, 'models/simple_maze'), glob(os.path.join('models/simple_maze', '*.*'))),
        # (os.path.join('share', package_name, 'models/simple_maze/meshes'), glob(os.path.join('models/simple_maze/meshes', '*.*'))),
        # (os.path.join('share', package_name, 'models/turtlebot'), glob(os.path.join('models/turtlebot', '*.*'))),
        # (os.path.join('share', package_name, 'models/turtlebot/meshes'), glob(os.path.join('models/turtlebot/meshes', '*.*'))),
        # (os.path.join('share', package_name, 'models/maze_4_metal_6x6'), glob(os.path.join('models/maze_4_metal_6x6', '*.*'))),
        # (os.path.join('share', package_name, 'models/maze_4_metal_6x6/meshes'), glob(os.path.join('models/maze_4_metal_6x6/meshes', '*.*'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavan',
    maintainer_email='18pavansai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arrow_teleop = turtlebot.arrow_teleop:main',
        ],
    },
)
