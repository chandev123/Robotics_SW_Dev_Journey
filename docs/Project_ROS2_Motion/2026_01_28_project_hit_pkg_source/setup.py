from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'project_hit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # [기존 유지] Config 파일
        ('share/' + package_name + '/config', ['config/pose_config.yaml']),
        
        # [추가됨] Firebase 인증 키 (JSON 파일)
        ('share/' + package_name, glob('*.json')),
        
        # [추가됨] Frontend 파일 (HTML, CSS, JS)
        # frontend 폴더 구조를 그대로 share 폴더에 복사합니다.
        ('share/' + package_name + '/frontend', glob('frontend/*.html')),
        ('share/' + package_name + '/frontend/css', glob('frontend/css/*')),
        ('share/' + package_name + '/frontend/js', glob('frontend/js/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='markch',
    maintainer_email='cocoffeechan09@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hw_posx = project_hit.hw_node.hw_node_posx:main',
            'hw_force = project_hit.hw_node.hw_node_force:main',
            'task = project_hit.task_node:main',
            
            'ui = project_hit.ui_node:main',
            'posx = project_hit.hw_node_posx:main',
            
            'motion = project_hit.motion_control:main',
            'insert = project_hit.insert_card_slot:main',
        ],
    },
)
