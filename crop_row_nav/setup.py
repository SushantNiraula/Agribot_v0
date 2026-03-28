from setuptools import find_packages, setup

package_name = 'crop_row_nav'

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
    maintainer='agribot',
    maintainer_email='sushantniraula01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'crop_row_detector = crop_row_nav.crop_row_detector:main',
            'crop_row_navigator = crop_row_nav.crop_row_navigator:main',
            'image_throttler = crop_row_nav.image_throttler:main',
            'green_follower = crop_row_nav.green_follower:main', # <-- FIXED TYPO HERE
        ],
    },
)